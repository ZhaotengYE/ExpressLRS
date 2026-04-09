"""
LDTP (LoRa Dual-Channel Transport Protocol) — Bit-accurate Implementation

Frame formats per LDTP Spec v1.0:

Unreliable:  [FC:1] [ACK:0|2] [Payload:N]
Reliable:    [FC:1] [ACK:0|2] [SeqCtl:1] [Payload:N]
ACK-only:    [FC:1] [ACK:2]

FC byte:     R(1) A(1) F(1) TYPE/STREAM(3) VER(2)
ACK block:   SID(3) CACK(6) SACK(7)  — 16 bits big-endian
SeqCtl byte: SEQ(6) FRAG(2)
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SEQ_MOD      = 64        # 6-bit sequence space
WINDOW_SIZE  = 8
MAX_RETRIES  = 5
RTO_TICKS    = 4         # initial retransmission timeout in ticks (used as RTO_INITIAL before RTT samples)
RTO_MAX_EXP  = 4         # max exponent for exponential backoff (2^4 = 16x)
RTO_MIN      = 2         # minimum RTO in ticks (floor)
RTO_MAX      = 100       # maximum RTO in ticks (cap, ~5s at 20Hz)
ACK_DELAY_TICKS = 2      # if ACK pending for this many ticks with no piggyback opportunity, send ACK-only
PROTO_VER    = 0

# ═══════════════════════════════════════════════════════════════════════════
# Frame Encoding
# ═══════════════════════════════════════════════════════════════════════════

def _fc_byte(reliable: int, ack: int, frag_more: int, typ: int) -> int:
    return (reliable << 7) | (ack << 6) | (frag_more << 5) | ((typ & 7) << 2) | PROTO_VER

def _ack_bytes(sid: int, cack: int, sack: int) -> bytes:
    w = ((sid & 7) << 13) | ((cack & 0x3F) << 7) | (sack & 0x7F)
    return bytes([w >> 8, w & 0xFF])

def _seq_byte(seq: int, frag: int) -> int:
    return ((seq & 0x3F) << 2) | (frag & 3)

def build_unreliable(type_id: int, payload: bytes,
                     ack: Optional[Tuple[int,int,int]] = None) -> bytes:
    fc = _fc_byte(0, 1 if ack else 0, 0, type_id)
    frame = bytes([fc])
    if ack:
        frame += _ack_bytes(*ack)
    frame += payload
    return frame

def build_reliable(stream_id: int, seq: int, frag: int, more: bool,
                   payload: bytes,
                   ack: Optional[Tuple[int,int,int]] = None) -> bytes:
    fc = _fc_byte(1, 1 if ack else 0, 1 if more else 0, stream_id)
    frame = bytes([fc])
    if ack:
        frame += _ack_bytes(*ack)
    frame += bytes([_seq_byte(seq, frag)])
    frame += payload
    return frame

def build_ack_only(ack: Tuple[int,int,int]) -> bytes:
    fc = _fc_byte(0, 1, 0, 7)          # TYPE=7 reserved for ACK-only
    return bytes([fc]) + _ack_bytes(*ack)

# ═══════════════════════════════════════════════════════════════════════════
# Frame Decoding
# ═══════════════════════════════════════════════════════════════════════════

def decode_frame(data: bytes) -> Optional[dict]:
    """Decode raw bytes → dict with every protocol field."""
    if not data:
        return None
    fc = data[0]
    r = {
        'reliable':   (fc >> 7) & 1,
        'ack_present': (fc >> 6) & 1,
        'more_frag':  (fc >> 5) & 1,
        'type_or_stream': (fc >> 2) & 7,
        'version':    fc & 3,
        'raw_hex':    data.hex(),
        'total_len':  len(data),
    }
    off = 1

    if r['ack_present']:
        if len(data) < off + 2:
            return None
        w = (data[off] << 8) | data[off+1]
        r['ack_sid']  = (w >> 13) & 7
        r['ack_cack'] = (w >> 7) & 0x3F
        r['ack_sack'] = w & 0x7F
        off += 2

    if r['reliable']:
        if len(data) < off + 1:
            return None
        sc = data[off]
        r['seq']  = (sc >> 2) & 0x3F
        r['frag'] = sc & 3
        off += 1

    r['payload']      = data[off:]
    r['payload_hex']  = data[off:].hex()
    r['header_bytes'] = off
    return r

# ═══════════════════════════════════════════════════════════════════════════
# Sequence-number helpers (modular arithmetic)
# ═══════════════════════════════════════════════════════════════════════════

def _seq_dist(a, b):
    """How far ahead a is from b  (0 .. SEQ_MOD-1)."""
    return (a - b) % SEQ_MOD

def _seq_in_range(seq, base, count):
    """Is seq in [base, base+count) mod SEQ_MOD?"""
    return _seq_dist(seq, base) < count

# ═══════════════════════════════════════════════════════════════════════════
# Reliable Sender  (per-stream, Selective-Repeat ARQ)
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class _SenderEntry:
    payload: bytes
    label: str              # human-readable description
    frag: int = 0
    more_frag: bool = False
    sent: bool = False
    acked: bool = False
    retx_count: int = 0
    timer: int = 0
    needs_retx: bool = False
    sent_tick: int = -1     # first TX tick (for latency tracking)

class ReliableSender:
    def __init__(self, stream_id: int):
        self.sid = stream_id
        self.next_seq = 0
        self.base_seq = 0
        self.entries: Dict[int, _SenderEntry] = {}
        self.queue: list = []           # [(payload, label)]
        self.delivered: list = []       # [(seq, label, latency)]
        self.failed: list = []          # [(seq, label)]
        # Adaptive RTO state (Jacobson/Karels algorithm)
        self.rtt_est = 0.0             # smoothed RTT estimate (ticks)
        self.rtt_dev = 0.0             # RTT deviation estimate (ticks)
        self.rtt_initialized = False   # first sample bootstraps the estimator
        self.rto = RTO_TICKS           # current base RTO (before backoff)

    # -- public -----------------------------------------------------------
    def queue_message(self, payload: bytes, label: str = ""):
        self.queue.append((payload, label))

    def has_work(self) -> bool:
        return bool(self.entries) or bool(self.queue)

    def tick(self, now: int) -> List[dict]:
        """Advance timers. Returns list of event dicts."""
        self._admit()
        events = []
        for seq in list(self.entries):
            e = self.entries[seq]
            if e.sent and not e.acked and not e.needs_retx:
                e.timer -= 1
                if e.timer <= 0:
                    e.retx_count += 1
                    if e.retx_count > MAX_RETRIES:
                        events.append({'event': 'reliable_failed', 'seq': seq,
                                       'label': e.label})
                        self.failed.append((seq, e.label))
                        del self.entries[seq]
                        self._advance_base()
                    else:
                        e.needs_retx = True
                        events.append({'event': 'retx_timeout', 'seq': seq,
                                       'attempt': e.retx_count})
        return events

    def get_frame(self, ack_info=None, now: int = 0):
        """Return (frame_bytes, meta_dict) or (None, None)."""
        self._admit()
        # priority: retransmission > new
        for seq in self._iter_window():
            e = self.entries.get(seq)
            if not e or e.acked:
                continue
            if e.needs_retx:
                return self._tx(seq, e, ack_info, now, is_retx=True)
        for seq in self._iter_window():
            e = self.entries.get(seq)
            if not e or e.acked or e.sent:
                continue
            return self._tx(seq, e, ack_info, now, is_retx=False)
        return None, None

    def process_ack(self, sid, cack, sack, now: int = 0) -> List[dict]:
        if sid != self.sid:
            return []
        events = []

        def _ack_entry(seq):
            e = self.entries[seq]
            e.acked = True
            lat = now - e.sent_tick if e.sent_tick >= 0 else 0
            self.delivered.append((seq, e.label, lat))
            events.append({'event': 'reliable_delivered', 'seq': seq,
                           'label': e.label, 'latency': lat})
            # Adaptive RTO: sample RTT only from non-retransmitted packets (Karn's algorithm)
            if e.retx_count == 0 and e.sent_tick >= 0 and lat > 0:
                self._update_rtt(lat)

        # cumulative
        seq = self.base_seq
        guard = 0
        while seq != (cack + 1) % SEQ_MOD and guard < SEQ_MOD:
            if seq in self.entries and not self.entries[seq].acked:
                _ack_entry(seq)
            seq = (seq + 1) % SEQ_MOD
            guard += 1
        # selective
        for i in range(7):
            sseq = (cack + 1 + i) % SEQ_MOD
            if sack & (1 << (6 - i)):
                if sseq in self.entries and not self.entries[sseq].acked:
                    _ack_entry(sseq)
        self._advance_base()
        return events

    def _update_rtt(self, sample: float):
        """Update RTT estimator using Jacobson/Karels algorithm."""
        if not self.rtt_initialized:
            self.rtt_est = sample
            self.rtt_dev = sample / 2.0
            self.rtt_initialized = True
        else:
            err = sample - self.rtt_est
            self.rtt_est = 0.875 * self.rtt_est + 0.125 * sample
            self.rtt_dev = 0.75 * self.rtt_dev + 0.25 * abs(err)
        # RTO = RTT_est + 4 × RTT_dev, clamped to [RTO_MIN, RTO_MAX]
        self.rto = max(RTO_MIN, min(RTO_MAX, round(self.rtt_est + 4 * self.rtt_dev)))

    # -- private ----------------------------------------------------------
    def _admit(self):
        while self.queue and self._window_count() < WINDOW_SIZE:
            payload, label = self.queue.pop(0)
            seq = self.next_seq
            self.next_seq = (self.next_seq + 1) % SEQ_MOD
            self.entries[seq] = _SenderEntry(payload=payload, label=label)

    def _tx(self, seq, e: _SenderEntry, ack_info, now, is_retx):
        e.needs_retx = False
        e.sent = True
        if e.sent_tick < 0:
            e.sent_tick = now
        # Adaptive RTO with exponential backoff
        e.timer = min(RTO_MAX, self.rto * (2 ** min(e.retx_count, RTO_MAX_EXP)))
        frame = build_reliable(self.sid, seq, e.frag, e.more_frag, e.payload, ack_info)
        meta = {'seq': seq, 'label': e.label, 'is_retx': is_retx,
                'retx_count': e.retx_count, 'rto': self.rto}
        return frame, meta

    def _window_count(self):
        return _seq_dist(self.next_seq, self.base_seq)

    def _iter_window(self):
        s = self.base_seq
        for _ in range(self._window_count()):
            yield s
            s = (s + 1) % SEQ_MOD

    def _advance_base(self):
        while self.base_seq in self.entries and self.entries[self.base_seq].acked:
            del self.entries[self.base_seq]
            self.base_seq = (self.base_seq + 1) % SEQ_MOD
        while self.base_seq not in self.entries and self.base_seq != self.next_seq:
            self.base_seq = (self.base_seq + 1) % SEQ_MOD

# ═══════════════════════════════════════════════════════════════════════════
# Reliable Receiver (per-stream)
# ═══════════════════════════════════════════════════════════════════════════

class ReliableReceiver:
    def __init__(self, stream_id: int):
        self.sid = stream_id
        self.expected = 0
        self.buf: Dict[int, bytes] = {}
        self.delivered: list = []           # [payload_bytes, ...]
        self._has_data = False
        self._ack_pending_since = -1       # tick when ACK became pending (-1 = no ACK pending)
        self._last_ack_sent = -1           # tick when last ACK was sent (piggyback or standalone)

    def receive(self, seq: int, frag: int, more: bool, payload: bytes, now: int = 0) -> bool:
        dist = _seq_dist(seq, self.expected)
        if dist >= SEQ_MOD // 2:
            return False        # old dup
        if seq in self.buf:
            return False        # dup
        self._has_data = True
        self._ack_pending_since = now      # new data → fresh ACK needed
        self.buf[seq] = payload
        # deliver in order
        while self.expected in self.buf:
            self.delivered.append(self.buf.pop(self.expected))
            self.expected = (self.expected + 1) % SEQ_MOD
        return True

    def get_ack(self) -> Optional[Tuple[int,int,int]]:
        if not self._has_data:
            return None
        cack = (self.expected - 1) % SEQ_MOD
        sack = 0
        for i in range(7):
            if (self.expected + i) % SEQ_MOD in self.buf:
                sack |= 1 << (6 - i)
        return (self.sid, cack, sack)

    def mark_ack_sent(self, now: int = 0):
        """Call this after an ACK has been sent (piggybacked or standalone)."""
        self._ack_pending_since = -1
        self._last_ack_sent = now

    def needs_standalone_ack(self, now: int) -> bool:
        """Returns True if an ACK-only frame should be sent (no piggyback opportunity for too long)."""
        if not self._has_data or self._ack_pending_since < 0:
            return False
        return (now - self._ack_pending_since) >= ACK_DELAY_TICKS

    def pop_delivered(self) -> list:
        out = self.delivered
        self.delivered = []
        return out


# ═══════════════════════════════════════════════════════════════════════════
# Link Quality Monitor (app-layer indicator)
# ═══════════════════════════════════════════════════════════════════════════

class LinkQualityMonitor:
    """Sliding-window link quality tracker. Reports delivery ratio over recent packets."""

    def __init__(self, window_size: int = 50):
        self.window_size = window_size
        self._history: list = []     # list of bools: True = delivered, False = lost
        self.total_sent = 0
        self.total_lost = 0

    def record(self, delivered: bool):
        self._history.append(delivered)
        self.total_sent += 1
        if not delivered:
            self.total_lost += 1
        # trim to window
        if len(self._history) > self.window_size:
            self._history.pop(0)

    @property
    def quality(self) -> float:
        """Returns link quality as 0.0 (all lost) to 1.0 (all delivered)."""
        if not self._history:
            return 1.0
        return sum(self._history) / len(self._history)

    @property
    def quality_pct(self) -> int:
        """Returns link quality as integer percentage 0-100."""
        return round(self.quality * 100)

    @property
    def total_quality(self) -> float:
        """Overall quality since start."""
        if self.total_sent == 0:
            return 1.0
        return (self.total_sent - self.total_lost) / self.total_sent
