# LoRa Dual-Channel Transport Protocol (LDTP) Specification v1.0

## 1. Overview

LDTP is a lightweight transport protocol designed for LoRa full-duplex links (separate uplink and downlink frequencies). It provides two logical channels over a single radio direction:

- **Unreliable Channel**: Fire-and-forget delivery. No acknowledgment, no retransmission. Suitable for periodic state data (RC channels, telemetry) where freshness matters more than completeness.
- **Reliable Channel**: Acknowledged delivery with selective retransmission. Suitable for commands, configuration, file transfer, and any data that must arrive intact.

### 1.1 Design Principles

1. Full duplex assumed: uplink on frequency F1, downlink on frequency F2, simultaneous.
2. Each LoRa packet belongs to exactly ONE channel type (reliable or unreliable). No mixing.
3. ACK information for the reliable channel is piggybacked in reverse-direction packets of either channel type.
4. Minimal header overhead: 1 byte for unreliable, 2 bytes for reliable.
5. LoRa PHY provides packet length detection (explicit header mode) and CRC; the protocol does not duplicate these.

### 1.2 Terminology

| Term | Meaning |
|------|---------|
| GCS | Ground Control Station (ground node) |
| UAV | Unmanned Aerial Vehicle (air node) |
| UL | Uplink: GCS → UAV |
| DL | Downlink: UAV → GCS |
| SEQ | Sequence number |
| CACK | Cumulative ACK sequence number |
| SACK | Selective ACK bitmap |

---

## 2. Physical Layer Assumptions

| Parameter | Requirement |
|-----------|-------------|
| Modulation | LoRa (any SF/BW combination) |
| Duplex | Full duplex via frequency separation (UL on F1, DL on F2) |
| Max PHY payload | Implementation-defined. Protocol works with any size ≥ 8 bytes. Recommended: 64–200 bytes. |
| CRC | LoRa hardware CRC enabled (mandatory) |
| Header mode | Explicit (LoRa provides payload length to receiver) |
| FHSS | Out of scope. May be layered underneath if regulatory requirements demand it. |

---

## 3. Frame Format

Every LoRa packet carries exactly one LDTP frame. The frame structure depends on the channel type (determined by Bit 7 of Byte 0).

### 3.1 Frame Control Byte (Byte 0) — Present in ALL frames

```
Bit 7    : R    — Reliable flag (0 = unreliable channel, 1 = reliable channel)
Bit 6    : A    — ACK block present (0 = no ACK block, 1 = 2-byte ACK block follows)
Bit 5    : F    — More Fragments (only meaningful when R=1; must be 0 when R=0)
Bit 4-2  : TYPE — 3-bit field, interpretation depends on R:
                     R=0: Message Type ID (application-defined, 0–7)
                     R=1: Stream ID (0–7, identifies independent reliable streams)
Bit 1-0  : VER  — Protocol version (00 = v1.0)
```

### 3.2 Unreliable Frame

```
┌──────────────────┬──────────────────────┬─────────────────────┐
│  Frame Control   │  ACK Block (if A=1)  │      Payload        │
│     1 byte       │  0 or 2 bytes        │     N bytes         │
└──────────────────┴──────────────────────┴─────────────────────┘
```

Total overhead: 1 byte (without ACK) or 3 bytes (with piggybacked ACK).

The TYPE field (Bit 4-2) identifies the application message type. Examples:

| TYPE value | UL meaning (example) | DL meaning (example) |
|------------|----------------------|----------------------|
| 0 | RC Channels | Attitude |
| 1 | (reserved) | GPS Position |
| 2 | (reserved) | Battery Status |
| 3 | (reserved) | Link Statistics |
| 4–7 | Application-defined | Application-defined |

The TYPE mapping is application-defined and not enforced by the protocol. The protocol only guarantees delivery semantics (unreliable = no guarantee).

### 3.3 Reliable Frame

```
┌──────────────────┬──────────────────────┬──────────────────┬─────────────────────┐
│  Frame Control   │  ACK Block (if A=1)  │   Seq Control    │      Payload        │
│     1 byte       │  0 or 2 bytes        │     1 byte       │     N bytes         │
└──────────────────┴──────────────────────┴──────────────────┴─────────────────────┘
```

Total overhead: 2 bytes (without ACK) or 4 bytes (with piggybacked ACK).

**Seq Control Byte:**

```
Bit 7-2 : SEQ  — Sequence number (0–63), per Stream ID
Bit 1-0 : FRAG — Fragment index within a multi-fragment message (0–3)
```

Fragment semantics (using F bit from Frame Control + FRAG field):

| F bit | FRAG | Meaning |
|-------|------|---------|
| 0 | 0 | Complete message (single frame, no fragmentation) |
| 1 | 0 | First fragment of a multi-fragment message |
| 1 | 1 | Second fragment, more follow |
| 1 | 2 | Third fragment, more follow |
| 0 | 1 | Second fragment, last |
| 0 | 2 | Third fragment, last |
| 0 | 3 | Fourth fragment, last |
| 1 | 3 | (illegal, reserved) |

This supports messages up to 4 fragments. With a 200-byte LoRa packet and 4-byte overhead, a single message can be up to 4 × 196 = 784 bytes.

All fragments of the same message share the same SEQ number. Each fragment is ACKed individually via the SEQ+FRAG combination, but the application layer only receives the complete reassembled message.

### 3.4 ACK Block (2 bytes, present when A=1)

The ACK block acknowledges reliable frames received from the **opposite direction**. It can be attached to ANY frame (reliable or unreliable) traveling in the reverse direction.

```
Bit 15-13 : SID   — Stream ID being acknowledged (3 bits)
Bit 12-7  : CACK  — Cumulative ACK: highest SEQ where all SEQ ≤ CACK have been received (6 bits)
Bit 6-0   : SACK  — Selective ACK bitmap for SEQ = CACK+1 through CACK+7 (7 bits)
                     Bit 6 = CACK+1, Bit 5 = CACK+2, ... Bit 0 = CACK+7
                     1 = received, 0 = not received
```

Example: Sender transmitted SEQ 10, 11, 12, 13, 14, 15. Receiver got 10, 11, 13, 15.

```
CACK = 11      (everything up to 11 received consecutively)
SACK = 0b1010000
         |||||||
         ||||||+— CACK+7 = 18: 0 (not sent)
         |||||+── CACK+6 = 17: 0 (not sent)
         ||||+─── CACK+5 = 16: 0 (not received)
         |||+──── CACK+4 = 15: 1 (received)
         ||+───── CACK+3 = 14: 0 (not received)
         |+────── CACK+2 = 13: 1 (received)
         +─────── CACK+1 = 12: 0 (not received)
```

Sender now knows: retransmit SEQ 12 and 14 only.

### 3.5 ACK-Only Frame

When no application data needs to be sent but an ACK is pending, a minimal frame can be sent:

```
┌──────────────────┬──────────────────────┐
│  Frame Control   │     ACK Block        │
│  R=0, A=1, TYPE=7│     2 bytes          │
└──────────────────┴──────────────────────┘
```

Total size: 3 bytes. TYPE=7 with zero-length payload is reserved as "ACK-only, no application data."

---

## 4. Reliable Channel Protocol

### 4.1 Sender State (per Stream ID)

Each stream maintains:

```
next_seq      : uint6  — next SEQ to assign (0–63, wrapping)
window_base   : uint6  — oldest unacknowledged SEQ
window[]      : array  — buffer of unacknowledged frames (max WINDOW_SIZE entries)
retx_count[]  : array  — per-SEQ retransmission counter
retx_timer[]  : array  — per-SEQ retransmission timeout
```

### 4.2 Receiver State (per Stream ID)

Each stream maintains:

```
expected_seq  : uint6  — next SEQ expected in-order
recv_bitmap   : uint64 — bitmap of received SEQ numbers (sliding window)
frag_buffer[] : array  — partial fragment reassembly buffers
```

### 4.3 Transmission Procedure

1. Application calls `reliable_send(stream_id, data, len)`.
2. If `len` fits in a single frame payload: assign SEQ = `next_seq++`, FRAG=0, F=0. Enqueue.
3. If `len` requires fragmentation: assign SEQ = `next_seq++`, split into fragments with FRAG=0,1,2,3 and F bits set accordingly. Enqueue all fragments.
4. Sender transmits the oldest un-ACKed frame from the queue (or the highest-priority one if multiple streams are active).
5. Start retransmission timer for the transmitted SEQ.
6. If the frame is ACKed (via CACK or SACK from the receiver), remove from window.
7. If retransmission timer expires, retransmit the frame. Increment `retx_count`.
8. If `retx_count` exceeds `MAX_RETRIES`, notify application of delivery failure. Remove from window.

### 4.4 Reception Procedure

1. Receive a reliable frame. Check SEQ and FRAG.
2. If SEQ is within the receive window, store the fragment.
3. If all fragments of a message (identified by same SEQ, FRAG 0 through last) are received, deliver the reassembled message to the application.
4. Update `expected_seq` and `recv_bitmap`.
5. On next outgoing packet (any channel, any direction), attach ACK block with current CACK and SACK.

### 4.5 Retransmission Timer (Adaptive RTO)

The retransmission timeout (RTO) adapts to measured link conditions using the Jacobson/Karels algorithm:

```
Initial state:
  RTT_est = 0           (no estimate until first sample)
  RTT_dev = 0
  RTO     = RTO_INITIAL (4 ticks at 20Hz = 200ms)

On first RTT sample:
  RTT_est = sample
  RTT_dev = sample / 2
  RTO     = clamp(RTT_est + 4 × RTT_dev, RTO_MIN, RTO_MAX)

On subsequent RTT samples:
  err     = sample − RTT_est
  RTT_est = 0.875 × RTT_est + 0.125 × sample
  RTT_dev = 0.75  × RTT_dev + 0.25  × |err|
  RTO     = clamp(RTT_est + 4 × RTT_dev, RTO_MIN, RTO_MAX)

RTT_sample = tick_ACK_received − tick_frame_sent
  (Only sampled from non-retransmitted frames per Karn's algorithm)

Per-frame timer on transmission:
  timer = min(RTO_MAX, RTO × 2^min(retx_count, RTO_MAX_EXP))
```

Constants:

| Parameter | Value | Description |
|-----------|-------|-------------|
| RTO_INITIAL | 4 ticks (200ms @ 20Hz) | Used before any RTT sample |
| RTO_MIN | 2 ticks (100ms @ 20Hz) | Minimum RTO floor |
| RTO_MAX | 100 ticks (5000ms @ 20Hz) | Maximum RTO cap |
| RTO_MAX_EXP | 4 | Max backoff exponent (2^4 = 16×) |

Karn's algorithm: RTT is only sampled from packets that were not retransmitted. Retransmitted packets have ambiguous RTT (the ACK could be for the original or the retransmission), so they are excluded from the estimator. On retransmission, the existing RTO is used with exponential backoff but the estimator is not updated.

### 4.6 Window Size

```
WINDOW_SIZE (recommended) = 8
SEQ space = 64 ≥ 2 × WINDOW_SIZE (requirement for Selective Repeat ARQ)
```

The window size is a sender-side parameter and does not need to be communicated to the receiver. The receiver accepts any SEQ within [expected_seq, expected_seq + 63) modulo 64.

### 4.7 Sequence Number Wraparound

SEQ numbers are 6 bits and wrap around from 63 to 0. All comparisons use modular arithmetic:

```
seq_after(a, b)  = ((a - b) mod 64) > 0 && ((a - b) mod 64) < 32
seq_before(a, b) = seq_after(b, a)
```

---

## 5. Unreliable Channel Protocol

### 5.1 Behavior

- No sequence numbers, no retransmission, no acknowledgment.
- Each frame is self-contained. The receiver processes it immediately or discards it.
- If the application sends data faster than the link can transmit, the newest data overwrites the oldest (latest-value semantics).
- The TYPE field allows the receiver to dispatch different message types to different handlers without inspecting the payload.

### 5.2 Packet Rate

The application defines the unreliable packet rate (e.g., 20 Hz for RC channels, 10 Hz for telemetry). The protocol does not enforce any rate.

---

## 6. ACK Piggybacking Strategy

Since the link is full duplex, both directions are always active. The ACK strategy is:

1. **Piggyback first**: When the reverse direction has outgoing data (reliable or unreliable), attach the ACK block (set A=1). Cost: 2 extra bytes. Every outgoing packet checks whether the reverse-direction receiver has a pending ACK and piggybacks it.
2. **Standalone ACK trigger**: If no piggybacking opportunity occurs within `ACK_DELAY_TICKS` ticks (recommended: 2 ticks = 100ms at 20Hz) after receiving a reliable frame, the receiver MUST send an ACK-only frame (3 bytes total). This prevents ACK starvation when reverse-direction traffic pauses.
3. **ACK coalescing**: The receiver tracks `ack_pending_since` (the tick when data was last received). After sending an ACK (piggybacked or standalone), this timestamp is reset. The standalone ACK deadline is: `now − ack_pending_since ≥ ACK_DELAY_TICKS`.

The standalone ACK trigger is critical for preventing unnecessary retransmissions. Without it, a sender whose reverse-direction peer has no data to send would never receive ACKs, causing needless retransmissions and eventual delivery failure (ACK starvation).

| Parameter | Value | Description |
|-----------|-------|-------------|
| ACK_DELAY_TICKS | 2 | Max ticks before standalone ACK is triggered |

Typical scenario at 20 Hz:

```
Time   UL (GCS → UAV)                    DL (UAV → GCS)
──────────────────────────────────────────────────────────
 0ms   [Unreliable: RC Channels]          [Unreliable: Attitude + ACK(UL stream 0)]
50ms   [Reliable: Waypoint cmd, SEQ=3]    [Unreliable: GPS]
100ms  [Unreliable: RC Channels]          [Unreliable: Battery + ACK(UL stream 0, CACK=3)]
150ms  [Reliable: Param set, SEQ=4]       [Reliable: Log chunk, SEQ=12 + ACK(UL stream 0, CACK=4)]
200ms  [Unreliable: RC Channels + ACK(DL stream 0, CACK=12)]  [Unreliable: Attitude]
```

Note: At 100ms, the UAV piggybacks ACK for the waypoint command (SEQ=3) onto a regular unreliable battery telemetry packet. No dedicated ACK packet needed.

---

## 7. Configuration Parameters

| Parameter | Recommended Value | Description |
|-----------|-------------------|-------------|
| WINDOW_SIZE | 8 | Max unacknowledged reliable frames per stream |
| MAX_RETRIES | 5 | Max retransmissions before delivery failure |
| ACK_DELAY_TICKS | 2 ticks (100ms @ 20Hz) | Max ticks before standalone ACK is sent |
| MAX_STREAMS | 8 | Max concurrent reliable streams per direction |
| MAX_FRAGMENTS | 4 | Max fragments per message |
| SEQ_BITS | 6 | Sequence number width (0–63) |
| RTO_INITIAL | 4 ticks (200ms @ 20Hz) | Initial RTO before any RTT sample |
| RTO_MIN | 2 ticks (100ms @ 20Hz) | Minimum RTO floor |
| RTO_MAX | 100 ticks (5000ms @ 20Hz) | Maximum RTO cap |
| RTO_MAX_EXP | 4 | Max backoff exponent (2^4 = 16×) |
| LQ_WINDOW | 50 packets | Link quality sliding window size |

---

## 8. Application Layer API

### 8.1 Unreliable Channel

```c
// Send data on unreliable channel. Returns immediately.
// If a previous unsent frame of the same type exists, it is overwritten.
void ldtp_send_unreliable(uint8_t type, const uint8_t *data, uint8_t len);

// Register handler for incoming unreliable messages.
void ldtp_on_unreliable(uint8_t type, void (*handler)(const uint8_t *data, uint8_t len));
```

### 8.2 Reliable Channel

```c
typedef void (*ldtp_delivery_cb)(uint8_t stream_id, uint8_t seq, bool success);

// Send data on reliable channel. Returns assigned SEQ.
// Callback is invoked when delivery is confirmed or fails after MAX_RETRIES.
uint8_t ldtp_send_reliable(uint8_t stream_id, const uint8_t *data, uint16_t len,
                           ldtp_delivery_cb callback);

// Register handler for incoming reliable messages (fully reassembled).
void ldtp_on_reliable(uint8_t stream_id, void (*handler)(const uint8_t *data, uint16_t len));
```

### 8.3 Link Quality Monitoring

The protocol provides an application-layer link quality indicator based on a sliding-window delivery ratio. This allows applications to display signal quality, trigger failsafe behaviors, or adapt transmission rates.

```c
// Returns link quality as percentage (0–100) for the specified direction.
// Based on sliding window of recent packets (default window: 50 packets).
uint8_t ldtp_link_quality(ldtp_direction_t dir);

// Returns overall link quality since session start.
uint8_t ldtp_link_quality_total(ldtp_direction_t dir);
```

Implementation: Each direction maintains a circular buffer of recent packet delivery outcomes (delivered or lost). The quality metric is the ratio of delivered packets in the window.

| Quality Level | Range | Recommended Action |
|---------------|-------|--------------------|
| Excellent | 80–100% | Normal operation |
| Marginal | 50–79% | Warn user, consider increasing retries |
| Poor | 20–49% | Failsafe warning, switch to essential data only |
| Critical | 0–19% | Trigger return-to-home or emergency procedure |

The link quality monitor operates independently per direction (UL and DL), providing asymmetric quality awareness. The window size is configurable (default: 50 packets, covering 2.5 seconds at 20 Hz).

---

## 9. State Machine Summary

### 9.1 Sender (per reliable frame)

```
         ┌─────────────┐
         │   QUEUED     │  Frame waiting to be transmitted
         └──────┬───────┘
                │ transmit
                v
         ┌──────────────┐
    ┌───>│   IN_FLIGHT   │  Frame sent, waiting for ACK
    │    └──────┬───────┘
    │           │
    │    ┌──────┴───────┐
    │    │              │
    │    v              v
    │  [ACK received]  [RTO expired]
    │    │              │
    │    v              v
    │ ┌──────────┐  ┌───────────────┐
    │ │DELIVERED │  │retx < MAX ?   │
    │ └──────────┘  └───┬───────┬───┘
    │                   │ yes   │ no
    │                   v       v
    └──── retransmit  ┌──────────┐
                      │ FAILED   │  → notify application
                      └──────────┘
```

### 9.2 Receiver (per stream)

```
  [frame arrives with SEQ, FRAG]
            │
            v
  ┌─────────────────────┐
  │ SEQ within window?  │── no ──> discard (duplicate or too old)
  └─────────┬───────────┘
            │ yes
            v
  ┌─────────────────────┐
  │ Store fragment       │
  │ Update recv_bitmap   │
  └─────────┬───────────┘
            │
            v
  ┌─────────────────────┐
  │ All frags of this   │── no ──> wait for more fragments
  │ SEQ received?       │
  └─────────┬───────────┘
            │ yes
            v
  ┌─────────────────────┐
  │ Reassemble message  │
  │ Deliver to app      │
  │ Advance window      │
  └─────────────────────┘
```

---

## 10. Wire Format Quick Reference

### Unreliable, no ACK (1 byte overhead):
```
Byte 0: [0][0][0][TYPE:3][VER:2]
Byte 1..N: Payload
```

### Unreliable, with piggybacked ACK (3 bytes overhead):
```
Byte 0: [0][1][0][TYPE:3][VER:2]
Byte 1: [SID:3][CACK:5(upper)]    ← ACK block
Byte 2: [CACK:1(lower)][SACK:7]   ← ACK block
Byte 3..N: Payload
```

### Reliable, no ACK (2 bytes overhead):
```
Byte 0: [1][0][F:1][STREAM:3][VER:2]
Byte 1: [SEQ:6][FRAG:2]
Byte 2..N: Payload
```

### Reliable, with piggybacked ACK (4 bytes overhead):
```
Byte 0: [1][1][F:1][STREAM:3][VER:2]
Byte 1: [SID:3][CACK:5(upper)]    ← ACK block
Byte 2: [CACK:1(lower)][SACK:7]   ← ACK block
Byte 3: [SEQ:6][FRAG:2]
Byte 4..N: Payload
```

### ACK-only (3 bytes total, no payload):
```
Byte 0: [0][1][0][111][VER:2]     ← TYPE=7 reserved for ACK-only
Byte 1: [SID:3][CACK:5(upper)]
Byte 2: [CACK:1(lower)][SACK:7]
```
