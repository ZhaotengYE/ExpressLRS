#!/usr/bin/env python3
"""
LDTP Protocol — Comprehensive Test Suite
==========================================
Tests the protocol under various channel conditions to verify correctness
and identify failure modes.

Channel models:
  1. Uniform random loss:  5%, 15%, 30%, 50%, 70%, 90%
  2. Burst loss:          Gilbert-Elliott model (good/bad states)
  3. Asymmetric loss:     different UL/DL loss rates
  4. Stress:              many concurrent reliable messages
  5. ACK starvation:      only reverse-direction ACKs are lost
  6. One-way blackout:    100% loss in one direction for a period

Output: test_results.html (comprehensive dashboard)
"""

import json, random, os, struct, sys, copy, time
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
from ldtp_protocol import (
    build_unreliable, build_reliable, build_ack_only, decode_frame,
    ReliableSender, ReliableReceiver, LinkQualityMonitor,
    SEQ_MOD, WINDOW_SIZE, MAX_RETRIES, RTO_TICKS, ACK_DELAY_TICKS
)

# ═══════════════════════════════════════════════════════════════════════════
# Channel Models
# ═══════════════════════════════════════════════════════════════════════════

class UniformLossChannel:
    """Each packet independently lost with fixed probability."""
    def __init__(self, loss_rate: float):
        self.loss_rate = loss_rate
        self.name = f"Uniform {loss_rate*100:.0f}%"
    def __call__(self, frame_bytes, direction='ul'):
        return None if random.random() < self.loss_rate else frame_bytes

class AsymmetricChannel:
    """Different loss rates for UL and DL."""
    def __init__(self, ul_rate: float, dl_rate: float):
        self.ul_rate = ul_rate
        self.dl_rate = dl_rate
        self.name = f"Asymmetric UL={ul_rate*100:.0f}% DL={dl_rate*100:.0f}%"
    def __call__(self, frame_bytes, direction='ul'):
        rate = self.ul_rate if direction == 'ul' else self.dl_rate
        return None if random.random() < rate else frame_bytes

class BurstLossChannel:
    """Gilbert-Elliott burst loss model.
    Two states: GOOD (low loss p_good) and BAD (high loss p_bad).
    Transition probabilities: p_to_bad, p_to_good.
    """
    def __init__(self, p_good=0.02, p_bad=0.80, p_to_bad=0.10, p_to_good=0.30,
                 label=""):
        self.p_good = p_good
        self.p_bad = p_bad
        self.p_to_bad = p_to_bad
        self.p_to_good = p_to_good
        self.state = 'good'  # start in good state
        self.name = label or f"Burst(good={p_good},bad={p_bad},→bad={p_to_bad},→good={p_to_good})"
    def __call__(self, frame_bytes, direction='ul'):
        # Determine loss for current state
        p_loss = self.p_good if self.state == 'good' else self.p_bad
        lost = random.random() < p_loss
        # State transition
        if self.state == 'good':
            if random.random() < self.p_to_bad:
                self.state = 'bad'
        else:
            if random.random() < self.p_to_good:
                self.state = 'good'
        return None if lost else frame_bytes

class BlackoutChannel:
    """Normal low-loss channel with a blackout window where one or both directions
    experience 100% loss."""
    def __init__(self, base_loss=0.05, blackout_start=40, blackout_end=70,
                 blackout_dir='both', label=""):
        self.base_loss = base_loss
        self.blackout_start = blackout_start
        self.blackout_end = blackout_end
        self.blackout_dir = blackout_dir  # 'ul', 'dl', or 'both'
        self.tick = -1
        self.name = label or f"Blackout t=[{blackout_start},{blackout_end}] dir={blackout_dir}"
    def advance_tick(self, t):
        self.tick = t
    def __call__(self, frame_bytes, direction='ul'):
        in_blackout = self.blackout_start <= self.tick < self.blackout_end
        if in_blackout and (self.blackout_dir == 'both' or self.blackout_dir == direction):
            return None
        return None if random.random() < self.base_loss else frame_bytes

class AckStarvationChannel:
    """Normal loss for data, but higher loss for packets carrying ACK blocks
    (simulating: ACK info is piggybacked, and reverse direction is lossy)."""
    def __init__(self, data_loss=0.10, ack_loss=0.50, label=""):
        self.data_loss = data_loss
        self.ack_loss = ack_loss
        self.name = label or f"ACK-starved(data={data_loss*100:.0f}%, ack_dir={ack_loss*100:.0f}%)"
    def __call__(self, frame_bytes, direction='ul'):
        # We simulate this by having high loss in DL direction (so UL ACKs are lost)
        # and low loss in UL direction
        rate = self.data_loss if direction == 'ul' else self.ack_loss
        return None if random.random() < rate else frame_bytes


# ═══════════════════════════════════════════════════════════════════════════
# Application Data
# ═══════════════════════════════════════════════════════════════════════════

def make_rc_payload(tick):
    vals = [1500 + int(200 * (((tick * 7 + ch * 13) % 100) / 100.0 - 0.5))
            for ch in range(8)]
    return struct.pack('<8H', *vals)

_telem_cycle = [0, 1, 2, 0]
_telem_names = {0: "Attitude", 1: "GPS", 2: "Battery"}

def make_telemetry(tick):
    t = _telem_cycle[tick % len(_telem_cycle)]
    if t == 0:
        data = struct.pack('<3f', 5.2 + tick*0.1, -1.3, 180.0 + tick*0.5)
    elif t == 1:
        data = struct.pack('<2d', 32.05 + tick*1e-5, 118.78 + tick*1e-5)
    else:
        data = struct.pack('<HBB', max(0, 1180 - tick), min(100, 22 + tick//10), 3)
    return t, data, _telem_names[t]


# ═══════════════════════════════════════════════════════════════════════════
# Simulation Engine  (parameterized)
# ═══════════════════════════════════════════════════════════════════════════

def run_scenario(channel_model, total_ticks, ul_schedule, dl_schedule, seed,
                 tick_hz=20):
    """Run one scenario. Returns (timeline, event_log, stats)."""
    random.seed(seed)
    tick_ms = 1000 // tick_hz

    ul_rel_sender   = ReliableSender(0)
    ul_rel_receiver = ReliableReceiver(0)
    dl_rel_sender   = ReliableSender(0)
    dl_rel_receiver = ReliableReceiver(0)

    # Link quality monitors (one per direction)
    ul_lqm = LinkQualityMonitor(window_size=50)
    dl_lqm = LinkQualityMonitor(window_size=50)

    timeline = []
    event_log = []

    def log(tick, msg):
        event_log.append({'tick': tick, 'msg': msg})

    # Stats trackers
    ul_sent = 0; ul_lost_count = 0
    dl_sent = 0; dl_lost_count = 0
    retx_events = []
    standalone_acks_sent = 0

    for t in range(total_ticks):
        tick_record = {'tick': t, 'uplink': None, 'downlink': None}

        # Advance blackout tick counter if applicable
        if hasattr(channel_model, 'advance_tick'):
            channel_model.advance_tick(t)

        # --- 0. Timers ---
        for ev in ul_rel_sender.tick(t):
            log(t, f"UL reliable: {ev['event']} SEQ={ev.get('seq')}")
            if ev['event'] == 'retx_timeout':
                retx_events.append(('UL', t, ev.get('seq'), ev.get('attempt')))
            elif ev['event'] == 'reliable_failed':
                log(t, f"UL FAILED: SEQ={ev['seq']} exhausted {MAX_RETRIES} retries")
        for ev in dl_rel_sender.tick(t):
            log(t, f"DL reliable: {ev['event']} SEQ={ev.get('seq')}")
            if ev['event'] == 'retx_timeout':
                retx_events.append(('DL', t, ev.get('seq'), ev.get('attempt')))
            elif ev['event'] == 'reliable_failed':
                log(t, f"DL FAILED: SEQ={ev['seq']} exhausted {MAX_RETRIES} retries")

        # --- 1. Enqueue ---
        if t in ul_schedule:
            payload, label = ul_schedule[t]
            ul_rel_sender.queue_message(payload, label)
            log(t, f"APP  GCS enqueues: \"{label}\"")

        if t in dl_schedule:
            payload, label = dl_schedule[t]
            dl_rel_sender.queue_message(payload, label)
            log(t, f"APP  UAV enqueues: \"{label}\"")

        # --- 2. UL frame ---
        dl_ack = dl_rel_receiver.get_ack()
        ul_frame = None
        ul_meta = {'direction': 'uplink', 'tick': t}
        ul_ack_piggybacked = False

        if ul_rel_sender.has_work():
            frame, meta = ul_rel_sender.get_frame(ack_info=dl_ack, now=t)
            if frame:
                ul_frame = frame
                ul_meta.update({
                    'channel': 'reliable', 'stream': 0,
                    'seq': meta['seq'], 'label': meta['label'],
                    'is_retx': meta['is_retx'], 'retx_count': meta['retx_count'],
                    'has_ack': dl_ack is not None, 'rto': meta.get('rto', RTO_TICKS),
                })
                if dl_ack is not None:
                    ul_ack_piggybacked = True
                    dl_rel_receiver.mark_ack_sent(t)
                retx_str = f" (RETX #{meta['retx_count']})" if meta['is_retx'] else ""
                log(t, f"UL TX reliable SEQ={meta['seq']} \"{meta['label']}\"{retx_str} rto={meta.get('rto',RTO_TICKS)}")

        if ul_frame is None:
            # Check if we need a standalone ACK-only frame for DL receiver
            if dl_rel_receiver.needs_standalone_ack(t) and dl_ack is not None:
                ul_frame = build_ack_only(dl_ack)
                ul_meta.update({
                    'channel': 'ack_only', 'has_ack': True,
                })
                ul_ack_piggybacked = True
                dl_rel_receiver.mark_ack_sent(t)
                standalone_acks_sent += 1
                log(t, f"UL TX standalone ACK-only frame")
            else:
                rc_data = make_rc_payload(t)
                ul_frame = build_unreliable(0, rc_data, ack=dl_ack)
                ul_meta.update({
                    'channel': 'unreliable', 'type': 0,
                    'type_name': 'RC', 'has_ack': dl_ack is not None,
                })
                if dl_ack is not None:
                    ul_ack_piggybacked = True
                    dl_rel_receiver.mark_ack_sent(t)

        # --- 3. DL frame ---
        ul_ack = ul_rel_receiver.get_ack()
        dl_frame = None
        dl_meta = {'direction': 'downlink', 'tick': t}
        dl_ack_piggybacked = False

        if dl_rel_sender.has_work():
            frame, meta = dl_rel_sender.get_frame(ack_info=ul_ack, now=t)
            if frame:
                dl_frame = frame
                dl_meta.update({
                    'channel': 'reliable', 'stream': 0,
                    'seq': meta['seq'], 'label': meta['label'],
                    'is_retx': meta['is_retx'], 'retx_count': meta['retx_count'],
                    'has_ack': ul_ack is not None, 'rto': meta.get('rto', RTO_TICKS),
                })
                if ul_ack is not None:
                    dl_ack_piggybacked = True
                    ul_rel_receiver.mark_ack_sent(t)
                retx_str = f" (RETX #{meta['retx_count']})" if meta['is_retx'] else ""
                log(t, f"DL TX reliable SEQ={meta['seq']} \"{meta['label']}\"{retx_str} rto={meta.get('rto',RTO_TICKS)}")

        if dl_frame is None:
            # Check if we need a standalone ACK-only frame for UL receiver
            if ul_rel_receiver.needs_standalone_ack(t) and ul_ack is not None:
                dl_frame = build_ack_only(ul_ack)
                dl_meta.update({
                    'channel': 'ack_only', 'has_ack': True,
                })
                dl_ack_piggybacked = True
                ul_rel_receiver.mark_ack_sent(t)
                standalone_acks_sent += 1
                log(t, f"DL TX standalone ACK-only frame")
            else:
                ttype, tdata, tname = make_telemetry(t)
                dl_frame = build_unreliable(ttype, tdata, ack=ul_ack)
                dl_meta.update({
                    'channel': 'unreliable', 'type': ttype,
                    'type_name': tname, 'has_ack': ul_ack is not None,
                })
                if ul_ack is not None:
                    dl_ack_piggybacked = True
                    ul_rel_receiver.mark_ack_sent(t)

        # --- 4. Channel ---
        ul_rx = channel_model(ul_frame, direction='ul')
        dl_rx = channel_model(dl_frame, direction='dl')

        ul_sent += 1; dl_sent += 1
        if ul_rx is None: ul_lost_count += 1
        if dl_rx is None: dl_lost_count += 1

        # Link quality monitor
        ul_lqm.record(ul_rx is not None)
        dl_lqm.record(dl_rx is not None)

        ul_meta['lost'] = (ul_rx is None)
        dl_meta['lost'] = (dl_rx is None)
        ul_meta['frame_hex'] = ul_frame.hex()
        dl_meta['frame_hex'] = dl_frame.hex()
        ul_meta['decoded'] = decode_frame(ul_frame)
        dl_meta['decoded'] = decode_frame(dl_frame)

        # --- 5. GCS processes DL ---
        if dl_rx is not None:
            d = decode_frame(dl_rx)
            if d:
                if d['ack_present']:
                    evts = ul_rel_sender.process_ack(d['ack_sid'], d['ack_cack'], d['ack_sack'], now=t)
                    for ev in evts:
                        log(t, f"GCS ACK→UL: {ev['event']} SEQ={ev['seq']} lat={ev.get('latency',0)}")
                if d['reliable']:
                    dl_rel_receiver.receive(d['seq'], d['frag'], d['more_frag'], d['payload'], now=t)
                    for msg in dl_rel_receiver.pop_delivered():
                        log(t, f"GCS APP delivered DL reliable: {msg[:40]}")

        # --- 6. UAV processes UL ---
        if ul_rx is not None:
            d = decode_frame(ul_rx)
            if d:
                if d['ack_present']:
                    evts = dl_rel_sender.process_ack(d['ack_sid'], d['ack_cack'], d['ack_sack'], now=t)
                    for ev in evts:
                        log(t, f"UAV ACK→DL: {ev['event']} SEQ={ev['seq']} lat={ev.get('latency',0)}")
                if d['reliable']:
                    ul_rel_receiver.receive(d['seq'], d['frag'], d['more_frag'], d['payload'], now=t)
                    for msg in ul_rel_receiver.pop_delivered():
                        log(t, f"UAV APP delivered UL reliable: {msg[:40]}")

        tick_record['uplink'] = ul_meta
        tick_record['downlink'] = dl_meta
        timeline.append(tick_record)

    # --- Summary ---
    stats = {
        'config': {
            'tick_hz': tick_hz, 'tick_ms': tick_ms,
            'total_ticks': total_ticks, 'channel': channel_model.name,
            'seed': seed,
        },
        'ul_total': ul_sent, 'dl_total': dl_sent,
        'ul_lost': ul_lost_count, 'dl_lost': dl_lost_count,
        'ul_loss_pct': round(100*ul_lost_count/max(ul_sent,1), 1),
        'dl_loss_pct': round(100*dl_lost_count/max(dl_sent,1), 1),
        'ul_reliable_delivered': len(ul_rel_sender.delivered),
        'ul_reliable_failed': len(ul_rel_sender.failed),
        'dl_reliable_delivered': len(dl_rel_sender.delivered),
        'dl_reliable_failed': len(dl_rel_sender.failed),
        'total_retx': len(retx_events),
        'standalone_acks': standalone_acks_sent,
        'ul_link_quality': ul_lqm.quality_pct,
        'dl_link_quality': dl_lqm.quality_pct,
        'ul_rto_final': ul_rel_sender.rto,
        'dl_rto_final': dl_rel_sender.rto,
        'ul_rtt_est': round(ul_rel_sender.rtt_est, 2) if ul_rel_sender.rtt_initialized else None,
        'dl_rtt_est': round(dl_rel_sender.rtt_est, 2) if dl_rel_sender.rtt_initialized else None,
        'ul_avg_latency': (round(sum(d[2] for d in ul_rel_sender.delivered) /
                                 len(ul_rel_sender.delivered), 2)
                           if ul_rel_sender.delivered else None),
        'dl_avg_latency': (round(sum(d[2] for d in dl_rel_sender.delivered) /
                                 len(dl_rel_sender.delivered), 2)
                           if dl_rel_sender.delivered else None),
        'ul_max_latency': (max(d[2] for d in ul_rel_sender.delivered)
                           if ul_rel_sender.delivered else None),
        'dl_max_latency': (max(d[2] for d in dl_rel_sender.delivered)
                           if dl_rel_sender.delivered else None),
        'ul_delivered_detail': [
            {'seq': s, 'label': l, 'latency': la}
            for s, l, la in ul_rel_sender.delivered
        ],
        'dl_delivered_detail': [
            {'seq': s, 'label': l, 'latency': la}
            for s, l, la in dl_rel_sender.delivered
        ],
        'ul_failed_detail': [{'seq': s, 'label': l} for s, l in ul_rel_sender.failed],
        'dl_failed_detail': [{'seq': s, 'label': l} for s, l in dl_rel_sender.failed],
        'retx_events': [{'dir': d, 'tick': t, 'seq': s, 'attempt': a}
                        for d, t, s, a in retx_events],
    }

    return timeline, event_log, stats


# ═══════════════════════════════════════════════════════════════════════════
# Test Scenarios
# ═══════════════════════════════════════════════════════════════════════════

def define_scenarios():
    """Returns list of (scenario_name, channel_model, total_ticks,
    ul_schedule, dl_schedule, seed) tuples."""

    # Standard reliable messages (same as original sim)
    std_ul = {
        20: (b'WPT:LAT32.05,LON118.78,ALT100', "SET_WAYPOINT"),
        50: (b'CMD:ARM_MOTORS',                 "ARM"),
        70: (b'CMD:RETURN_TO_HOME',             "RTH"),
    }
    std_dl = {
        30: (b'ALERT:LOW_BATTERY_22PCT',        "LOW_BATTERY"),
        60: (b'ALERT:GPS_DEGRADED_HDOP8',       "GPS_DEGRADED"),
    }

    # Heavy traffic: 10 reliable messages rapidly on UL
    heavy_ul = {}
    for i in range(10):
        heavy_ul[10 + i*2] = (f'CMD:WAYPOINT_{i}'.encode(), f"WPT_{i}")
    heavy_dl = {
        15: (b'ALERT:GEOFENCE_WARN', "GEOFENCE"),
        35: (b'ALERT:WIND_HIGH',     "WIND"),
        55: (b'ALERT:LOW_BAT_10PCT', "CRITICAL_BAT"),
    }

    # Rapid-fire: 20 messages all at once (tests window saturation)
    burst_ul = {}
    for i in range(20):
        burst_ul[10] = burst_ul.get(10, [])  # can't reuse same tick key like this
    # Actually we need to queue them differently. Let's use tick 10-29
    window_ul = {}
    for i in range(20):
        window_ul[10 + i] = (f'CMD:SEQ_TEST_{i}'.encode(), f"SEQ_{i}")
    window_dl = {}

    scenarios = []

    # ──── Group 1: Uniform loss sweep ────
    for loss in [0.05, 0.15, 0.30, 0.50, 0.70, 0.90]:
        scenarios.append((
            f"uniform_{int(loss*100)}pct",
            UniformLossChannel(loss),
            200,  # 10 seconds
            std_ul, std_dl, 42
        ))

    # ──── Group 2: Burst loss ────
    # Mild bursts: short bad periods
    scenarios.append((
        "burst_mild",
        BurstLossChannel(p_good=0.02, p_bad=0.60, p_to_bad=0.08, p_to_good=0.40,
                         label="Burst-Mild (bad=60%, →bad=8%, →good=40%)"),
        200, std_ul, std_dl, 42
    ))
    # Severe bursts: long bad periods
    scenarios.append((
        "burst_severe",
        BurstLossChannel(p_good=0.03, p_bad=0.90, p_to_bad=0.15, p_to_good=0.10,
                         label="Burst-Severe (bad=90%, →bad=15%, →good=10%)"),
        300, std_ul, std_dl, 42
    ))

    # ──── Group 3: Asymmetric loss ────
    # UL clean, DL lossy (ACK starvation for UL reliable data)
    scenarios.append((
        "asym_ul_clean_dl_lossy",
        AsymmetricChannel(0.05, 0.40),
        200, std_ul, std_dl, 42
    ))
    # UL lossy, DL clean
    scenarios.append((
        "asym_ul_lossy_dl_clean",
        AsymmetricChannel(0.40, 0.05),
        200, std_ul, std_dl, 42
    ))

    # ──── Group 4: Blackout ────
    # 30-tick blackout in both directions (1.5 seconds at 20Hz)
    scenarios.append((
        "blackout_both_30ticks",
        BlackoutChannel(base_loss=0.05, blackout_start=40, blackout_end=70,
                        blackout_dir='both',
                        label="Blackout both t=[40,70] (1.5s)"),
        200, std_ul, std_dl, 42
    ))
    # 50-tick blackout in UL only
    scenarios.append((
        "blackout_ul_50ticks",
        BlackoutChannel(base_loss=0.05, blackout_start=30, blackout_end=80,
                        blackout_dir='ul',
                        label="Blackout UL only t=[30,80] (2.5s)"),
        200, std_ul, std_dl, 42
    ))

    # ──── Group 5: Stress — many concurrent messages ────
    scenarios.append((
        "stress_20_messages",
        UniformLossChannel(0.15),
        400,   # 20 seconds to give time
        window_ul, heavy_dl, 42
    ))

    # ──── Group 6: Stress — high loss + many messages ────
    scenarios.append((
        "stress_high_loss_many_msg",
        UniformLossChannel(0.40),
        400,
        window_ul, heavy_dl, 42
    ))

    # ──── Group 7: Repeated seeds for statistical analysis ────
    for seed in [1, 2, 3, 4, 5]:
        scenarios.append((
            f"uniform_30pct_seed{seed}",
            UniformLossChannel(0.30),
            200,
            std_ul, std_dl, seed
        ))

    return scenarios


# ═══════════════════════════════════════════════════════════════════════════
# Main: Run all scenarios and generate report
# ═══════════════════════════════════════════════════════════════════════════

def main():
    scenarios = define_scenarios()
    all_results = []

    print(f"Running {len(scenarios)} test scenarios...")
    print("=" * 70)

    for name, ch_model, ticks, ul_sched, dl_sched, seed in scenarios:
        t0 = time.time()
        timeline, event_log, stats = run_scenario(ch_model, ticks, ul_sched, dl_sched, seed)

        elapsed = time.time() - t0
        # Sanitize for JSON (convert bytes in decoded)
        for rec in timeline:
            for d in ['uplink', 'downlink']:
                dec = rec[d].get('decoded')
                if dec and 'payload' in dec and isinstance(dec['payload'], (bytes, bytearray)):
                    dec['payload_hex'] = dec['payload'].hex()
                    del dec['payload']

        ul_delivered = stats['ul_reliable_delivered']
        ul_expected = len(ul_sched)
        dl_delivered = stats['dl_reliable_delivered']
        dl_expected = len(dl_sched)
        ul_failed = stats['ul_reliable_failed']
        dl_failed = stats['dl_reliable_failed']

        status = "PASS" if (ul_delivered == ul_expected and dl_delivered == dl_expected) else \
                 "PARTIAL" if (ul_delivered + dl_delivered > 0) else "FAIL"

        print(f"  [{status:7s}] {name:35s}  "
              f"UL={ul_delivered}/{ul_expected} DL={dl_delivered}/{dl_expected}  "
              f"lost=UL:{stats['ul_loss_pct']}%/DL:{stats['dl_loss_pct']}%  "
              f"retx={stats['total_retx']}  "
              f"UL_lat={'%.1f'%stats['ul_avg_latency'] if stats['ul_avg_latency'] is not None else 'N/A'}  "
              f"DL_lat={'%.1f'%stats['dl_avg_latency'] if stats['dl_avg_latency'] is not None else 'N/A'}  "
              f"({elapsed:.2f}s)")

        all_results.append({
            'name': name,
            'status': status,
            'stats': stats,
            'event_log': event_log,
            'timeline': timeline,
        })

    print("=" * 70)

    # ── Summarize ──
    passed = sum(1 for r in all_results if r['status'] == 'PASS')
    partial = sum(1 for r in all_results if r['status'] == 'PARTIAL')
    failed = sum(1 for r in all_results if r['status'] == 'FAIL')
    print(f"\nSummary: {passed} PASS, {partial} PARTIAL, {failed} FAIL  "
          f"out of {len(all_results)} scenarios\n")

    # ── Detailed analysis ──
    print("═" * 70)
    print("DETAILED ANALYSIS")
    print("═" * 70)

    # 1. Loss rate vs delivery
    print("\n── Uniform Loss Sweep ──")
    print(f"{'Loss%':>8s} {'UL del':>8s} {'DL del':>8s} {'UL fail':>8s} {'DL fail':>8s} "
          f"{'UL lat':>8s} {'DL lat':>8s} {'UL maxlat':>10s} {'DL maxlat':>10s} {'Retx':>6s}")
    for r in all_results:
        if r['name'].startswith('uniform_') and 'seed' not in r['name']:
            s = r['stats']
            print(f"{s['config']['channel']:>8s} "
                  f"{s['ul_reliable_delivered']:>8d} {s['dl_reliable_delivered']:>8d} "
                  f"{s['ul_reliable_failed']:>8d} {s['dl_reliable_failed']:>8d} "
                  f"{'%.1f'%s['ul_avg_latency'] if s['ul_avg_latency'] is not None else 'N/A':>8s} "
                  f"{'%.1f'%s['dl_avg_latency'] if s['dl_avg_latency'] is not None else 'N/A':>8s} "
                  f"{'%d'%s['ul_max_latency'] if s['ul_max_latency'] is not None else 'N/A':>10s} "
                  f"{'%d'%s['dl_max_latency'] if s['dl_max_latency'] is not None else 'N/A':>10s} "
                  f"{s['total_retx']:>6d}")

    # 2. Burst analysis
    print("\n── Burst Loss Analysis ──")
    for r in all_results:
        if 'burst' in r['name']:
            s = r['stats']
            print(f"  {r['name']}: {s['config']['channel']}")
            print(f"    Actual loss: UL={s['ul_loss_pct']}% DL={s['dl_loss_pct']}%")
            print(f"    UL: {s['ul_reliable_delivered']} delivered, {s['ul_reliable_failed']} failed, "
                  f"avg_lat={'%.1f'%s['ul_avg_latency'] if s['ul_avg_latency'] is not None else 'N/A'}, "
                  f"max_lat={'%d'%s['ul_max_latency'] if s['ul_max_latency'] is not None else 'N/A'}")
            print(f"    DL: {s['dl_reliable_delivered']} delivered, {s['dl_reliable_failed']} failed, "
                  f"avg_lat={'%.1f'%s['dl_avg_latency'] if s['dl_avg_latency'] is not None else 'N/A'}, "
                  f"max_lat={'%d'%s['dl_max_latency'] if s['dl_max_latency'] is not None else 'N/A'}")
            print(f"    Total retransmissions: {s['total_retx']}")

    # 3. Asymmetric
    print("\n── Asymmetric Loss Analysis ──")
    for r in all_results:
        if 'asym' in r['name']:
            s = r['stats']
            print(f"  {r['name']}: {s['config']['channel']}")
            print(f"    UL: {s['ul_reliable_delivered']} delivered, {s['ul_reliable_failed']} failed  "
                  f"(lat avg={'%.1f'%s['ul_avg_latency'] if s['ul_avg_latency'] is not None else 'N/A'})")
            print(f"    DL: {s['dl_reliable_delivered']} delivered, {s['dl_reliable_failed']} failed  "
                  f"(lat avg={'%.1f'%s['dl_avg_latency'] if s['dl_avg_latency'] is not None else 'N/A'})")
            if s['ul_reliable_failed'] or s['dl_reliable_failed']:
                print(f"    ⚠ FAILURE detected under asymmetric conditions!")
                if 'ul_clean' in r['name']:
                    print(f"    → DL loss starves UL ACKs. UL sender cannot confirm delivery.")
                else:
                    print(f"    → UL loss starves DL ACKs. DL sender cannot confirm delivery.")

    # 4. Blackout
    print("\n── Blackout Analysis ──")
    for r in all_results:
        if 'blackout' in r['name']:
            s = r['stats']
            print(f"  {r['name']}: {s['config']['channel']}")
            print(f"    UL: {s['ul_reliable_delivered']} delivered, {s['ul_reliable_failed']} failed")
            print(f"    DL: {s['dl_reliable_delivered']} delivered, {s['dl_reliable_failed']} failed")
            print(f"    Retransmissions: {s['total_retx']}")
            if s['ul_reliable_failed'] or s['dl_reliable_failed']:
                print(f"    ⚠ Messages FAILED during blackout!")

    # 5. Stress
    print("\n── Stress Test (Many Concurrent Messages) ──")
    for r in all_results:
        if 'stress' in r['name']:
            s = r['stats']
            print(f"  {r['name']}: {s['config']['channel']}")
            print(f"    UL: {s['ul_reliable_delivered']} delivered, {s['ul_reliable_failed']} failed  "
                  f"(max_lat={'%d'%s['ul_max_latency'] if s['ul_max_latency'] is not None else 'N/A'})")
            print(f"    DL: {s['dl_reliable_delivered']} delivered, {s['dl_reliable_failed']} failed  "
                  f"(max_lat={'%d'%s['dl_max_latency'] if s['dl_max_latency'] is not None else 'N/A'})")
            print(f"    Retransmissions: {s['total_retx']}")

    # 6. Statistical variance (30% loss, 5 seeds)
    print("\n── Statistical Variance (30% loss, 5 seeds) ──")
    seed_results = [r for r in all_results if r['name'].startswith('uniform_30pct_seed')]
    if seed_results:
        ul_lats = [r['stats']['ul_avg_latency'] for r in seed_results if r['stats']['ul_avg_latency'] is not None]
        dl_lats = [r['stats']['dl_avg_latency'] for r in seed_results if r['stats']['dl_avg_latency'] is not None]
        ul_fails = [r['stats']['ul_reliable_failed'] for r in seed_results]
        dl_fails = [r['stats']['dl_reliable_failed'] for r in seed_results]
        retxs = [r['stats']['total_retx'] for r in seed_results]

        for r in seed_results:
            s = r['stats']
            print(f"    {r['name']}: UL={s['ul_reliable_delivered']}/{s['ul_reliable_delivered']+s['ul_reliable_failed']} "
                  f"DL={s['dl_reliable_delivered']}/{s['dl_reliable_delivered']+s['dl_reliable_failed']} "
                  f"retx={s['total_retx']} "
                  f"UL_lat={'%.1f'%s['ul_avg_latency'] if s['ul_avg_latency'] is not None else 'N/A'} "
                  f"DL_lat={'%.1f'%s['dl_avg_latency'] if s['dl_avg_latency'] is not None else 'N/A'}")

        if ul_lats:
            avg_ul = sum(ul_lats)/len(ul_lats)
            print(f"  UL avg latency: mean={avg_ul:.2f} min={min(ul_lats):.1f} max={max(ul_lats):.1f}")
        if dl_lats:
            avg_dl = sum(dl_lats)/len(dl_lats)
            print(f"  DL avg latency: mean={avg_dl:.2f} min={min(dl_lats):.1f} max={max(dl_lats):.1f}")
        print(f"  UL failures across seeds: {ul_fails}")
        print(f"  DL failures across seeds: {dl_fails}")
        print(f"  Retransmissions: {retxs}")

    # ── Protocol Issues Found ──
    print("\n" + "═" * 70)
    print("IDENTIFIED PROTOCOL ISSUES & OBSERVATIONS")
    print("═" * 70)

    issues = []

    # Check for failures at each loss level
    for r in all_results:
        if r['status'] != 'PASS':
            s = r['stats']
            if s['ul_reliable_failed'] > 0:
                issues.append(f"[{r['name']}] UL reliable FAILED: {s['ul_reliable_failed']} messages "
                             f"(MAX_RETRIES={MAX_RETRIES} exhausted)")
            if s['dl_reliable_failed'] > 0:
                issues.append(f"[{r['name']}] DL reliable FAILED: {s['dl_reliable_failed']} messages "
                             f"(MAX_RETRIES={MAX_RETRIES} exhausted)")

    # Check asymmetric ACK starvation
    for r in all_results:
        if 'asym_ul_clean_dl_lossy' in r['name']:
            if r['stats']['ul_reliable_failed'] > 0:
                issues.append(
                    f"[ACK Starvation] UL data delivered to UAV but ACK lost in DL. "
                    f"Sender retries unnecessarily → eventually fails despite data arriving. "
                    f"Root cause: piggybacked ACK shares fate with DL data channel.")

    # Check window exhaustion
    for r in all_results:
        if 'stress' in r['name']:
            s = r['stats']
            total_queued_ul = len([k for k in [10+i for i in range(20)]])
            if s['ul_reliable_failed'] > 0:
                issues.append(
                    f"[Window Stress] {r['name']}: {s['ul_reliable_failed']} of 20 UL messages failed. "
                    f"Window size {WINDOW_SIZE} means at most {WINDOW_SIZE} in-flight at once. "
                    f"With high loss, window stalls waiting for ACKs.")

    # Latency explosion
    for r in all_results:
        s = r['stats']
        if s['ul_max_latency'] and s['ul_max_latency'] > 50:
            issues.append(f"[Latency] {r['name']}: UL max latency = {s['ul_max_latency']} ticks "
                         f"({s['ul_max_latency'] * s['config']['tick_ms']}ms)")
        if s['dl_max_latency'] and s['dl_max_latency'] > 50:
            issues.append(f"[Latency] {r['name']}: DL max latency = {s['dl_max_latency']} ticks "
                         f"({s['dl_max_latency'] * s['config']['tick_ms']}ms)")

    if not issues:
        print("  No critical issues found — protocol handled all scenarios.")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")

    # ── Save JSON for further analysis ──
    out_dir = os.path.dirname(os.path.abspath(__file__))

    # Build compact summary JSON (no full timeline for space)
    summary = []
    for r in all_results:
        summary.append({
            'name': r['name'],
            'status': r['status'],
            'stats': r['stats'],
        })

    json_path = os.path.join(out_dir, 'test_results.json')
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\nDetailed JSON → {json_path}")

    return all_results, issues


if __name__ == '__main__':
    all_results, issues = main()
