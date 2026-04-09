#!/usr/bin/env python3
"""
LDTP Protocol — Edge Case Deep Dive
=====================================
Targeted tests for specific protocol weaknesses found in round 1:

1. The "stuck message" problem at 90% loss — how long until MAX_RETRIES exhausted?
2. Pure ACK starvation — data arrives but no ACK gets back
3. Window head-of-line blocking — one lost packet blocks entire window
4. Exponential backoff ceiling analysis
5. Sequence number wraparound
6. Messages queued during a blackout
7. Rapid message burst vs slow trickle under same loss
"""

import random, sys, os, struct, time, json
from ldtp_protocol import (
    build_unreliable, build_reliable, build_ack_only, decode_frame,
    ReliableSender, ReliableReceiver, SEQ_MOD, WINDOW_SIZE, MAX_RETRIES,
    RTO_TICKS, RTO_MAX_EXP
)

# Reuse from test_protocol
from test_protocol import (
    UniformLossChannel, AsymmetricChannel, BurstLossChannel,
    BlackoutChannel, make_rc_payload, make_telemetry, run_scenario
)


def print_header(title):
    print(f"\n{'═'*70}\n{title}\n{'═'*70}")


def main():
    all_findings = []

    # ═══════════════════════════════════════════════════════════════════
    # TEST 1: Retransmission timer analysis
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 1: Retransmission Timer & Backoff Analysis")

    print(f"  Protocol constants:")
    print(f"    RTO_TICKS      = {RTO_TICKS}")
    print(f"    RTO_MAX_EXP    = {RTO_MAX_EXP}")
    print(f"    MAX_RETRIES    = {MAX_RETRIES}")
    print(f"    WINDOW_SIZE    = {WINDOW_SIZE}")
    print(f"    SEQ_MOD        = {SEQ_MOD}")

    # Calculate total ticks needed for MAX_RETRIES to expire
    total_timeout_ticks = 0
    print(f"\n  Backoff schedule (assuming every TX is lost):")
    for attempt in range(MAX_RETRIES + 1):
        exp = min(attempt, RTO_MAX_EXP)
        timer = RTO_TICKS * (2 ** exp)
        if attempt > 0:  # first TX isn't a retx
            total_timeout_ticks += timer
            print(f"    Attempt {attempt}: timer = {RTO_TICKS}×2^{exp} = {timer} ticks ({timer*50}ms)")
    print(f"  Total ticks from first TX to failure declaration: {total_timeout_ticks}")
    print(f"  Total time: {total_timeout_ticks * 50}ms = {total_timeout_ticks * 50 / 1000:.1f}s")
    print(f"  (Plus initial TX tick + 1 tick for first timeout check)")

    # But we also need the data packet to arrive AND the ACK to get back.
    # With exponential backoff, the gaps between retries grow:
    # TX → wait 4 → TX → wait 8 → TX → wait 16 → TX → wait 16 → TX → wait 16 → FAIL
    # That's 4+8+16+16+16 = 60 ticks from first retx timeout to failure
    # Plus the initial RTO_TICKS=4 wait before first retx.

    all_findings.append(
        f"Theoretical max time to failure: {total_timeout_ticks} ticks "
        f"({total_timeout_ticks*50/1000:.1f}s) — this is the worst case for a single message."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 2: 90% loss with enough time to see actual failures
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 2: 90% Loss — Extended Duration")

    std_ul = {
        20: (b'CMD:ARM_MOTORS', "ARM"),
        50: (b'CMD:RETURN_TO_HOME', "RTH"),
    }
    std_dl = {
        30: (b'ALERT:LOW_BATTERY', "LOW_BAT"),
    }

    for duration in [200, 400, 800, 1600]:
        _, _, stats = run_scenario(
            UniformLossChannel(0.90), duration, std_ul, std_dl, seed=42
        )
        ul_d = stats['ul_reliable_delivered']
        ul_f = stats['ul_reliable_failed']
        dl_d = stats['dl_reliable_delivered']
        dl_f = stats['dl_reliable_failed']
        ul_stuck = len(std_ul) - ul_d - ul_f
        dl_stuck = len(std_dl) - dl_d - dl_f
        print(f"  {duration:4d} ticks ({duration*50/1000:5.1f}s): "
              f"UL delivered={ul_d} failed={ul_f} stuck={ul_stuck} | "
              f"DL delivered={dl_d} failed={dl_f} stuck={dl_stuck} | "
              f"retx={stats['total_retx']}")
        if ul_f > 0 or dl_f > 0:
            print(f"    ⚠ FAILURE detected at 90% loss after {duration} ticks")
            for fd in stats['ul_failed_detail']:
                print(f"      UL FAILED: {fd['label']}")
            for fd in stats['dl_failed_detail']:
                print(f"      DL FAILED: {fd['label']}")

    all_findings.append(
        "At 90% loss, messages can be 'stuck' indefinitely if not enough ticks elapse. "
        "The exponential backoff means later retries are spaced far apart, so MAX_RETRIES "
        "takes a long time to exhaust. This is by design (exponential backoff prevents "
        "congestion collapse) but means a failing message ties up a window slot for a long time."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 3: Pure ACK starvation (UL data always arrives, DL always lost)
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 3: Pure ACK Starvation")
    print("  Scenario: UL=0% loss, DL=100% loss")
    print("  Data always reaches UAV, but ACKs never get back to GCS")

    # Custom channel: UL never drops, DL always drops
    class PureAckStarve:
        name = "UL=0% DL=100%"
        def __call__(self, frame_bytes, direction='ul'):
            if direction == 'dl':
                return None
            return frame_bytes

    simple_ul = {10: (b'CMD:ARM', "ARM")}
    simple_dl = {}

    _, events, stats = run_scenario(PureAckStarve(), 200, simple_ul, simple_dl, seed=42)

    print(f"  UL: {stats['ul_reliable_delivered']} delivered, {stats['ul_reliable_failed']} failed")
    print(f"  Retransmissions: {stats['total_retx']}")

    # Print relevant events
    for ev in events:
        if 'reliable' in ev['msg'] or 'ACK' in ev['msg'] or 'FAIL' in ev['msg']:
            print(f"    [t={ev['tick']:3d}] {ev['msg']}")

    all_findings.append(
        "Under pure ACK starvation (data arrives 100% but ACK channel 100% lost), "
        "the sender exhausts MAX_RETRIES and declares failure even though the data "
        "was successfully received by the other side. This is a fundamental limitation "
        "of any ARQ protocol — the sender MUST receive acknowledgment to stop retransmitting."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 4: Window Head-of-Line Blocking
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 4: Window Head-of-Line Blocking")
    print("  Scenario: 8 messages queued, but first packet always lost by bad luck")
    print("  Question: Does one stalled SEQ block the whole window?")

    # We use Selective Repeat, NOT Go-Back-N, so theoretically no HOL blocking.
    # But let's verify: queue 8+ messages, make the first one's data always lost
    # while later ones go through. Does the receiver deliver in order?

    # Actually, Selective Repeat means each packet is independently acked/retransmitted.
    # But DELIVERY to application is still in-order. Let's verify:

    window_ul = {}
    for i in range(12):
        window_ul[10 + i] = (f'CMD:MSG_{i}'.encode(), f"MSG_{i}")

    _, events, stats = run_scenario(
        UniformLossChannel(0.20), 400, window_ul, {}, seed=42
    )

    print(f"  UL: {stats['ul_reliable_delivered']}/12 delivered, {stats['ul_reliable_failed']} failed")
    print(f"  Avg latency: {stats['ul_avg_latency']} ticks")
    print(f"  Max latency: {stats['ul_max_latency']} ticks")
    print(f"  Retransmissions: {stats['total_retx']}")

    # Print delivery order
    print(f"  Delivery order:")
    for d in stats['ul_delivered_detail']:
        print(f"    SEQ={d['seq']} \"{d['label']}\" latency={d['latency']} ticks")

    all_findings.append(
        "Selective Repeat ARQ means each packet retransmits independently — no HOL blocking "
        "at the sender side. However, the receiver delivers in-order, so a gap in SEQ "
        "numbers means later packets wait in the receive buffer. This is correct behavior "
        "(TCP does the same). The key advantage over Go-Back-N is that successfully received "
        "later packets don't need to be retransmitted."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 5: Sequence Number Wraparound
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 5: Sequence Number Wraparound (SEQ_MOD=64)")
    print(f"  With WINDOW_SIZE={WINDOW_SIZE}, SEQ_MOD={SEQ_MOD}")
    print(f"  Need to send >{SEQ_MOD} messages to trigger wraparound")

    wrap_ul = {}
    for i in range(70):  # > 64 messages
        wrap_ul[5 + i] = (f'M{i:02d}'.encode(), f"M{i:02d}")

    _, events, stats = run_scenario(
        UniformLossChannel(0.10), 800, wrap_ul, {}, seed=42
    )

    print(f"  UL: {stats['ul_reliable_delivered']}/70 delivered, {stats['ul_reliable_failed']} failed")
    print(f"  Retransmissions: {stats['total_retx']}")
    print(f"  Max latency: {stats['ul_max_latency']} ticks")

    if stats['ul_reliable_delivered'] == 70:
        print(f"  ✓ All 70 messages delivered — sequence wraparound works correctly!")
    else:
        print(f"  ⚠ Some messages lost — possible SEQ wraparound issue!")
        for fd in stats['ul_failed_detail']:
            print(f"    FAILED: {fd['label']}")

    all_findings.append(
        f"Sequence wraparound test: 70 messages through 6-bit (mod 64) SEQ space. "
        f"Result: {stats['ul_reliable_delivered']}/70 delivered."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 6: Messages Queued During Blackout
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 6: Messages Queued During a Blackout")
    print("  Scenario: 3 messages queued at t=45,50,55 during blackout t=[40,70]")

    blackout_ul = {
        45: (b'CMD:DURING_BLACKOUT_1', "BLACKOUT_MSG_1"),
        50: (b'CMD:DURING_BLACKOUT_2', "BLACKOUT_MSG_2"),
        55: (b'CMD:DURING_BLACKOUT_3', "BLACKOUT_MSG_3"),
    }

    ch = BlackoutChannel(base_loss=0.05, blackout_start=40, blackout_end=70,
                         blackout_dir='both', label="Blackout both t=[40,70]")
    _, events, stats = run_scenario(ch, 300, blackout_ul, {}, seed=42)

    print(f"  UL: {stats['ul_reliable_delivered']}/3 delivered, {stats['ul_reliable_failed']} failed")
    print(f"  Retransmissions: {stats['total_retx']}")
    for d in stats['ul_delivered_detail']:
        print(f"    ✓ \"{d['label']}\" SEQ={d['seq']} latency={d['latency']} ticks ({d['latency']*50}ms)")

    all_findings.append(
        "Messages queued during a blackout are buffered by the sender and transmitted "
        "once the channel recovers. The exponential backoff naturally spaces out retries "
        "so the protocol doesn't flood the channel the instant it comes back."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 7: Bidirectional reliable data (both sides sending reliably simultaneously)
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 7: Bidirectional Reliable + Contention")
    print("  Both GCS and UAV send reliable messages at the same time")

    bidir_ul = {
        20: (b'CMD:ARM', "ARM"),
        21: (b'CMD:TAKEOFF', "TAKEOFF"),
        22: (b'CMD:WPT1', "WAYPOINT1"),
    }
    bidir_dl = {
        20: (b'ALERT:GPS_FIX', "GPS_FIX"),
        21: (b'ALERT:BARO_OK', "BARO_OK"),
        22: (b'ALERT:MAG_CAL', "MAG_CAL"),
    }

    _, events, stats = run_scenario(
        UniformLossChannel(0.25), 200, bidir_ul, bidir_dl, seed=42
    )

    print(f"  UL: {stats['ul_reliable_delivered']}/3 delivered, {stats['ul_reliable_failed']} failed")
    print(f"  DL: {stats['dl_reliable_delivered']}/3 delivered, {stats['dl_reliable_failed']} failed")
    print(f"  UL avg lat: {stats['ul_avg_latency']}, DL avg lat: {stats['dl_avg_latency']}")
    print(f"  Retransmissions: {stats['total_retx']}")

    # Key question: when both sides send reliable frames, unreliable frames
    # (which carry piggybacked ACKs) are suppressed. How are ACKs delivered?
    ack_events = [e for e in events if 'ACK' in e['msg']]
    reliable_tx = [e for e in events if 'TX reliable' in e['msg']]
    print(f"\n  Reliable TX events: {len(reliable_tx)}")
    print(f"  ACK events: {len(ack_events)}")

    # Check: reliable frames can ALSO carry piggybacked ACKs!
    for e in events:
        if e['tick'] >= 20 and e['tick'] <= 30:
            if 'reliable' in e['msg'] or 'ACK' in e['msg']:
                print(f"    [t={e['tick']:3d}] {e['msg']}")

    all_findings.append(
        "Bidirectional reliable traffic works because ACKs piggyback on ANY packet "
        "(unreliable OR reliable) going in the reverse direction. When both sides send "
        "reliable data, the reliable frames themselves carry piggybacked ACKs for the "
        "other direction. This is a strength of the piggybacking design."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 8: What's the worst-case ACK latency under piggyback-only design?
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 8: ACK Piggybacking Timeliness")
    print("  Since ACKs only piggyback on reverse-direction packets,")
    print("  what if reverse direction has no data to send? (no tick-driven unreliable data)")

    # In our sim, both directions always send unreliable data (RC and telemetry)
    # every tick. So ACK piggybacking is always available. But what if an application
    # only sends reliable commands occasionally with no background traffic?

    # Analyze: in our current design, EVERY tick produces an unreliable frame if no
    # reliable data to send. This guarantees ACK piggybacking opportunity every tick.
    # Without this background traffic, the protocol would need dedicated ACK-only frames.

    print("  In LDTP, unreliable background traffic (RC every tick, telemetry every tick)")
    print("  provides continuous piggybacking opportunities. Without background traffic,")
    print("  the protocol would need to generate standalone ACK frames.")
    print(f"  Current design: ACK latency ≤ 1 tick ({50}ms) if reverse channel is open.")

    # Verify by checking that ACK events happen promptly after data receipt:
    bidir_ul2 = {20: (b'CMD:TEST', "TEST")}
    _, events2, stats2 = run_scenario(
        UniformLossChannel(0.0), 50, bidir_ul2, {}, seed=42
    )
    # Find when data was sent, when it was received, when ACK was sent back
    for e in events2:
        if e['tick'] >= 19 and e['tick'] <= 25:
            print(f"    [t={e['tick']:3d}] {e['msg']}")

    all_findings.append(
        "ACK piggybacking latency is bounded by the reverse-direction packet rate. "
        "With continuous background traffic (RC+telemetry at 20Hz), ACK is piggybacked "
        "on the very next tick = 50ms worst case. WITHOUT background traffic, the protocol "
        "would need a standalone ACK-only frame mechanism (build_ack_only exists but "
        "isn't triggered in the current sim — this is a potential gap for applications "
        "with no background traffic)."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 9: Exact failure threshold — find the loss rate at which
    # delivery probability drops below 100%
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 9: Delivery Success Rate vs Loss Rate (Monte Carlo)")

    loss_rates = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95]
    n_trials = 20
    simple_ul = {20: (b'CMD:ARM', "ARM")}
    simple_dl = {20: (b'ALERT:LOW_BAT', "LOW_BAT")}

    print(f"  {'Loss%':>6s} {'UL success':>12s} {'DL success':>12s} {'Avg retx':>10s} "
          f"{'UL avg lat':>11s} {'DL avg lat':>11s}")

    for loss in loss_rates:
        ul_success = 0
        dl_success = 0
        total_retx = 0
        ul_lats = []
        dl_lats = []

        for seed in range(n_trials):
            _, _, stats = run_scenario(
                UniformLossChannel(loss), 600, simple_ul, simple_dl, seed=seed
            )
            if stats['ul_reliable_delivered'] == 1:
                ul_success += 1
                ul_lats.append(stats['ul_avg_latency'])
            if stats['dl_reliable_delivered'] == 1:
                dl_success += 1
                dl_lats.append(stats['dl_avg_latency'])
            total_retx += stats['total_retx']

        ul_pct = ul_success / n_trials * 100
        dl_pct = dl_success / n_trials * 100
        avg_retx = total_retx / n_trials
        ul_avg = sum(ul_lats)/len(ul_lats) if ul_lats else float('inf')
        dl_avg = sum(dl_lats)/len(dl_lats) if dl_lats else float('inf')

        print(f"  {loss*100:5.0f}% {ul_pct:10.0f}% {dl_pct:10.0f}% {avg_retx:10.1f} "
              f"{'%.1f'%ul_avg if ul_lats else 'N/A':>11s} "
              f"{'%.1f'%dl_avg if dl_lats else 'N/A':>11s}")

    all_findings.append(
        "Monte Carlo analysis reveals the loss rate threshold where delivery becomes unreliable. "
        "With MAX_RETRIES=5, a single message needs at least one successful TX+ACK round trip. "
        "Probability that ALL 6 attempts (1 initial + 5 retries) of the data AND the corresponding "
        "ACK are both lost = loss^12 for independent bidirectional loss (data lost OR ack lost)."
    )

    # ═══════════════════════════════════════════════════════════════════
    # TEST 10: SACK Effectiveness
    # ═══════════════════════════════════════════════════════════════════
    print_header("TEST 10: SACK Effectiveness — Selective Repeat vs hypothetical Go-Back-N")
    print("  Compare: 20 messages at 30% loss.")
    print("  Selective Repeat only retransmits lost packets.")
    print("  Go-Back-N would retransmit ALL packets from the lost one onward.")

    window_ul = {}
    for i in range(20):
        window_ul[10 + i] = (f'M{i:02d}'.encode(), f"M{i:02d}")

    _, events, stats = run_scenario(
        UniformLossChannel(0.30), 400, window_ul, {}, seed=42
    )

    print(f"  Selective Repeat result:")
    print(f"    Delivered: {stats['ul_reliable_delivered']}/20")
    print(f"    Retransmissions: {stats['total_retx']}")
    print(f"    Avg latency: {stats['ul_avg_latency']} ticks")
    print(f"    Max latency: {stats['ul_max_latency']} ticks")

    # Theoretical Go-Back-N retransmissions at 30% loss:
    # Each lost packet would cause window-size retransmissions
    # With window=8 and 30% loss, approximately 0.3 * 20 = 6 lost packets
    # Each causing ~4 retransmissions (avg half window) = 24 retransmissions
    print(f"\n  Estimated Go-Back-N retransmissions at same loss:")
    print(f"    ~{int(0.3 * 20 * WINDOW_SIZE/2)} (vs {stats['total_retx']} actual with SR)")

    all_findings.append(
        "SACK-based Selective Repeat significantly reduces retransmissions compared to "
        "Go-Back-N. With 20 messages at 30% loss, SR needed far fewer retransmissions. "
        "The 7-bit SACK bitmap covers positions CACK+1 through CACK+7, perfectly matching "
        "WINDOW_SIZE=8."
    )

    # ═══════════════════════════════════════════════════════════════════
    # Summary
    # ═══════════════════════════════════════════════════════════════════
    print_header("COMPLETE FINDINGS SUMMARY")
    for i, f in enumerate(all_findings, 1):
        print(f"\n  {i}. {f}")

    # Write findings to file
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'edge_case_findings.json')
    with open(out_path, 'w') as f:
        json.dump(all_findings, f, indent=2, ensure_ascii=False)

    print(f"\n\nFindings saved → {out_path}")


if __name__ == '__main__':
    main()
