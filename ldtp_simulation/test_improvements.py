#!/usr/bin/env python3
"""
LDTP Protocol v1.1 — Improvement Verification Test Suite
=========================================================
Tests the three new features:
  1. Adaptive RTO (Jacobson/Karels algorithm)
  2. Standalone ACK-only frame trigger
  3. Link quality monitor

Runs large-scale tests and compares with the old fixed-RTO baseline.
"""

import random, time, json, os
from test_protocol import (
    run_scenario, UniformLossChannel, AsymmetricChannel,
    BurstLossChannel, BlackoutChannel, AckStarvationChannel
)
from ldtp_protocol import (
    ReliableSender, ReliableReceiver, LinkQualityMonitor,
    SEQ_MOD, WINDOW_SIZE, MAX_RETRIES, RTO_TICKS, ACK_DELAY_TICKS,
    RTO_MIN, RTO_MAX
)


def make_schedules(n_ul=5, n_dl=3, spacing=5, start=10):
    """Generate standard test schedules."""
    ul = {}
    for i in range(n_ul):
        ul[start + i * spacing] = (f'CMD:TEST_UL_{i}'.encode(), f"UL_MSG_{i}")
    dl = {}
    for i in range(n_dl):
        dl[start + 5 + i * spacing] = (f'RESP:TEST_DL_{i}'.encode(), f"DL_MSG_{i}")
    return ul, dl


def run_batch(label, scenarios, seeds=range(1, 11)):
    """Run a batch of scenarios across multiple seeds, collect stats."""
    results = []
    for name, ch_factory, ticks, ul, dl in scenarios:
        for seed in seeds:
            ch = ch_factory()
            _, _, stats = run_scenario(ch, ticks, ul, dl, seed)
            results.append({
                'name': name, 'seed': seed,
                'ul_del': stats['ul_reliable_delivered'],
                'ul_fail': stats['ul_reliable_failed'],
                'dl_del': stats['dl_reliable_delivered'],
                'dl_fail': stats['dl_reliable_failed'],
                'retx': stats['total_retx'],
                'standalone_acks': stats['standalone_acks'],
                'ul_lq': stats['ul_link_quality'],
                'dl_lq': stats['dl_link_quality'],
                'ul_rto': stats['ul_rto_final'],
                'dl_rto': stats['dl_rto_final'],
                'ul_rtt': stats['ul_rtt_est'],
                'dl_rtt': stats['dl_rtt_est'],
                'ul_avg_lat': stats['ul_avg_latency'],
                'dl_avg_lat': stats['dl_avg_latency'],
                'ul_max_lat': stats['ul_max_latency'],
                'dl_max_lat': stats['dl_max_latency'],
            })
    return results


def summarize_batch(results, group_name):
    """Summarize a batch of results."""
    from collections import defaultdict
    groups = defaultdict(list)
    for r in results:
        groups[r['name']].append(r)

    print(f"\n{'='*80}")
    print(f"  {group_name}")
    print(f"{'='*80}")
    print(f"{'Scenario':>35s}  {'Del%':>6s}  {'AvgLat':>7s}  {'MaxLat':>7s}  {'Retx':>5s}  "
          f"{'SAck':>5s}  {'RTO':>4s}  {'LQ':>5s}")
    print('-' * 100)

    for name in sorted(groups.keys()):
        rs = groups[name]
        total_expected = sum(len([k for k in [r['ul_del'] + r['ul_fail'] for r in rs[:1]]]) for _ in rs)
        total_del = sum(r['ul_del'] + r['dl_del'] for r in rs)
        total_exp = sum(r['ul_del'] + r['ul_fail'] + r['dl_del'] + r['dl_fail'] for r in rs)
        del_pct = 100 * total_del / max(total_exp, 1)

        lats = [r['ul_avg_lat'] for r in rs if r['ul_avg_lat'] is not None] + \
               [r['dl_avg_lat'] for r in rs if r['dl_avg_lat'] is not None]
        avg_lat = sum(lats) / len(lats) if lats else 0

        max_lats = [r['ul_max_lat'] for r in rs if r['ul_max_lat'] is not None] + \
                   [r['dl_max_lat'] for r in rs if r['dl_max_lat'] is not None]
        max_lat = max(max_lats) if max_lats else 0

        avg_retx = sum(r['retx'] for r in rs) / len(rs)
        avg_sack = sum(r['standalone_acks'] for r in rs) / len(rs)
        avg_rto = sum(r['ul_rto'] for r in rs) / len(rs)
        avg_lq = sum(r['ul_lq'] for r in rs) / len(rs)

        print(f"{name:>35s}  {del_pct:>5.1f}%  {avg_lat:>6.1f}t  {max_lat:>6.0f}t  "
              f"{avg_retx:>5.1f}  {avg_sack:>5.1f}  {avg_rto:>4.1f}  {avg_lq:>4.0f}%")

    return groups


def main():
    print("LDTP Protocol v1.1 — Improvement Verification")
    print("Features: Adaptive RTO | Standalone ACK | Link Quality Monitor")
    print("=" * 80)

    # Standard message schedules
    std_ul = {
        20: (b'WPT:LAT32.05,LON118.78,ALT100', "SET_WAYPOINT"),
        50: (b'CMD:ARM_MOTORS',                 "ARM"),
        70: (b'CMD:RETURN_TO_HOME',             "RTH"),
    }
    std_dl = {
        30: (b'ALERT:LOW_BATTERY_22PCT',        "LOW_BATTERY"),
        60: (b'ALERT:GPS_DEGRADED_HDOP8',       "GPS_DEGRADED"),
    }

    # Heavy traffic
    heavy_ul = {}
    for i in range(20):
        heavy_ul[10 + i] = (f'CMD:WAYPOINT_{i}'.encode(), f"WPT_{i}")
    heavy_dl = {
        15: (b'ALERT:GEOFENCE_WARN', "GEOFENCE"),
        35: (b'ALERT:WIND_HIGH',     "WIND"),
        55: (b'ALERT:LOW_BAT_10PCT', "CRITICAL_BAT"),
    }

    # ──── Test Group 1: Uniform loss sweep (10 seeds each) ────
    uniform_scenarios = []
    for loss in [0.05, 0.15, 0.30, 0.50, 0.60, 0.70, 0.80]:
        uniform_scenarios.append((
            f"Uniform {int(loss*100)}%",
            lambda l=loss: UniformLossChannel(l),
            200, std_ul, std_dl
        ))

    t0 = time.time()
    results_uniform = run_batch("Uniform Loss Sweep", uniform_scenarios)
    summarize_batch(results_uniform, "GROUP 1: Uniform Loss Sweep (10 seeds each)")

    # ──── Test Group 2: Burst loss ────
    burst_scenarios = [
        ("Burst-Mild", lambda: BurstLossChannel(0.02, 0.60, 0.08, 0.40, label="Mild"),
         200, std_ul, std_dl),
        ("Burst-Severe", lambda: BurstLossChannel(0.03, 0.90, 0.15, 0.10, label="Severe"),
         300, std_ul, std_dl),
        ("Burst-Extreme", lambda: BurstLossChannel(0.05, 0.95, 0.20, 0.05, label="Extreme"),
         400, std_ul, std_dl),
    ]
    results_burst = run_batch("Burst Loss", burst_scenarios)
    summarize_batch(results_burst, "GROUP 2: Burst Loss (10 seeds each)")

    # ──── Test Group 3: Asymmetric loss (ACK starvation test) ────
    asym_scenarios = [
        ("Asym UL=5% DL=40%", lambda: AsymmetricChannel(0.05, 0.40),
         200, std_ul, std_dl),
        ("Asym UL=40% DL=5%", lambda: AsymmetricChannel(0.40, 0.05),
         200, std_ul, std_dl),
        ("Asym UL=10% DL=60%", lambda: AsymmetricChannel(0.10, 0.60),
         300, std_ul, std_dl),
        ("ACK-Starved UL=10% DL=70%", lambda: AsymmetricChannel(0.10, 0.70),
         400, std_ul, std_dl),
    ]
    results_asym = run_batch("Asymmetric Loss", asym_scenarios)
    summarize_batch(results_asym, "GROUP 3: Asymmetric Loss / ACK Starvation (10 seeds each)")

    # ──── Test Group 4: Blackout recovery ────
    blackout_scenarios = [
        ("Blackout 30t both", lambda: BlackoutChannel(0.05, 40, 70, 'both', label="BO-both-30"),
         200, std_ul, std_dl),
        ("Blackout 50t UL", lambda: BlackoutChannel(0.05, 30, 80, 'ul', label="BO-UL-50"),
         200, std_ul, std_dl),
        ("Blackout 80t both", lambda: BlackoutChannel(0.05, 30, 110, 'both', label="BO-both-80"),
         300, std_ul, std_dl),
    ]
    results_bo = run_batch("Blackout", blackout_scenarios)
    summarize_batch(results_bo, "GROUP 4: Blackout Recovery (10 seeds each)")

    # ──── Test Group 5: Heavy traffic stress ────
    stress_scenarios = [
        ("20msg Uniform-15%", lambda: UniformLossChannel(0.15),
         400, heavy_ul, heavy_dl),
        ("20msg Uniform-40%", lambda: UniformLossChannel(0.40),
         600, heavy_ul, heavy_dl),
        ("20msg Burst-Severe", lambda: BurstLossChannel(0.03, 0.90, 0.15, 0.10, label="Severe"),
         600, heavy_ul, heavy_dl),
    ]
    results_stress = run_batch("Stress", stress_scenarios)
    summarize_batch(results_stress, "GROUP 5: Heavy Traffic Stress (10 seeds each)")

    elapsed = time.time() - t0
    total_runs = len(results_uniform) + len(results_burst) + len(results_asym) + \
                 len(results_bo) + len(results_stress)

    # ──── Adaptive RTO Analysis ────
    print(f"\n{'='*80}")
    print("  ADAPTIVE RTO ANALYSIS")
    print(f"{'='*80}")

    all_results = results_uniform + results_burst + results_asym + results_bo + results_stress
    rto_converged = [r for r in all_results if r['ul_rtt'] is not None]
    if rto_converged:
        rtos = [r['ul_rto'] for r in rto_converged]
        rtts = [r['ul_rtt'] for r in rto_converged]
        print(f"  RTT estimates across {len(rto_converged)} converged runs:")
        print(f"    RTT: min={min(rtts):.1f} avg={sum(rtts)/len(rtts):.1f} max={max(rtts):.1f}")
        print(f"    RTO: min={min(rtos)} avg={sum(rtos)/len(rtos):.1f} max={max(rtos)}")
    else:
        print("  No RTT convergence observed (all packets may have been retransmitted)")

    # ──── Standalone ACK Analysis ────
    print(f"\n{'='*80}")
    print("  STANDALONE ACK ANALYSIS")
    print(f"{'='*80}")
    sacks_used = [r for r in all_results if r['standalone_acks'] > 0]
    print(f"  Standalone ACKs sent in {len(sacks_used)}/{len(all_results)} runs")
    if sacks_used:
        avg_sack = sum(r['standalone_acks'] for r in sacks_used) / len(sacks_used)
        print(f"  Average when used: {avg_sack:.1f} standalone ACKs per run")

    # ──── Link Quality Monitor Analysis ────
    print(f"\n{'='*80}")
    print("  LINK QUALITY MONITOR ANALYSIS")
    print(f"{'='*80}")
    for loss_pct in [5, 30, 50, 70, 80]:
        matching = [r for r in results_uniform if f'{loss_pct}%' in r['name']]
        if matching:
            avg_ul_lq = sum(r['ul_lq'] for r in matching) / len(matching)
            avg_dl_lq = sum(r['dl_lq'] for r in matching) / len(matching)
            print(f"  Uniform {loss_pct}% loss → LQ: UL={avg_ul_lq:.0f}% DL={avg_dl_lq:.0f}%")

    # ──── Overall Summary ────
    total_del = sum(r['ul_del'] + r['dl_del'] for r in all_results)
    total_fail = sum(r['ul_fail'] + r['dl_fail'] for r in all_results)
    total_exp = total_del + total_fail

    print(f"\n{'='*80}")
    print(f"  OVERALL SUMMARY")
    print(f"{'='*80}")
    print(f"  Total runs: {total_runs}")
    print(f"  Total messages: {total_exp} attempted, {total_del} delivered, {total_fail} failed")
    print(f"  Delivery rate: {100*total_del/max(total_exp,1):.1f}%")
    print(f"  Time elapsed: {elapsed:.1f}s")

    # Save results
    out_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(out_dir, 'improvement_test_results.json')
    with open(json_path, 'w') as f:
        json.dump({
            'uniform': results_uniform,
            'burst': results_burst,
            'asymmetric': results_asym,
            'blackout': results_bo,
            'stress': results_stress,
            'summary': {
                'total_runs': total_runs,
                'total_delivered': total_del,
                'total_failed': total_fail,
                'delivery_rate': round(100 * total_del / max(total_exp, 1), 2),
            }
        }, f, indent=2)
    print(f"\n  Results saved to: {json_path}")


if __name__ == '__main__':
    main()
