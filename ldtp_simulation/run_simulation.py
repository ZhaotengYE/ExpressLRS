#!/usr/bin/env python3
"""
LDTP Protocol Simulation
=========================
Simulates a GCS ↔ UAV full-duplex LoRa link using the LDTP protocol.

Data flows:
  UL unreliable : GCS → UAV  RC channel data         (every tick)
  UL reliable   : GCS → UAV  Commands                (occasional)
  DL unreliable : UAV → GCS  Telemetry               (every tick)
  DL reliable   : UAV → GCS  Safety alerts            (occasional)

Channel model: each packet independently lost with probability LOSS_RATE.
Output: simulation_results.html  (self-contained visualization)
"""

import json, random, os, struct, sys
from ldtp_protocol import (
    build_unreliable, build_reliable, build_ack_only, decode_frame,
    ReliableSender, ReliableReceiver
)

# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════
TICK_HZ       = 20          # packets per second per direction
TICK_MS       = 1000 // TICK_HZ
TOTAL_TICKS   = 100         # 5 seconds
LOSS_RATE     = 0.15        # 15% independent packet loss
SEED          = 42

# Application-layer schedule  {tick: (payload_bytes, label)}
UL_RELIABLE_SCHEDULE = {
    20: (b'WPT:LAT32.05,LON118.78,ALT100', "SET_WAYPOINT"),
    50: (b'CMD:ARM_MOTORS',                 "ARM"),
    70: (b'CMD:RETURN_TO_HOME',             "RTH"),
}
DL_RELIABLE_SCHEDULE = {
    30: (b'ALERT:LOW_BATTERY_22PCT',        "LOW_BATTERY"),
    60: (b'ALERT:GPS_DEGRADED_HDOP8',       "GPS_DEGRADED"),
}

# Type IDs for unreliable channel
UL_TYPE_RC   = 0
DL_TYPE_ATT  = 0
DL_TYPE_GPS  = 1
DL_TYPE_BATT = 2

# Reliable stream IDs
UL_STREAM_CMD   = 0
DL_STREAM_ALERT = 0

random.seed(SEED)

# ═══════════════════════════════════════════════════════════════════════════
# Simulated application data generators
# ═══════════════════════════════════════════════════════════════════════════

def make_rc_payload(tick):
    """16 channels × 11 bits ≈ 22 bytes. Simplified to 16 bytes."""
    vals = [1500 + int(200 * (((tick * 7 + ch * 13) % 100) / 100.0 - 0.5))
            for ch in range(8)]
    return struct.pack('<8H', *vals)

_telem_cycle = [DL_TYPE_ATT, DL_TYPE_GPS, DL_TYPE_BATT, DL_TYPE_ATT]
_telem_names = {DL_TYPE_ATT: "Attitude", DL_TYPE_GPS: "GPS", DL_TYPE_BATT: "Battery"}

def make_telemetry(tick):
    """Rotate through telemetry types."""
    t = _telem_cycle[tick % len(_telem_cycle)]
    if t == DL_TYPE_ATT:
        data = struct.pack('<3f', 5.2 + tick*0.1, -1.3, 180.0 + tick*0.5)  # roll,pitch,yaw
    elif t == DL_TYPE_GPS:
        data = struct.pack('<2d', 32.05 + tick*1e-5, 118.78 + tick*1e-5)    # lat,lon
    else:
        data = struct.pack('<HBB', 1180 - tick, 22 + tick//10, 3)           # mV, %, cells
    return t, data, _telem_names[t]

# ═══════════════════════════════════════════════════════════════════════════
# Channel model
# ═══════════════════════════════════════════════════════════════════════════

def channel(frame_bytes):
    """Returns frame_bytes if delivered, None if lost."""
    if random.random() < LOSS_RATE:
        return None
    return frame_bytes

# ═══════════════════════════════════════════════════════════════════════════
# Simulation
# ═══════════════════════════════════════════════════════════════════════════

def run():
    # -- protocol instances --
    ul_rel_sender   = ReliableSender(UL_STREAM_CMD)
    ul_rel_receiver = ReliableReceiver(UL_STREAM_CMD)
    dl_rel_sender   = ReliableSender(DL_STREAM_ALERT)
    dl_rel_receiver = ReliableReceiver(DL_STREAM_ALERT)

    timeline = []       # per-tick records
    event_log = []      # flat list of human-readable events

    def log(tick, msg):
        event_log.append({'tick': tick, 'msg': msg})

    for t in range(TOTAL_TICKS):
        tick_record = {'tick': t, 'uplink': None, 'downlink': None}

        # --- 0. Timers -----------------------------------------------
        for ev in ul_rel_sender.tick(t):
            log(t, f"UL reliable: {ev['event']} SEQ={ev.get('seq')}")
        for ev in dl_rel_sender.tick(t):
            log(t, f"DL reliable: {ev['event']} SEQ={ev.get('seq')}")

        # --- 1. Application layer enqueue ----------------------------
        if t in UL_RELIABLE_SCHEDULE:
            payload, label = UL_RELIABLE_SCHEDULE[t]
            ul_rel_sender.queue_message(payload, label)
            log(t, f"APP  GCS enqueues reliable: \"{label}\" ({len(payload)} bytes)")

        if t in DL_RELIABLE_SCHEDULE:
            payload, label = DL_RELIABLE_SCHEDULE[t]
            dl_rel_sender.queue_message(payload, label)
            log(t, f"APP  UAV enqueues reliable: \"{label}\" ({len(payload)} bytes)")

        # --- 2. Prepare uplink frame (GCS → UAV) --------------------
        # Piggyback ACK for DL reliable data we received
        dl_ack = dl_rel_receiver.get_ack()

        ul_frame = None
        ul_meta = {'direction': 'uplink', 'tick': t}

        if ul_rel_sender.has_work():
            frame, meta = ul_rel_sender.get_frame(ack_info=dl_ack, now=t)
            if frame:
                ul_frame = frame
                ul_meta.update({
                    'channel': 'reliable', 'stream': UL_STREAM_CMD,
                    'seq': meta['seq'], 'label': meta['label'],
                    'is_retx': meta['is_retx'], 'retx_count': meta['retx_count'],
                    'has_ack': dl_ack is not None,
                })
                retx_str = f" (RETX #{meta['retx_count']})" if meta['is_retx'] else ""
                log(t, f"UL TX reliable  STREAM={UL_STREAM_CMD} SEQ={meta['seq']} "
                       f"\"{meta['label']}\"{retx_str}  [{len(frame)}B]")

        if ul_frame is None:
            rc_data = make_rc_payload(t)
            ul_frame = build_unreliable(UL_TYPE_RC, rc_data, ack=dl_ack)
            ul_meta.update({
                'channel': 'unreliable', 'type': UL_TYPE_RC,
                'type_name': 'RC Channels', 'has_ack': dl_ack is not None,
            })
            log(t, f"UL TX unreliable TYPE=0 (RC Channels)  [{len(ul_frame)}B]"
                   + (f"  +ACK(DL CACK={dl_ack[1]})" if dl_ack else ""))

        # --- 3. Prepare downlink frame (UAV → GCS) ------------------
        ul_ack = ul_rel_receiver.get_ack()

        dl_frame = None
        dl_meta = {'direction': 'downlink', 'tick': t}

        if dl_rel_sender.has_work():
            frame, meta = dl_rel_sender.get_frame(ack_info=ul_ack, now=t)
            if frame:
                dl_frame = frame
                dl_meta.update({
                    'channel': 'reliable', 'stream': DL_STREAM_ALERT,
                    'seq': meta['seq'], 'label': meta['label'],
                    'is_retx': meta['is_retx'], 'retx_count': meta['retx_count'],
                    'has_ack': ul_ack is not None,
                })
                retx_str = f" (RETX #{meta['retx_count']})" if meta['is_retx'] else ""
                log(t, f"DL TX reliable  STREAM={DL_STREAM_ALERT} SEQ={meta['seq']} "
                       f"\"{meta['label']}\"{retx_str}  [{len(frame)}B]")

        if dl_frame is None:
            ttype, tdata, tname = make_telemetry(t)
            dl_frame = build_unreliable(ttype, tdata, ack=ul_ack)
            dl_meta.update({
                'channel': 'unreliable', 'type': ttype,
                'type_name': tname, 'has_ack': ul_ack is not None,
            })
            log(t, f"DL TX unreliable TYPE={ttype} ({tname})  [{len(dl_frame)}B]"
                   + (f"  +ACK(UL CACK={ul_ack[1]})" if ul_ack else ""))

        # --- 4. Channel simulation -----------------------------------
        ul_rx = channel(ul_frame)
        dl_rx = channel(dl_frame)

        ul_meta['lost'] = (ul_rx is None)
        dl_meta['lost'] = (dl_rx is None)
        ul_meta['frame_hex'] = ul_frame.hex()
        dl_meta['frame_hex'] = dl_frame.hex()
        ul_meta['decoded'] = decode_frame(ul_frame)
        dl_meta['decoded'] = decode_frame(dl_frame)

        if ul_meta['lost']:
            log(t, "UL ✗ LOST in channel")
        else:
            log(t, f"UL ✓ delivered  ({len(ul_frame)}B)")
        if dl_meta['lost']:
            log(t, "DL ✗ LOST in channel")
        else:
            log(t, f"DL ✓ delivered  ({len(dl_frame)}B)")

        # --- 5. GCS processes received DL frame ----------------------
        if dl_rx is not None:
            d = decode_frame(dl_rx)
            if d:
                # ACK block → feed to UL sender
                if d['ack_present']:
                    evts = ul_rel_sender.process_ack(d['ack_sid'], d['ack_cack'], d['ack_sack'], now=t)
                    for ev in evts:
                        log(t, f"GCS got ACK for UL: {ev['event']} SEQ={ev['seq']} "
                               f"\"{ev.get('label','')}\" latency={ev.get('latency',0)} ticks")
                # Reliable payload → DL receiver
                if d['reliable']:
                    new = dl_rel_receiver.receive(d['seq'], d['frag'], d['more_frag'], d['payload'])
                    for msg in dl_rel_receiver.pop_delivered():
                        log(t, f"GCS APP received reliable DL: {msg[:40]}")
                else:
                    if d['type_or_stream'] != 7:  # not ACK-only
                        pass  # unreliable telemetry delivered to app (implicit)

        # --- 6. UAV processes received UL frame ----------------------
        if ul_rx is not None:
            d = decode_frame(ul_rx)
            if d:
                if d['ack_present']:
                    evts = dl_rel_sender.process_ack(d['ack_sid'], d['ack_cack'], d['ack_sack'], now=t)
                    for ev in evts:
                        log(t, f"UAV got ACK for DL: {ev['event']} SEQ={ev['seq']} "
                               f"\"{ev.get('label','')}\" latency={ev.get('latency',0)} ticks")
                if d['reliable']:
                    new = ul_rel_receiver.receive(d['seq'], d['frag'], d['more_frag'], d['payload'])
                    for msg in ul_rel_receiver.pop_delivered():
                        log(t, f"UAV APP received reliable UL: {msg[:40]}")
                else:
                    pass  # RC channels delivered to app (implicit)

        tick_record['uplink'] = ul_meta
        tick_record['downlink'] = dl_meta
        timeline.append(tick_record)

    # --- Summary statistics ------------------------------------------
    ul_total  = sum(1 for r in timeline)
    dl_total  = sum(1 for r in timeline)
    ul_lost   = sum(1 for r in timeline if r['uplink']['lost'])
    dl_lost   = sum(1 for r in timeline if r['downlink']['lost'])
    ul_rel_tx = sum(1 for r in timeline if r['uplink']['channel'] == 'reliable')
    dl_rel_tx = sum(1 for r in timeline if r['downlink']['channel'] == 'reliable')

    stats = {
        'config': {
            'tick_hz': TICK_HZ, 'tick_ms': TICK_MS,
            'total_ticks': TOTAL_TICKS, 'loss_rate': LOSS_RATE, 'seed': SEED,
        },
        'ul_total': ul_total, 'dl_total': dl_total,
        'ul_lost': ul_lost, 'dl_lost': dl_lost,
        'ul_loss_pct': round(100*ul_lost/ul_total, 1),
        'dl_loss_pct': round(100*dl_lost/dl_total, 1),
        'ul_reliable_frames': ul_rel_tx,
        'dl_reliable_frames': dl_rel_tx,
        'ul_reliable_delivered': len(ul_rel_sender.delivered),
        'ul_reliable_failed': len(ul_rel_sender.failed),
        'dl_reliable_delivered': len(dl_rel_sender.delivered),
        'dl_reliable_failed': len(dl_rel_sender.failed),
        'ul_avg_latency': (round(sum(d[2] for d in ul_rel_sender.delivered) /
                                 len(ul_rel_sender.delivered), 2)
                           if ul_rel_sender.delivered else 0),
        'dl_avg_latency': (round(sum(d[2] for d in dl_rel_sender.delivered) /
                                 len(dl_rel_sender.delivered), 2)
                           if dl_rel_sender.delivered else 0),
        'ul_delivered_detail': [
            {'seq': s, 'label': l, 'latency': la}
            for s, l, la in ul_rel_sender.delivered
        ],
        'dl_delivered_detail': [
            {'seq': s, 'label': l, 'latency': la}
            for s, l, la in dl_rel_sender.delivered
        ],
    }

    return timeline, event_log, stats

# ═══════════════════════════════════════════════════════════════════════════
# HTML Visualization Generator
# ═══════════════════════════════════════════════════════════════════════════

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>LDTP Protocol Simulation</title>
<style>
  :root { --bg:#1a1a2e; --card:#16213e; --accent:#0f3460; --hi:#e94560;
          --ok:#4ecca3; --warn:#f0a500; --txt:#eee; --dim:#888; }
  * { margin:0; padding:0; box-sizing:border-box; }
  body { background:var(--bg); color:var(--txt); font-family:'SF Mono',Consolas,monospace;
         font-size:13px; padding:20px; }
  h1 { font-size:20px; margin-bottom:4px; color:var(--ok); }
  h2 { font-size:15px; margin:16px 0 8px; color:var(--warn); }
  .subtitle { color:var(--dim); margin-bottom:16px; }

  /* --- Config bar --- */
  .config { display:flex; gap:20px; background:var(--card); padding:10px 16px;
            border-radius:8px; margin-bottom:16px; flex-wrap:wrap; }
  .config span { color:var(--dim); }
  .config b { color:var(--ok); }

  /* --- Stats --- */
  .stats { display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr));
           gap:10px; margin-bottom:16px; }
  .stat-card { background:var(--card); padding:12px; border-radius:8px; }
  .stat-card .val { font-size:22px; font-weight:bold; color:var(--ok); }
  .stat-card .lbl { color:var(--dim); font-size:11px; }

  /* --- Timeline --- */
  .timeline-wrapper { overflow-x:auto; margin-bottom:16px; background:var(--card);
                      border-radius:8px; padding:12px; }
  .timeline-label { color:var(--dim); font-size:11px; margin-bottom:4px; }
  .timeline { display:flex; gap:2px; min-width:max-content; margin-bottom:8px; }
  .pkt { width:22px; height:32px; border-radius:3px; cursor:pointer; display:flex;
         align-items:center; justify-content:center; font-size:9px; font-weight:bold;
         transition: transform 0.1s; position:relative; }
  .pkt:hover { transform:scaleY(1.3); z-index:10; }
  .pkt.unrel       { background:#2d6a4f; color:#b7e4c7; }
  .pkt.rel          { background:#1d3557; color:#a8dadc; border:2px solid #457b9d; }
  .pkt.rel.retx     { background:#6a040f; color:#fec89a; border:2px solid #d62828; }
  .pkt.lost         { background:#333; color:#666; }
  .pkt.lost::after  { content:'✗'; color:var(--hi); font-size:14px; }
  .pkt .ack-dot     { position:absolute; top:1px; right:1px; width:5px; height:5px;
                      background:var(--warn); border-radius:50%; }

  /* --- Detail panel --- */
  .detail { background:var(--card); border-radius:8px; padding:16px; margin-bottom:16px;
            min-height:120px; }
  .detail h3 { color:var(--warn); margin-bottom:8px; }
  .detail .field { margin-bottom:3px; }
  .detail .field .k { color:var(--dim); display:inline-block; width:140px; }
  .detail .hex { background:#111; padding:8px; border-radius:4px; margin-top:6px;
                 word-break:break-all; color:#7ec8e3; font-size:12px; }
  .detail .bits { margin-top:4px; }
  .detail .bits span { display:inline-block; padding:1px 4px; margin:1px; border-radius:2px;
                       font-size:11px; }
  .detail .bits .b-fc  { background:#1d3557; }
  .detail .bits .b-ack { background:#5c4033; }
  .detail .bits .b-seq { background:#2d6a4f; }
  .detail .bits .b-pay { background:#333; }

  /* --- Event log --- */
  .log-panel { background:var(--card); border-radius:8px; padding:12px;
               max-height:360px; overflow-y:auto; }
  .log-line { padding:2px 0; border-bottom:1px solid #222; font-size:12px; }
  .log-line .lt { color:var(--dim); display:inline-block; width:50px; }
  .log-line .lost-text { color:var(--hi); }
  .log-line .ok-text { color:var(--ok); }
  .log-line .rel-text { color:#a8dadc; }

  /* --- Reliable message tracker --- */
  .msg-tracker { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr));
                 gap:10px; margin-bottom:16px; }
  .msg-card { background:var(--card); padding:12px; border-radius:8px;
              border-left:4px solid var(--ok); }
  .msg-card.failed { border-left-color:var(--hi); }
  .msg-card .dir { font-size:11px; color:var(--dim); }
  .msg-card .name { font-size:14px; font-weight:bold; margin:4px 0; }
  .msg-card .info { font-size:12px; color:var(--dim); }
</style>
</head>
<body>

<h1>LDTP Protocol Simulation Dashboard</h1>
<p class="subtitle">LoRa Dual-Channel Transport Protocol — Full Duplex Simulation</p>

<div class="config" id="config"></div>

<div class="stats" id="stats"></div>

<h2>Reliable Message Tracking</h2>
<div class="msg-tracker" id="msg-tracker"></div>

<h2>Packet Timeline (← scroll →)</h2>
<div class="timeline-wrapper">
  <div class="timeline-label">GCS → UAV (Uplink)</div>
  <div class="timeline" id="tl-ul"></div>
  <div class="timeline-label">UAV → GCS (Downlink)</div>
  <div class="timeline" id="tl-dl"></div>
</div>

<h2>Packet Detail</h2>
<div class="detail" id="detail"><i style="color:var(--dim)">Click a packet in the timeline to inspect its frame.</i></div>

<h2>Event Log</h2>
<div class="log-panel" id="log"></div>

<script>
// ====== EMBEDDED SIMULATION DATA ======
const DATA = __DATA_JSON__;

// ====== Render ======
function init() {
  renderConfig();
  renderStats();
  renderMsgTracker();
  renderTimeline();
  renderLog();
}

function renderConfig() {
  const c = DATA.stats.config;
  document.getElementById('config').innerHTML =
    `<span>Packet Rate: <b>${c.tick_hz} Hz</b></span>
     <span>Tick: <b>${c.tick_ms} ms</b></span>
     <span>Duration: <b>${c.total_ticks} ticks (${(c.total_ticks*c.tick_ms/1000).toFixed(1)}s)</b></span>
     <span>Loss Rate: <b>${(c.loss_rate*100).toFixed(0)}%</b></span>
     <span>Seed: <b>${c.seed}</b></span>`;
}

function statCard(val, label) {
  return `<div class="stat-card"><div class="val">${val}</div><div class="lbl">${label}</div></div>`;
}

function renderStats() {
  const s = DATA.stats;
  document.getElementById('stats').innerHTML =
    statCard(s.ul_total + s.dl_total, 'Total Packets') +
    statCard(s.ul_lost + s.dl_lost, `Lost (${((s.ul_lost+s.dl_lost)*100/(s.ul_total+s.dl_total)).toFixed(1)}%)`) +
    statCard(s.ul_reliable_delivered + '/' + (s.ul_reliable_delivered + s.ul_reliable_failed), 'UL Reliable Delivered') +
    statCard(s.dl_reliable_delivered + '/' + (s.dl_reliable_delivered + s.dl_reliable_failed), 'DL Reliable Delivered') +
    statCard(s.ul_avg_latency + ' ticks', 'UL Avg Latency') +
    statCard(s.dl_avg_latency + ' ticks', 'DL Avg Latency');
}

function renderMsgTracker() {
  const el = document.getElementById('msg-tracker');
  let html = '';
  for (const m of DATA.stats.ul_delivered_detail) {
    html += `<div class="msg-card"><div class="dir">UL (GCS → UAV)</div>
      <div class="name">${m.label}</div>
      <div class="info">SEQ=${m.seq} | Latency: ${m.latency} ticks (${m.latency * DATA.stats.config.tick_ms}ms)</div></div>`;
  }
  for (const m of DATA.stats.dl_delivered_detail) {
    html += `<div class="msg-card"><div class="dir">DL (UAV → GCS)</div>
      <div class="name">${m.label}</div>
      <div class="info">SEQ=${m.seq} | Latency: ${m.latency} ticks (${m.latency * DATA.stats.config.tick_ms}ms)</div></div>`;
  }
  el.innerHTML = html || '<i style="color:var(--dim)">No reliable messages.</i>';
}

function pktClass(meta) {
  if (meta.lost) return 'pkt lost';
  if (meta.channel === 'reliable') return 'pkt rel' + (meta.is_retx ? ' retx' : '');
  return 'pkt unrel';
}

function pktLabel(meta) {
  if (meta.lost) return '';
  if (meta.channel === 'reliable') return 'R' + (meta.seq !== undefined ? meta.seq : '');
  return {0:'RC',1:'GP',2:'BT'}[meta.type] || 'U';
}

function renderTimeline() {
  const ulEl = document.getElementById('tl-ul');
  const dlEl = document.getElementById('tl-dl');
  let ulH = '', dlH = '';
  for (const rec of DATA.timeline) {
    const u = rec.uplink, d = rec.downlink;
    const uDot = u.has_ack ? '<div class="ack-dot"></div>' : '';
    const dDot = d.has_ack ? '<div class="ack-dot"></div>' : '';
    ulH += `<div class="${pktClass(u)}" data-dir="uplink" data-tick="${rec.tick}"
                title="t=${rec.tick}">${pktLabel(u)}${uDot}</div>`;
    dlH += `<div class="${pktClass(d)}" data-dir="downlink" data-tick="${rec.tick}"
                title="t=${rec.tick}">${pktLabel(d)}${dDot}</div>`;
  }
  ulEl.innerHTML = ulH;
  dlEl.innerHTML = dlH;

  // click handlers
  document.querySelectorAll('.pkt').forEach(el => {
    el.addEventListener('click', () => {
      const tick = parseInt(el.dataset.tick);
      const dir = el.dataset.dir;
      showDetail(tick, dir);
    });
  });
}

function hexBlock(hex, headerLen) {
  // Color-code hex bytes by field
  let out = '';
  for (let i = 0; i < hex.length; i += 2) {
    const byteIdx = i / 2;
    let cls = 'b-pay';
    if (byteIdx === 0) cls = 'b-fc';
    else if (byteIdx < headerLen && byteIdx > 0) {
      // Determine if ack or seq
      cls = 'b-ack';
    }
    out += `<span class="${cls}">${hex.substring(i, i+2)}</span>`;
  }
  return out;
}

function sackBits(sack) {
  let s = '';
  for (let i = 6; i >= 0; i--) s += (sack >> i) & 1;
  return s;
}

function showDetail(tick, dir) {
  const rec = DATA.timeline[tick];
  const meta = rec[dir];
  const d = meta.decoded;
  const el = document.getElementById('detail');

  let html = `<h3>Tick ${tick} | ${dir === 'uplink' ? 'GCS → UAV' : 'UAV → GCS'} | `;
  if (meta.lost) html += `<span style="color:var(--hi)">LOST IN CHANNEL</span>`;
  else html += `<span style="color:var(--ok)">DELIVERED</span>`;
  html += `</h3>`;

  html += `<div class="field"><span class="k">Channel:</span> ${meta.channel}</div>`;
  if (meta.channel === 'reliable') {
    html += `<div class="field"><span class="k">Stream ID:</span> ${meta.stream}</div>`;
    html += `<div class="field"><span class="k">SEQ:</span> ${meta.seq}</div>`;
    html += `<div class="field"><span class="k">Label:</span> ${meta.label}</div>`;
    if (meta.is_retx) html += `<div class="field"><span class="k">Retransmission:</span> #${meta.retx_count}</div>`;
  } else {
    html += `<div class="field"><span class="k">Type:</span> ${meta.type_name || meta.type}</div>`;
  }

  // Frame Control decode
  html += `<div class="field"><span class="k">Frame Control:</span> 0x${d.raw_hex.substring(0,2).toUpperCase()}  `;
  html += `R=${d.reliable} A=${d.ack_present} F=${d.more_frag} TYPE=${d.type_or_stream} VER=${d.version}</div>`;

  if (d.ack_present) {
    html += `<div class="field"><span class="k">ACK Block:</span> SID=${d.ack_sid} CACK=${d.ack_cack} SACK=0b${sackBits(d.ack_sack)}</div>`;
  }
  if (d.reliable) {
    html += `<div class="field"><span class="k">Seq Control:</span> SEQ=${d.seq} FRAG=${d.frag}</div>`;
  }

  html += `<div class="field"><span class="k">Payload:</span> ${d.payload_hex.length/2} bytes</div>`;
  html += `<div class="field"><span class="k">Total Frame:</span> ${d.total_len} bytes (header: ${d.header_bytes}B)</div>`;

  // Hex dump with coloring
  html += `<div class="hex"><div class="bits">${hexBlock(meta.frame_hex, d.header_bytes)}</div></div>`;
  html += `<div style="font-size:11px;color:var(--dim);margin-top:4px;">
    <span class="bits"><span class="b-fc">FC</span> <span class="b-ack">ACK/SEQ</span> <span class="b-pay">Payload</span></span></div>`;

  // Show payload as ASCII if printable
  if (d.payload_hex) {
    try {
      let ascii = '';
      for (let i = 0; i < d.payload_hex.length; i += 2) {
        const c = parseInt(d.payload_hex.substring(i, i+2), 16);
        ascii += (c >= 32 && c < 127) ? String.fromCharCode(c) : '.';
      }
      html += `<div class="field" style="margin-top:4px"><span class="k">Payload ASCII:</span> ${ascii}</div>`;
    } catch(e) {}
  }

  el.innerHTML = html;
}

function renderLog() {
  const el = document.getElementById('log');
  let html = '';
  for (const e of DATA.event_log) {
    let cls = '';
    if (e.msg.includes('LOST')) cls = 'lost-text';
    else if (e.msg.includes('delivered') || e.msg.includes('✓')) cls = 'ok-text';
    else if (e.msg.includes('reliable')) cls = 'rel-text';
    html += `<div class="log-line"><span class="lt">[t=${e.tick}]</span> <span class="${cls}">${e.msg}</span></div>`;
  }
  el.innerHTML = html;
}

document.addEventListener('DOMContentLoaded', init);
</script>
</body>
</html>"""

# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    print("Running LDTP simulation...")
    print(f"  {TOTAL_TICKS} ticks @ {TICK_HZ} Hz = {TOTAL_TICKS*TICK_MS/1000:.1f}s")
    print(f"  Packet loss rate: {LOSS_RATE*100:.0f}%")
    print(f"  Seed: {SEED}")
    print()

    timeline, event_log, stats = run()

    # Convert bytes in decoded frames to hex strings for JSON serialization
    for rec in timeline:
        for d in ['uplink', 'downlink']:
            dec = rec[d].get('decoded')
            if dec and 'payload' in dec:
                dec['payload_hex'] = dec['payload'].hex()
                del dec['payload']

    data = {'timeline': timeline, 'event_log': event_log, 'stats': stats}
    data_json = json.dumps(data, indent=None, ensure_ascii=False)

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'simulation_results.html')
    html = HTML_TEMPLATE.replace('__DATA_JSON__', data_json)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)

    # Print summary
    print("═" * 60)
    print("SIMULATION RESULTS")
    print("═" * 60)
    print(f"Uplink:   {stats['ul_total']} packets, {stats['ul_lost']} lost ({stats['ul_loss_pct']}%)")
    print(f"Downlink: {stats['dl_total']} packets, {stats['dl_lost']} lost ({stats['dl_loss_pct']}%)")
    print(f"UL reliable:  {stats['ul_reliable_delivered']} delivered, "
          f"{stats['ul_reliable_failed']} failed, avg latency {stats['ul_avg_latency']} ticks")
    print(f"DL reliable:  {stats['dl_reliable_delivered']} delivered, "
          f"{stats['dl_reliable_failed']} failed, avg latency {stats['dl_avg_latency']} ticks")
    print()
    for m in stats['ul_delivered_detail']:
        print(f"  UL ✓ SEQ={m['seq']}  \"{m['label']}\"  latency={m['latency']} ticks ({m['latency']*TICK_MS}ms)")
    for m in stats['dl_delivered_detail']:
        print(f"  DL ✓ SEQ={m['seq']}  \"{m['label']}\"  latency={m['latency']} ticks ({m['latency']*TICK_MS}ms)")
    print()
    print(f"Visualization → {out_path}")

if __name__ == '__main__':
    main()
