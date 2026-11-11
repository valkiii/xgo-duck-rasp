'use strict';

const $ = (s, r = document) => r.querySelector(s);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

const STEPS = [
  { id: 'parts', phase: 'On the bench', title: 'What connects to what', sub: 'Parts and the URT-2' },
  { id: 'urt', title: 'Connect the URT-2', sub: 'Servo board on this Mac' },
  { id: 'ids', title: 'Give each servo its ID', sub: 'One servo at a time' },
  { id: 'assemble', title: 'Assemble and check', sub: 'All 15 servos answering' },
  { id: 'sd', phase: 'Raspberry Pi', title: 'Prepare the SD card', sub: 'Pi OS 64-bit and SSH' },
  { id: 'install', title: 'Install the software', sub: 'Copy, install, reboot' },
  { id: 'imu', title: 'IMU wiring and orientation', sub: 'MPU6050 on the Pi' },
  { id: 'run', phase: 'The robot', title: 'Start the robot software', sub: 'Service and live status' },
  { id: 'calibrate', title: 'Calibrate and first moves', sub: 'Zero, pose, walk' },
];
const GROUPS = [['Left leg', [10, 11, 12, 13, 14]], ['Right leg', [20, 21, 22, 23, 24]], ['Neck and head', [30, 31, 32, 33]], ['Mouth', [34]]];
const ALL_IDS = GROUPS.flatMap((g) => g[1]);
const GUIDE = 'https://github.com/LuwuDynamics/xgoduck_hardware/blob/main/Assembly_Guide.pdf';
const VIDEO = 'https://www.youtube.com/watch?v=nElGguHO5dY';

let O = null;                 // overview from the server
const ui = { logs: {}, jobs: {} };
let current = 0;
try { current = Math.max(0, STEPS.findIndex((s) => s.id === localStorage.getItem('xgo-step'))); } catch (e) { /* storage off */ }

// ---- server ------------------------------------------------------------
async function api(path, body) {
  const opt = body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Wizard': '1' }, body: JSON.stringify(body) };
  const r = await fetch(path, opt);
  const x = await r.json().catch(() => ({ detail: r.statusText }));
  if (!r.ok) throw new Error(x.detail || 'request failed');
  return x;
}
async function refresh() { O = await api('/api/overview'); }
async function saveState(patch) { O.state = await api('/api/state', patch); }

function toast(msg, bad = false) {
  const t = $('#toast');
  t.textContent = msg; t.className = 'toast show' + (bad ? ' bad' : '');
  clearTimeout(toast.timer); toast.timer = setTimeout(() => { t.className = 'toast' + (bad ? ' bad' : ''); }, bad ? 6000 : 3000);
}

// Run an action with a busy button; errors become a toast and stay on screen in ui.err[key].
async function act(btn, key, fn) {
  if (btn) { btn.disabled = true; btn.classList.add('busy'); }
  ui.err = ui.err || {};
  delete ui.err[key];
  try { await fn(); } catch (e) { ui.err[key] = e.message; toast(e.message, true); }
  finally { if (btn) { btn.disabled = false; btn.classList.remove('busy'); } await refresh().catch(() => {}); render(); }
}
const errBox = (key) => (ui.err && ui.err[key] ? `<div class="result bad"><span class="bad-text">✕</span> ${esc(ui.err[key])}</div>` : '');

// ---- jobs (long commands with streamed output) --------------------------------
async function watchJob(name, job) {
  ui.jobs[name] = job; ui.logs[name] = [...job.lines];
  let next = job.next;
  render();
  while (true) {
    await new Promise((r) => setTimeout(r, 700));
    const j = await api(`/api/jobs/${job.id}?since=${next}`).catch(() => null);
    if (!j) break;
    next = j.next; ui.logs[name].push(...j.lines); ui.jobs[name] = j;
    const pre = document.getElementById('log-' + name);
    if (pre) { pre.textContent = ui.logs[name].join('\n'); pre.scrollTop = pre.scrollHeight; }
    const st = document.getElementById('status-' + name);
    if (st) st.innerHTML = jobStatus(j);
    if (j.status !== 'running') { render(); return j; }
  }
}
function jobStatus(j) {
  if (!j) return '';
  if (j.status === 'running') return `<span class="warn-text">running · ${j.seconds}s</span>`;
  return j.status === 'ok' ? `<span class="ok-text">✓ finished in ${j.seconds}s</span>` : `<span class="bad-text">✕ failed (exit ${j.code})</span>`;
}
function jobPanel(name) {
  const j = ui.jobs[name];
  if (!j) return '';
  return `<div class="row"><span id="status-${name}">${jobStatus(j)}</span></div><pre class="log" id="log-${name}">${esc((ui.logs[name] || []).join('\n'))}</pre>`;
}

// ---- layout ------------------------------------------------------------------
function chips() {
  const b = O.bench, s = O.sim, host = O.state.host;
  return [
    `<span class="chip"><span class="dot ${b.connected ? 'on' : ''}"></span>URT-2 <b>${b.connected ? esc(b.simulated ? 'simulator' : b.device.replace('/dev/', '')) : 'not connected'}</b></span>`,
    `<span class="chip"><span class="dot ${host ? 'warn' : ''}"></span>Pi <b>${esc(host || 'not set')}</b></span>`,
    `<span class="chip"><span class="dot ${s.running ? 'on' : ''}"></span>Simulated robot ${s.running
      ? `<a class="button" href="${esc(s.url)}" target="_blank" rel="noopener">Open</a><button data-act="simStop">Stop</button>`
      : '<button data-act="simStart">Try it</button>'}</span>`,
  ].join('');
}

function nav() {
  let html = '';
  STEPS.forEach((s, i) => {
    if (s.phase) html += `<div class="phase">${esc(s.phase)}</div>`;
    const done = O.state.steps_done.includes(s.id);
    html += `<button class="step-link ${done ? 'done' : ''}" data-act="go" data-i="${i}" ${i === current ? 'aria-current="step"' : ''}>
      <span class="num">${done ? '✓' : i + 1}</span><span><span class="t">${esc(s.title)}</span><span class="s">${esc(s.sub)}</span></span></button>`;
  });
  return html;
}

function nextbar() {
  const s = STEPS[current], done = O.state.steps_done.includes(s.id);
  return `<div class="nextbar">
    <span>${current > 0 ? `<button data-act="go" data-i="${current - 1}">← ${esc(STEPS[current - 1].title)}</button>` : ''}</span>
    <span class="row" style="margin:0">
      <button data-act="toggleDone" class="${done ? '' : 'primary'}">${done ? 'Done ✓ (undo)' : 'Mark this step done'}</button>
      ${current < STEPS.length - 1 ? `<button data-act="go" data-i="${current + 1}">${esc(STEPS[current + 1].title)} →</button>` : ''}
    </span></div>`;
}

function render() {
  if (!O) return;
  $('#chips').innerHTML = chips();
  $('#steps').innerHTML = nav();
  const s = STEPS[current];
  const keep = document.activeElement && document.activeElement.id;
  $('#main').innerHTML = `<small>Step ${current + 1} of ${STEPS.length}</small>${VIEWS[s.id]()}${nextbar()}`;
  if (keep && document.getElementById(keep)) document.getElementById(keep).focus();
  for (const [name] of Object.entries(ui.jobs)) {
    const pre = document.getElementById('log-' + name);
    if (pre) pre.scrollTop = pre.scrollHeight;
  }
  const here = document.querySelector('.step-link[aria-current="step"]');
  if (here && window.matchMedia('(max-width: 860px)').matches) here.scrollIntoView({ inline: 'center', block: 'nearest' });
  onEnter(s.id);
}

// ---- views -------------------------------------------------------------------
const diagram = `
<svg class="diagram" viewBox="0 0 760 300" role="img" aria-label="Wiring: battery through switch to the URT-2 power terminal; URT-2 TTL ports to the servos; URT-2 USB to the Pi; MPU6050 to the Pi GPIO pins">
  <defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="#9cafc3"/></marker></defs>
  <g font-family="system-ui,sans-serif" font-size="13" fill="#dce5ef">
    <rect x="10" y="40" width="130" height="56" rx="10" fill="#192a3a" stroke="#3c5b78"/><text x="75" y="64" text-anchor="middle" font-weight="600">2S battery</text><text x="75" y="83" text-anchor="middle" fill="#9cafc3" font-size="12">7.4–8.4 V</text>
    <rect x="180" y="40" width="110" height="56" rx="10" fill="#192a3a" stroke="#3c5b78"/><text x="235" y="64" text-anchor="middle" font-weight="600">Switch</text><text x="235" y="83" text-anchor="middle" fill="#9cafc3" font-size="12">in the + wire</text>
    <rect x="330" y="20" width="170" height="130" rx="12" fill="#0f2a25" stroke="#6de1c4"/><text x="415" y="50" text-anchor="middle" font-weight="700" fill="#8ce2cc">FE-URT-2</text><text x="415" y="70" text-anchor="middle" fill="#9cafc3" font-size="12">servo board</text>
    <text x="340" y="98" fill="#9cafc3" font-size="11">power terminal</text><text x="415" y="140" text-anchor="middle" fill="#9cafc3" font-size="11">3-pin TTL ports</text><text x="490" y="98" text-anchor="end" fill="#9cafc3" font-size="11">USB-C</text>
    <rect x="330" y="215" width="170" height="60" rx="10" fill="#192a3a" stroke="#3c5b78"/><text x="415" y="240" text-anchor="middle" font-weight="600">15 servos</text><text x="415" y="259" text-anchor="middle" fill="#9cafc3" font-size="12">daisy-chained</text>
    <rect x="600" y="20" width="150" height="130" rx="12" fill="#192a3a" stroke="#3c5b78"/><text x="675" y="52" text-anchor="middle" font-weight="600">Pi Zero 2 W</text><text x="675" y="72" text-anchor="middle" fill="#9cafc3" font-size="12">runs the AI + web page</text><text x="675" y="122" text-anchor="middle" fill="#9cafc3" font-size="12">PiSugar 3 underneath</text><text x="675" y="138" text-anchor="middle" fill="#9cafc3" font-size="11">(powers the Pi only)</text>
    <rect x="600" y="215" width="150" height="60" rx="10" fill="#192a3a" stroke="#3c5b78"/><text x="675" y="240" text-anchor="middle" font-weight="600">MPU6050</text><text x="675" y="259" text-anchor="middle" fill="#9cafc3" font-size="12">motion sensor</text>
  </g>
  <g stroke="#9cafc3" stroke-width="1.6" fill="none">
    <path d="M140 68H178" marker-end="url(#a)"/><path d="M290 68H328" marker-end="url(#a)"/>
    <path d="M415 150V213" marker-end="url(#a)"/><path d="M500 85H598" marker-end="url(#a)" marker-start="url(#a)"/>
    <path d="M675 215V152" marker-end="url(#a)"/>
  </g>
  <g font-family="system-ui,sans-serif" font-size="11.5" fill="#f1ca86">
    <text x="549" y="64" text-anchor="middle">USB</text><text x="549" y="78" text-anchor="middle">OTG cable</text><text x="425" y="190">one data wire + power</text><text x="665" y="180" text-anchor="end">I²C to the Pi</text><text x="665" y="195" text-anchor="end">pins 1, 3, 5, 6, 17</text>
  </g>
</svg>`;

const CHECKLIST = [
  ['kit', 'Servo kit: 15 × HD-1910 and the FE-URT-2 board', 'Your Alibaba kit'],
  ['cables', 'One 3-pin servo cable per servo (about 15 cm)', 'Check the servo boxes; order 15 AMP 2.0 mm 3-pin cables if they are missing'],
  ['battery', '2S Li-ion battery (AOLIKES 7.4 V 2600 mAh, XH2.54 plug)', 'Powers the servos'],
  ['charger', '8.4 V 2A Li-ion charger', 'Charge outside the robot until the LED turns green'],
  ['switch', 'Rocker switch and 4.8 mm female spade connectors', 'In the + wire between battery and URT-2'],
  ['connectors', 'DC jack screw adapters and the XH2.54 2-pin leads', 'Check the lead has pins that fit the battery plug'],
  ['imu', 'MPU6050 board and female-to-female jumper wires', ''],
  ['usbcable', 'USB-C to micro-USB cable', 'URT-2 to the Pi Zero data port'],
  ['pi', 'Pi Zero 2 W (with GPIO header), microSD card, PiSugar 3', 'No header? Use a solder-free hammer header'],
  ['bearings', 'Bearings: 11 × 16×22×4 mm, 2 × 10×15×3 mm', 'The thin 3 mm ones go in the feet'],
  ['screws', 'Screws: M2×6 countersunk, M2.3×8, M3×16, M2.5×6 servo horn screws', ''],
  ['printed', 'All printed parts, TPU feet and beak', ''],
  ['meter', 'Multimeter', 'For polarity checks before anything is plugged in'],
];

const VIEWS = {
  parts: () => `<h2>What connects to what</h2>
    <p class="lead">The FE-URT-2 that came with your servos is the servo board. During setup it plugs into this Mac. On the robot it plugs into the Pi's USB port and carries all 15 servos on one shared data wire.</p>
    <div class="callout">It replaces the servo part of the Luwu board. The two other things that board did are separate parts here: the <b>MPU6050</b> motion sensor (wired to the Pi) and the <b>power switch</b> (between battery and URT-2).</div>
    <div class="card">${diagram}</div>
    <div class="card"><h3>Parts checklist</h3>${CHECKLIST.map(([k, t, n]) => `<label class="check"><input type="checkbox" data-act="check" data-k="${k}" ${O.state.checklist[k] ? 'checked' : ''}><span>${esc(t)}${n ? `<small>${esc(n)}</small>` : ''}</span></label>`).join('')}</div>`,

  urt: () => {
    const b = O.bench, ports = ui.ports || [];
    const scan = ui.scan;
    return `<h2>Connect the URT-2</h2>
    <p class="lead">Plug the URT-2 into this Mac first. Setting servo IDs happens here, before the servos go into the robot.</p>
    <div class="card"><ol class="howto">
      <li>Connect the URT-2's <b>USB-C</b> port to this Mac.</li>
      <li>Give it servo power on the <b>blue screw terminal</b>. For setting IDs, your 5 V adapter through a DC-jack screw adapter is enough. Later the battery goes there through the switch. Match + and − to the terminal labels and check with the multimeter first.</li>
      <li>Plug one servo into a <b>3-pin TTL</b> port, not the 4-pin RS485 ones.</li>
    </ol></div>
    <div class="card"><h3>Adapter</h3>
      <div class="row"><select id="port" aria-label="Serial port">${ports.map((p) => `<option value="${esc(p.device)}" ${p.device === b.device ? 'selected' : ''}>${esc(p.device)}${p.description ? ' · ' + esc(p.description) : ''}</option>`).join('') || '<option>Click “Find ports”</option>'}</select>
      <button data-act="ports">Find ports</button>
      ${b.connected ? '<button data-act="disconnect">Disconnect</button>' : '<button class="primary" data-act="connect">Connect</button>'}</div>
      ${b.connected ? `<div class="result ok"><span class="ok-text">✓</span> Connected to <code>${esc(b.device)}</code> · adapter echo ${b.echo ? 'detected (handled)' : 'off'}</div>` : '<p class="muted">On a Mac the URT-2 usually shows up as <code>/dev/cu.usbmodem…</code> or <code>/dev/cu.wchusbserial…</code>. The two Simulator entries work without any hardware.</p>'}
      ${errBox('connect')}</div>
    <div class="card"><h3>Who is on the bus?</h3>
      <div class="row"><button data-act="scan" ${b.connected ? '' : 'disabled'}>Scan IDs 0–253</button><span class="muted">takes about 2 seconds</span></div>
      ${scan ? (scan.ids.length ? `<table><tr><th>ID</th><th>Joint</th><th>Position</th></tr>${scan.ids.map((i) => `<tr><td>${i}</td><td>${esc(O.joints[i] || (i === 1 ? 'new servo (factory ID 1)' : ''))}</td><td>${scan.positions[i] ?? '—'}</td></tr>`).join('')}</table>` : `<div class="result bad">No servo answered. Check: servo power on the screw terminal, the servo is in a TTL port, the cable is fully in, and the right port is selected above.</div>`) : ''}
      ${errBox('scan')}</div>`;
  },

  ids: () => {
    const done = new Set(O.state.servos_done), b = O.bench;
    if (ui.target == null || done.has(ui.target)) ui.target = ALL_IDS.find((i) => !done.has(i)) ?? ui.target ?? 10;
    const t = ui.target, det = ui.detect;
    let flow = '';
    if (!b.connected) flow = `<div class="callout warn">Connect the URT-2 in the previous step first.</div>`;
    else {
      flow = `<ol class="howto"><li>Plug in <b>one</b> servo that has not been set up yet (nothing else on the bus).</li><li>Detect it, then set it to ID <b>${t}</b>.</li><li>Put a label “${t}” on it straight away.</li></ol>
      <div class="row"><button data-act="detect">Detect servo</button>
      ${det && det.ids.length === 1 ? `<button class="primary" data-act="setup">Set ID ${det.ids[0]} → ${t}, center, store gains</button>` : ''}
      ${b.simulated && b.device === 'sim:bench' ? '<button class="link" data-act="simNew">Simulator: plug in the next new servo</button>' : ''}</div>`;
      if (det) {
        if (det.ids.length === 0) flow += `<div class="result bad">No servo found. Is the servo powered and in a TTL port?</div>`;
        else if (det.ids.length > 1) flow += `<div class="result bad">${det.ids.length} servos answer (IDs ${det.ids.join(', ')}). Unplug all but the one you are setting.</div>`;
        else {
          const id = det.ids[0];
          flow += `<div class="result">Found one servo at ID <b>${id}</b>${id === 1 ? ' (factory default)' : ''}, position ${det.positions[id] ?? '—'}.${done.has(id) && id !== t ? ` <span class="warn-text">ID ${id} is already marked done: is this the ${esc(O.joints[id])} servo again?</span>` : ''}</div>`;
        }
      }
      if (ui.setupResult) flow += `<div class="result ok"><span class="ok-text">✓ Servo is now ID ${ui.setupResult.id}.</span> ${ui.setupResult.log.map(esc).join(' · ')}<br><b>Label it “${ui.setupResult.id}”</b>, then plug in the next one.</div>`;
      flow += errBox('ids');
    }
    const tiles = GROUPS.map(([label, ids]) => `<div class="group-label">${label}</div><div class="ids">${ids.map((i) => `<button class="id-tile ${done.has(i) ? 'done' : ''} ${i === t ? 'selected' : ''}" data-act="pick" data-id="${i}"><b>${i}</b><small>${esc(O.joints[i])}</small></button>`).join('')}</div>`).join('');
    return `<h2>Give each servo its ID</h2>
    <p class="lead">Every new servo answers at ID 1, so they get their final ID one at a time, before assembly. This is the same sequence as upstream's Servo setup page: change the ID, move to the center (raw 2047), and store gains KP 5 / KD 20.</p>
    <div class="card"><h3>Servo for ID ${t}: ${esc(O.joints[t])}</h3>${flow}</div>
    <div class="card"><h3>Progress: ${done.size} of 15</h3><p class="muted">Page 1 of the <a href="${GUIDE}" target="_blank" rel="noopener">assembly guide</a> shows where each ID goes. Click a tile to choose which ID to set next.</p>${tiles}
      ${done.has(t) ? `<div class="row"><button data-act="wiggle" data-id="${t}" ${b.connected ? '' : 'disabled'}>Wiggle ID ${t}</button><button class="link" data-act="undoServo">Mark ID ${t} as not done</button></div>` : ''}</div>`;
  },

  assemble: () => {
    const b = O.bench, chain = ui.chain;
    return `<h2>Assemble and check</h2>
    <p class="lead">Build the duck with the <a href="${GUIDE}" target="_blank" rel="noopener">25-page assembly guide</a> (and Luwu's <a href="${VIDEO}" target="_blank" rel="noopener">assembly video</a>), then check that every servo answers.</p>
    <div class="card"><ul class="howto">
      <li>Page 2: keep the secondary horns on IDs 30 and 31; don't remove output shafts.</li>
      <li>Pages 11 and 14: the thin 10×15×3 bearings go in the feet, with M2.5×6 screws.</li>
      <li>Connect cables before a joint gets closed, and leave slack through each joint's full movement.</li>
      <li>The servo chains end at the URT-2's TTL ports. If there are more chains than ports, a Y-split cable joins two.</li>
    </ul></div>
    <div class="card"><h3>All 15 on the bus</h3>
      <p class="muted">With the URT-2 still on this Mac and powered, scan the assembled robot. “Wiggle” moves one joint a few degrees and back, so you can check each ID sits in the right place. Only that one servo gets torque.</p>
      <div class="row"><button class="primary" data-act="chain" ${b.connected ? '' : 'disabled'}>Scan for all 15</button>${b.connected ? '' : '<span class="muted">Connect the URT-2 in step 2 (or pick “Simulator: assembled robot”).</span>'}</div>
      ${chain ? `<p>${chain.ids.filter((i) => ALL_IDS.includes(i)).length} of 15 answering${chain.missing.length ? ` · <span class="bad-text">missing ${chain.missing.join(', ')}</span>` : ' · <span class="ok-text">complete</span>'}</p>
      ${GROUPS.map(([label, ids]) => `<div class="group-label">${label}</div><div class="ids">${ids.map((i) => {
        const ok = chain.ids.includes(i);
        return `<div class="id-tile ${ok ? 'done' : 'missing'}"><b>${i}</b><small>${esc(O.joints[i])}<br>${ok ? 'pos ' + (chain.positions[i] ?? '—') : 'no reply'}</small>${ok ? `<div style="margin-top:6px"><button data-act="wiggle" data-id="${i}">Wiggle</button></div>` : ''}</div>`;
      }).join('')}</div>`).join('')}
      ${chain.missing.length ? '<div class="callout warn">A missing ID usually means a loose cable on that chain, or that servo never got its ID. A whole chain missing means its first cable or the port.</div>' : ''}` : ''}
      ${errBox('chain')}</div>`;
  },

  sd: () => {
    const key = ui.key;
    return `<h2>Prepare the SD card</h2>
    <p class="lead">The Pi needs the <b>64-bit</b> Raspberry Pi OS. The AI library has no 32-bit version. Raspberry Pi Imager can set Wi-Fi and SSH up front, so the Pi never needs a screen.</p>
    <div class="card"><ol class="howto">
      <li>Install and open <a href="https://www.raspberrypi.com/software/" target="_blank" rel="noopener">Raspberry Pi Imager</a>.</li>
      <li>Device: <b>Raspberry Pi Zero 2 W</b>. Operating system: <b>Raspberry Pi OS (other) → Raspberry Pi OS Lite (64-bit)</b>.</li>
      <li>In the customisation settings, set hostname <code>xgoduck</code>, a username and password, and your Wi-Fi (country NL).</li>
      <li>Under Services, turn on <b>SSH</b> with <b>public-key authentication</b> and paste the key below.</li>
      <li>Write the card, put it in the Pi, power it from the PiSugar, and wait 2–3 minutes for the first boot.</li>
    </ol></div>
    <div class="card"><h3>This Mac's SSH key</h3>
      ${key ? (key.key ? `<div class="keybox" id="sshkey">${esc(key.key)}</div><div class="row"><button data-act="copyKey">Copy key</button><span class="muted">${esc(key.path)}</span></div>` : `<p>No SSH key on this Mac yet.</p><div class="row"><button class="primary" data-act="makeKey">Create one</button><span class="muted">creates ~/.ssh/id_ed25519 (no passphrase)</span></div>`) : '<p class="muted">Loading…</p>'}
      ${errBox('key')}</div>
    <div class="card"><h3>Connect to the Pi</h3>
      <div class="row"><input type="text" id="host" value="${esc(O.state.host || '')}" placeholder="user@xgoduck.local" aria-label="Pi address" autocomplete="off"><button data-act="saveHost">Save</button><button class="primary" data-act="probe" ${O.state.host ? '' : 'disabled'}>Test connection</button></div>
      ${ui.probe ? (ui.probe.ok ? `<div class="result ${ui.probe.arch_ok ? 'ok' : 'bad'}">${ui.probe.arch_ok ? '<span class="ok-text">✓ Connected.</span>' : '<span class="bad-text">✕ Wrong system.</span>'} ${esc(ui.probe.os)} · ${esc(ui.probe.arch)} · ${esc(ui.probe.python)}${ui.probe.message ? '<br>' + esc(ui.probe.message) : ''}</div>` : `<div class="result bad"><span class="bad-text">✕</span> ${esc(ui.probe.message)}</div>`) : ''}
      ${errBox('probe')}</div>`;
  },

  install: () => {
    const p = ui.probe;
    return `<h2>Install the software on the Pi</h2>
    <p class="lead">This copies the runtime to <code>~/xgo-duck</code> on the Pi and installs it. The install adds Python packages, turns on I²C at 400 kHz, gives your user access to the serial port and I²C, and sets the robot software to start at boot. On a Pi Zero it takes roughly 10–20 minutes.</p>
    ${O.state.host ? '' : '<div class="callout warn">Enter and test the Pi address in the previous step first.</div>'}
    <div class="card"><h3>1. Copy the files</h3><div class="row"><button class="primary" data-act="deploy" ${O.state.host ? '' : 'disabled'}>Copy to the Pi</button><span class="muted">safe to repeat; never deletes anything on the Pi and never touches your calibration</span></div>${jobPanel('deploy')}${errBox('deploy')}</div>
    <div class="card"><h3>2. Install</h3><div class="row"><button class="primary" data-act="install" ${O.state.host ? '' : 'disabled'}>Install on the Pi</button></div>${jobPanel('install')}${errBox('install')}</div>
    <div class="card"><h3>3. Reboot</h3><p class="muted">Needed once, so the I²C speed and group changes apply.</p>
      <div class="row"><button data-act="reboot" ${O.state.host ? '' : 'disabled'}>Reboot the Pi</button>${ui.rebooting ? `<span class="warn-text busy">${esc(ui.rebooting)}</span>` : ''}</div>
      ${p && p.ok && ui.afterReboot ? `<div class="result ok"><span class="ok-text">✓ The Pi is back.</span> Installed: ${p.installed ? 'yes' : 'no'} · start at boot: ${esc(p.service)}</div>` : ''}
      ${errBox('reboot')}</div>`;
  },

  imu: () => {
    const poses = O.state.imu_poses || {}, i2c = ui.i2c, ax = ui.axes, live = ui.live;
    const vec = (v) => (v ? `(${v.map((x) => x.toFixed(2)).join(', ')})` : '—');
    let liveHtml = '';
    if (live) {
      const g = live.body_grav, err = Math.hypot(g[0], g[1], g[2] + 1);
      liveHtml = `<div class="result ${err < 0.15 ? 'ok' : 'bad'}">Gravity in body frame ${vec(g)} with axes <code>${esc(live.axes)}</code>. ${err < 0.15 ? '<span class="ok-text">✓ Upright reads (0, 0, −1).</span>' : '<span class="warn-text">If the robot is upright, this should be close to (0, 0, −1). Capture the poses below.</span>'}</div>`;
    }
    return `<h2>IMU wiring and orientation</h2>
    <p class="lead">The walking AI needs to know which way is down and how fast the body turns. Wire the MPU6050 to the Pi (with the Pi switched off), then let the wizard work out how the board is mounted.</p>
    <div class="card"><h3>Wiring</h3><table><tr><th>MPU6050</th><th>Pi Zero pin</th><th></th></tr>
      <tr><td>VCC</td><td>1</td><td class="muted">3.3 V</td></tr><tr><td>GND</td><td>6</td><td></td></tr>
      <tr><td>SDA</td><td>3</td><td class="muted">GPIO2</td></tr><tr><td>SCL</td><td>5</td><td class="muted">GPIO3</td></tr>
      <tr><td>AD0</td><td>17</td><td class="muted">3.3 V: moves the IMU to address 0x69, away from the PiSugar clock at 0x68</td></tr>
      <tr><td>XDA, XCL, INT</td><td>—</td><td class="muted">not connected</td></tr></table>
      <div class="row"><button class="primary" data-act="i2c" ${O.state.host ? '' : 'disabled'}>Look for the IMU</button></div>
      ${i2c ? `<div class="result ${i2c.ok ? 'ok' : 'bad'}">${i2c.ok ? '<span class="ok-text">✓ MPU6050 found at 0x69.</span>' : '<span class="bad-text">✕ No IMU at 0x69.</span> ' + esc(i2c.hint)}<br>${i2c.devices.map((d) => `<code>${esc(d.address)}</code> ${esc(d.name)}`).join(' · ') || 'nothing on the I²C bus'}</div>` : ''}
      ${errBox('i2c')}</div>
    <div class="card"><h3>Mounting direction</h3>
      <p>Mount it rigidly on the robot's back, flat. The default setting expects components facing out, the Y arrow pointing up and the X arrow pointing to the robot's right. Whatever the mounting, these two captures find the right setting.</p>
      <div class="row"><button data-act="capture" data-pose="up" ${O.state.host ? '' : 'disabled'}>1. Capture: robot upright</button><span class="muted">${poses.up ? 'captured ' + vec(poses.up) : 'stand it up, hold still'}</span></div>
      <div class="row"><button data-act="capture" data-pose="nose_down" ${O.state.host ? '' : 'disabled'}>2. Capture: nose tilted down 30–45°</button><span class="muted">${poses.nose_down ? 'captured ' + vec(poses.nose_down) : 'tip it forward, hold still'}</span></div>
      <div class="row"><button class="primary" data-act="axes" ${poses.up && poses.nose_down ? '' : 'disabled'}>Work out the setting</button></div>
      ${ax ? `<div class="result ${ax.axes === ax.configured ? 'ok' : ''}">Mounting setting: <code>axes = "${esc(ax.axes)}"</code>. ${ax.axes === ax.configured ? '<span class="ok-text">✓ Already in config.toml.</span>' : `config.toml has <code>${esc(ax.configured)}</code>. <button class="primary" data-act="saveAxes" data-axes="${esc(ax.axes)}">Save and copy to the Pi</button>`}</div>` : ''}
      ${jobPanel('deploy')}${errBox('capture')}${errBox('axes')}</div>
    <div class="card"><h3>Check</h3><div class="row"><button data-act="live" ${O.state.host ? '' : 'disabled'}>Read the IMU now</button><span class="muted">uses the setting currently on the Pi</span></div>${liveHtml}${errBox('live')}
      <p class="muted">Changed the setting while the robot software is running? Restart it in the next step.</p></div>`;
  },

  run: () => {
    const rt = ui.runtime, host = O.state.host, hn = host ? host.split('@').pop() : '';
    let status = '<p class="muted">Waiting for the robot software…</p>';
    if (rt && rt.reachable) {
      const fhz = rt.feedback_hz || 0, ihz = rt.inference_hz || 0;
      status = `<div class="kv">
        <div><small>Mode</small><strong>${esc(rt.mode)}</strong></div>
        <div><small>Servos answering</small><strong class="${rt.servos === 15 ? 'ok-text' : 'bad-text'}">${rt.servos} / 15</strong></div>
        <div><small>IMU</small><strong class="${rt.imu_ok ? 'ok-text' : 'bad-text'}">${rt.imu_ok ? 'online' : 'offline'}</strong></div>
        <div><small>Feedback (target 100 Hz)</small><strong class="${fhz > 90 ? 'ok-text' : 'warn-text'}">${fhz.toFixed(1)} Hz</strong></div>
        <div><small>Policy (target 50 Hz)</small><strong class="${ihz > 45 ? 'ok-text' : 'warn-text'}">${ihz.toFixed(1)} Hz</strong></div>
        <div><small>Late servo ticks</small><strong>${rt.overruns ?? '—'}</strong></div></div>
        ${rt.missing && rt.missing.length ? `<p class="bad-text">Missing IDs: ${rt.missing.join(', ')}</p>` : ''}
        ${rt.pi && rt.pi.bus_error ? `<p class="bad-text">${esc(rt.pi.bus_error)}</p>` : ''}
        ${rt.last_error ? `<p class="warn-text">Last message: ${esc(rt.last_error)}</p>` : ''}
        <p class="muted">The feedback rate should sit close to 100 Hz with few late ticks. Note what you see here: the Pi Zero's timing is the one thing that couldn't be measured before.</p>`;
    } else if (rt && host) {
      status = `<p class="muted">Not answering at ${esc(rt.url || '')}. Start the software below; it takes about 20 seconds to load the AI models.</p>`;
    }
    return `<h2>Start the robot software</h2>
    <p class="lead">Now the URT-2 moves from this Mac to the Pi. The software starts in “inference only” mode, with torque off, so nothing moves until you choose a mode on the control page.</p>
    <div class="card"><ol class="howto">
      <li>Plug the URT-2 into the Pi's <b>USB</b> port (the inner one, not PWR) with the USB-C to micro-USB cable.</li>
      <li>Battery through the switch to the URT-2 terminal; switch on. The PiSugar powers the Pi.</li>
      <li>Support the robot so its legs hang free before you enable any mode.</li>
    </ol>
    <div class="row"><button data-act="usb" ${host ? '' : 'disabled'}>Is the URT-2 visible on the Pi?</button></div>
    ${ui.usb ? `<pre class="log">${esc(ui.usb.output || '(nothing found: check the cable is in the USB port, not PWR)')}</pre>` : ''}${errBox('usb')}</div>
    <div class="card"><h3>Robot software</h3>
      <div class="row"><button class="primary" data-act="svc" data-a="start" ${host ? '' : 'disabled'}>Start</button><button data-act="svc" data-a="restart" ${host ? '' : 'disabled'}>Restart</button><button data-act="svc" data-a="stop" ${host ? '' : 'disabled'}>Stop</button><button data-act="logs" ${host ? '' : 'disabled'}>Show log</button>
      ${host ? `<a class="button primary" href="http://${esc(hn)}:9527/" target="_blank" rel="noopener">Open the control page ↗</a>` : ''}</div>
      ${ui.svc ? `<div class="result ${ui.svc.ok ? 'ok' : 'bad'}">${esc(ui.svc.output || ui.svc.message)}</div>` : ''}${errBox('svc')}
      ${ui.logsOut ? `<pre class="log" id="logs-out">${esc(ui.logsOut)}</pre>` : ''}</div>
    <div class="card"><h3>Live status</h3>${status}</div>
    <div class="card"><h3>Servo scan on the Pi</h3><p class="muted">Stop the robot software first: it owns the URT-2 while it runs.</p>
      <div class="row"><button data-act="piscan" ${host ? '' : 'disabled'}>Scan servos from the Pi</button></div>${ui.piscan ? `<pre class="log">${esc(ui.piscan.output)}</pre>` : ''}${errBox('piscan')}</div>`;
  },

  calibrate: () => {
    const host = O.state.host, hn = host ? host.split('@').pop() : 'xgoduck.local';
    return `<h2>Calibrate and first moves</h2>
    <p class="lead">These use upstream's own pages on the Pi. Calibration tells the software where each joint's zero is. Then try the default pose, and only then walking.</p>
    <div class="card"><h3>Calibration</h3><ol class="howto">
      <li>Support the robot so no joint can hit anything.</li>
      <li>Open <a href="http://${esc(hn)}:9527/calibrate.html" target="_blank" rel="noopener">Calibration ↗</a> and start it. All servos move to their center at low stiffness.</li>
      <li>For each joint, use the slider until the part sits at its mechanical zero (as in the assembly photos), then press <b>Finish</b> for that joint.</li>
      <li>Exit. The zeros are saved on the Pi in <code>~/xgo-duck/upstream/data/zero_pos.json</code>. Copying files with this wizard never overwrites it.</li></ol>
      <div class="callout warn">Luwu's docs note that a per-joint photo set of the mechanical zero is still missing. Go by the assembly guide photos, and keep a copy of <code>zero_pos.json</code> once it works.</div></div>
    <div class="card"><h3>First moves</h3><ol class="howto">
      <li>Robot supported: on the <a href="http://${esc(hn)}:9527/" target="_blank" rel="noopener">control page ↗</a>, choose <b>Default pose</b>. Check every joint goes to the standing pose without straining.</li>
      <li>Watch the IMU line: gravity should read about (0, 0, −1) while it is upright.</li>
      <li>On a clear, flat floor, choose <b>Walk / get up</b> and push the left pad gently forward. <kbd>Space</kbd> turns torque off.</li>
      <li>Closing the page, or leaving it in the background for 1 second, also turns torque off.</li></ol></div>`;
  },
};

// ---- step entry hooks --------------------------------------------------------------
let runtimeTimer = null;
function onEnter(id) {
  if (id === 'urt' && !ui.ports) loadPorts();
  if (id === 'sd' && !ui.key) api('/api/sshkey').then((k) => { ui.key = k; render(); }).catch(() => {});
  clearInterval(runtimeTimer); runtimeTimer = null;
  if (id === 'run' && O.state.host) {
    const poll = () => api('/api/pi/runtime').then((r) => {
      const changed = JSON.stringify(r) !== JSON.stringify(ui.runtime);
      ui.runtime = r;
      if (changed && STEPS[current].id === 'run' && !document.querySelector('.busy')) render();
    }).catch(() => {});
    if (!ui.runtime) poll();
    runtimeTimer = setInterval(poll, 2000);
  }
}
async function loadPorts() { try { ui.ports = (await api('/api/ports')).ports; } catch (e) { ui.ports = []; } render(); }

async function waitForPi() {
  ui.afterReboot = false;
  const until = Date.now() + 240000;
  await new Promise((r) => setTimeout(r, 15000));
  while (Date.now() < until) {
    ui.rebooting = 'waiting for the Pi to come back…'; render();
    const p = await api('/api/pi/probe', {}).catch(() => null);
    if (p && p.ok) { ui.probe = p; ui.rebooting = ''; ui.afterReboot = true; render(); return; }
    await new Promise((r) => setTimeout(r, 5000));
  }
  ui.rebooting = ''; ui.err = ui.err || {}; ui.err.reboot = 'The Pi did not come back within 4 minutes. Check its power and Wi-Fi.'; render();
}

// ---- actions ------------------------------------------------------------------
const ACTIONS = {
  go: (b) => { current = +b.dataset.i; try { localStorage.setItem('xgo-step', STEPS[current].id); } catch (e) { /* ignore */ } render(); $('#main').focus(); window.scrollTo(0, 0); },
  toggleDone: (b) => act(b, 'done', async () => {
    const id = STEPS[current].id, done = new Set(O.state.steps_done);
    done.has(id) ? done.delete(id) : done.add(id);
    await saveState({ steps_done: [...done] });
  }),
  check: (b) => act(null, 'check', () => saveState({ checklist: { ...O.state.checklist, [b.dataset.k]: b.checked } })),
  simStart: (b) => act(b, 'sim', async () => { const s = await api('/api/sim/start', {}); setTimeout(() => window.open(s.url, '_blank', 'noopener'), 2500); toast('Starting the simulated robot; its control page opens in a moment'); }),
  simStop: (b) => act(b, 'sim', () => api('/api/sim/stop', {})),
  ports: () => loadPorts(),
  connect: (b) => act(b, 'connect', async () => { const r = await api('/api/bench/connect', { device: $('#port').value }); ui.scan = null; ui.detect = null; toast(`Connected · echo ${r.echo ? 'on' : 'off'}`); }),
  disconnect: (b) => act(b, 'connect', async () => { await api('/api/bench/disconnect', {}); ui.scan = null; ui.detect = null; ui.chain = null; }),
  scan: (b) => act(b, 'scan', async () => { ui.scan = await api('/api/bench/scan', {}); }),
  pick: (b) => { ui.target = +b.dataset.id; ui.setupResult = null; render(); },
  detect: (b) => act(b, 'ids', async () => { ui.setupResult = null; ui.detect = await api('/api/bench/scan', {}); }),
  setup: (b) => act(b, 'ids', async () => {
    const from = ui.detect.ids[0], to = ui.target;
    ui.setupResult = await api('/api/bench/setup', { old_id: from, new_id: to });
    ui.detect = null;
    await saveState({ servos_done: [...new Set([...O.state.servos_done, to])] });
    toast(`Done: label this servo “${to}”`);
  }),
  simNew: (b) => act(b, 'ids', async () => { await api('/api/bench/sim_new_servo', {}); ui.detect = null; ui.setupResult = null; toast('Simulator: a new servo (ID 1) is on the bus'); }),
  undoServo: (b) => act(b, 'ids', () => saveState({ servos_done: O.state.servos_done.filter((i) => i !== ui.target) })),
  wiggle: (b) => act(b, current === 3 ? 'chain' : 'ids', async () => { await api('/api/bench/wiggle', { id: +b.dataset.id }); toast(`ID ${b.dataset.id} (${O.joints[b.dataset.id]}) moved and came back`); }),
  chain: (b) => act(b, 'chain', async () => { ui.chain = await api('/api/bench/scan', {}); }),
  copyKey: () => navigator.clipboard.writeText(ui.key.key).then(() => toast('Key copied: paste it into Raspberry Pi Imager'), () => toast('Select the key text and copy it', true)),
  makeKey: (b) => act(b, 'key', async () => { ui.key = await api('/api/sshkey/create', {}); }),
  saveHost: (b) => act(b, 'probe', async () => { await saveState({ host: $('#host').value.trim() }); ui.probe = null; ui.runtime = null; toast('Saved'); }),
  probe: (b) => act(b, 'probe', async () => { if ($('#host').value.trim() !== O.state.host) await saveState({ host: $('#host').value.trim() }); ui.probe = await api('/api/pi/probe', {}); }),
  deploy: (b) => act(b, 'deploy', async () => { watchJob('deploy', await api('/api/pi/deploy', {})); }),
  install: (b) => act(b, 'install', async () => { watchJob('install', await api('/api/pi/install', {})); }),
  reboot: (b) => {
    if (!confirm('Reboot the Pi now?')) return;
    act(b, 'reboot', async () => { await api('/api/pi/reboot', {}); waitForPi(); });
  },
  i2c: (b) => act(b, 'i2c', async () => {
    const r = await api('/api/pi/i2c', {});
    if (!r.ok) throw new Error(r.message || r.output);
    ui.i2c = r;
  }),
  capture: (b) => act(b, 'capture', async () => { await api('/api/pi/imu_capture', { pose: b.dataset.pose }); ui.axes = null; toast(b.dataset.pose === 'up' ? 'Upright pose captured' : 'Nose-down pose captured'); }),
  axes: (b) => act(b, 'axes', async () => { ui.axes = await api('/api/pi/imu_axes', {}); }),
  saveAxes: (b) => act(b, 'axes', async () => {
    await api('/api/pi/save_axes', { axes: b.dataset.axes });
    ui.axes = await api('/api/pi/imu_axes', {});
    watchJob('deploy', await api('/api/pi/deploy', {}));
    toast('Saved in config.toml; copying to the Pi');
  }),
  live: (b) => act(b, 'live', async () => { ui.live = await api('/api/pi/imu_live', {}); }),
  usb: (b) => act(b, 'usb', async () => { ui.usb = await api('/api/pi/usb', {}); }),
  svc: (b) => act(b, 'svc', async () => { ui.svc = await api('/api/pi/service', { action: b.dataset.a }); ui.runtime = null; }),
  logs: (b) => act(b, 'svc', async () => { const r = await api('/api/pi/logs', {}); ui.logsOut = r.output || r.message; }),
  piscan: (b) => act(b, 'piscan', async () => { ui.piscan = await api('/api/pi/bus_scan', {}); }),
};

document.addEventListener('click', (e) => {
  const b = e.target.closest('[data-act]');
  if (!b || b.tagName === 'A') return;
  if (b.type === 'checkbox') { ACTIONS.check(b); return; }
  e.preventDefault();
  ACTIONS[b.dataset.act](b);
});
document.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && e.target.id === 'host') ACTIONS.probe($('[data-act="probe"]'));
});

(async function start() {
  try { await refresh(); render(); } catch (e) { $('#main').innerHTML = `<div class="result bad">Cannot reach the wizard server: ${esc(e.message)}</div>`; }
  setInterval(() => { if (!document.querySelector('.busy')) refresh().then(() => { $('#chips').innerHTML = chips(); }).catch(() => {}); }, 5000);
})();
