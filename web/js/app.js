// Lógica de la interfaz: WebSocket con el backend C++ (WebServer.cpp) y
// enlace de los controles. Replica el comportamiento de MainWindow.cpp.

import { Spider3D } from './spider3d.js';

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];

const MOTOR_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43];
const LEG_NAMES = ['DD', 'DI', 'TI', 'TD'];

// ======================================================================
// Conexión
// ======================================================================
let ws = null;
let seq = 0;
const pending = new Map();   // seq -> callback(result)

function send(msg, onResult) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return false;
  if (onResult) { msg.seq = ++seq; pending.set(msg.seq, onResult); }
  ws.send(JSON.stringify(msg));
  return true;
}

function connect() {
  setWs('wait', 'Conectando…');
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { setWs('on', 'Backend'); document.body.classList.remove('offline'); };
  ws.onclose = () => {
    setWs('', 'Sin backend');
    document.body.classList.add('offline');
    pending.clear();
    releaseAllKeys(false);
    setTimeout(connect, 1500);
  };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === 'state') onState(m);
    else if (m.type === 'hello') { onHello(m); setControl(m.control); }
    else if (m.type === 'role') setControl(m.control);
    else if (m.type === 'poseReset') onPoseReset(m);
    else if (m.type === 'result') {
      const cb = pending.get(m.seq); pending.delete(m.seq);
      if (cb) cb(m);
      else if (!m.ok && m.msg) toast(m.msg);
    }
  };
}
function setWs(cls, text) {
  const p = $('#pillWs'); p.className = 'pill ' + cls; $('#wsText').textContent = text;
}

// Sólo la pestaña más reciente manda; las demás quedan en solo lectura
let hasControl = true;
function setControl(control) {
  hasControl = control !== false;
  document.body.classList.toggle('readonly', !hasControl);
  if (hasControl) { setWs('on', 'Backend'); $('#pillWs').title = ''; }
  else {
    releaseAllKeys(false);
    setWs('wait', 'Solo lectura · tomar control');
    $('#pillWs').title = 'Otra pestaña tiene el control. Pulsa para tomarlo.';
    toast('Otra pestaña ha tomado el control. Esta queda en solo lectura.', 'info');
  }
}
$('#pillWs').addEventListener('click', () => {
  if (!hasControl) send({ cmd: 'takeControl' }, r => r.ok && setControl(true));
});

// ======================================================================
// Visor 3D
// ======================================================================
const spider = new Spider3D($('#viewport'));
spider.ready.then(() => $('#vpLoading').classList.add('hidden'))
  .catch(e => { $('#vpLoading').textContent = 'No se pudo cargar el modelo 3D: ' + e.message; console.error(e); });
spider.onLegClick = (leg) => { $('#ikPanel').open = true; selectLeg(leg); };
window.spider = spider; // acceso desde la consola del navegador (depuración / calibración)
$('#btnView').onclick = () => spider.resetView();

let angleSource = 'cmd';
segmented($('#segSource'), b => { angleSource = b.dataset.src; });

// ======================================================================
// Estado entrante
// ======================================================================
function onHello(m) {
  fillPorts(m.ports || [], m.port);
  setGaitType(m.gaitType ?? 0, false);
  if (m.body) syncBodySliders(m.body);
  requestFootSync();
}

let lastState = null;
function onState(s) {
  lastState = s;
  $('#simBadge').hidden = !s.sim;
  pillOn('#pillUsb', s.usb);
  pillOn('#pillCan', s.can);

  const gs = $('#gaitState');
  const prep = !s.gait.active && s.gait.pending;
  gs.textContent = s.gait.active ? 'en marcha' : prep ? 'volviendo a reposo' : 'parada';
  gs.classList.toggle('live', s.gait.active);
  gs.classList.toggle('prep', prep);
  if (document.activeElement?.closest('#segGait') == null) setGaitType(s.gait.type, false);

  // Ángulos para el 3D
  const byId = Object.fromEntries(s.motors.map(m => [m.id, m]));
  const angles = [1, 2, 3, 4].map(l => [1, 2, 3].map(j => {
    const m = byId[l * 10 + j];
    return m ? (angleSource === 'cmd' ? m.cmd : m.pos) : 0;
  }));
  spider.setJointAngles(angles);
  spider.setBodyPose(s.body);
  spider.setGrounded(s.grounded);
  renderMotors(s);

  if (s.port && $('#selPort').value !== s.port && document.activeElement !== $('#selPort'))
    fillPorts(s.ports || [...$('#selPort').options].map(o => o.value), s.port);
}

function pillOn(sel, on) { $(sel).classList.toggle('on', !!on); }

// ======================================================================
// Tabla de motores (4 tarjetas, una por pata)
// ======================================================================
function stiffLabel(m) {
  if (!m.online) return m.active ? 'sim' : 'desc.';  // 'sim' = simulado (temporal)
  if (!m.active) return 'OFF';
  const kp = m.kp;
  if (kp <= 0) return 'suave';
  if (kp <= 5) return 'P1';
  if (kp <= 10) return 'P2';
  if (kp <= 20) return 'P3';
  if (kp <= 35) return 'P4';
  return 'P5';
}
function buildMotorCards() {
  const host = $('#motors');
  host.innerHTML = [1, 2, 3, 4].map(l => `
    <div class="leg-card" data-leg="${l}">
      <header><span>PATA ${l} · ${LEG_NAMES[l - 1]}</span><span class="gnd">—</span></header>
      ${[1, 2, 3].map(j => `
        <div class="mrow" data-id="${l * 10 + j}">
          <i></i><span class="id">${l * 10 + j}</span>
          <span class="val"><b>—</b><em>desc.</em></span>
        </div>`).join('')}
    </div>`).join('');
  $$('.leg-card', host).forEach(c => c.onclick = () => selectLeg(+c.dataset.leg));
}
function renderMotors(s) {
  for (const m of s.motors) {
    const row = $(`.mrow[data-id="${m.id}"]`);
    if (!row) continue;
    row.classList.toggle('online', m.online);
    row.classList.toggle('active', m.active);
    $('b', row).textContent = (m.online || m.active) ?`${(angleSource === 'cmd' && m.active ? m.cmd : m.pos).toFixed(1)}°` : '—';
    $('em', row).textContent = stiffLabel(m);
  }
  $$('.leg-card').forEach((c, i) => {
    const g = $('.gnd', c);
    const has = i === 0; // sólo la pata 1 tiene sensor Hall por ahora
    g.textContent = has ? (s.grounded[i] ? 'APOYADA' : 'EN AIRE') : '';
    g.classList.toggle('on', !!s.grounded[i]);
  });
}

// ======================================================================
// Puerto COM
// ======================================================================
function fillPorts(ports, current) {
  const sel = $('#selPort');
  const list = [...new Set([...(ports || []), current].filter(Boolean))]
    .sort((a, b) => parseInt(a.replace(/\D/g, '')) - parseInt(b.replace(/\D/g, '')));
  sel.innerHTML = list.map(p => `<option ${p === current ? 'selected' : ''}>${p}</option>`).join('');
}
$('#selPort').onchange = (e) => {
  send({ cmd: 'setPort', port: e.target.value }, r => toast(r.ok ? `Conectado a ${e.target.value}` : (r.msg || `No se pudo abrir ${e.target.value}`), r.ok ? 'info' : 'bad'));
};
$('#selPort').onfocus = () => send({ cmd: 'listPorts' }, r => r.ports && fillPorts(r.ports, $('#selPort').value));

// ======================================================================
// Botones simples
// ======================================================================
$$('[data-cmd]').forEach(b => b.onclick = () => send({ cmd: b.dataset.cmd }));

// ======================================================================
// Marcha
// ======================================================================
function setGaitType(type, emit = true) {
  $$('#segGait button').forEach(b => b.setAttribute('aria-checked', String(+b.dataset.gait === type)));
  if (emit) send({ cmd: 'gaitType', type });
}
segmented($('#segGait'), b => setGaitType(+b.dataset.gait));

const keys = { up: false, down: false, left: false, right: false };
function pushKeys() {
  $$('#dpad button').forEach(b => b.classList.toggle('down', keys[b.dataset.key]));
  send({ cmd: 'gaitKeys', ...keys });
}
function setKey(k, v) { if (keys[k] !== v) { keys[k] = v; pushKeys(); } }
function releaseAllKeys(emit = true) {
  for (const k in keys) keys[k] = false;
  $$('#dpad button').forEach(b => b.classList.remove('down'));
  if (emit) send({ cmd: 'gaitKeys', ...keys });
}
$$('#dpad button').forEach(b => {
  const k = b.dataset.key;
  b.addEventListener('pointerdown', e => { b.setPointerCapture(e.pointerId); setKey(k, true); });
  b.addEventListener('pointerup', () => setKey(k, false));
  b.addEventListener('pointercancel', () => setKey(k, false));
  b.addEventListener('lostpointercapture', () => setKey(k, false));
});
const ARROW = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right' };
const typing = () => /^(INPUT|SELECT|TEXTAREA)$/.test(document.activeElement?.tagName) && document.activeElement.type !== 'range';
window.addEventListener('keydown', e => {
  if (!ARROW[e.key] || typing()) return;
  e.preventDefault();
  if (!e.repeat) setKey(ARROW[e.key], true);
});
window.addEventListener('keyup', e => { if (ARROW[e.key]) { e.preventDefault(); setKey(ARROW[e.key], false); } });
// Seguridad: si la ventana pierde el foco, soltar todas las teclas
window.addEventListener('blur', () => releaseAllKeys());
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAllKeys(); });

// ======================================================================
// Sliders (helpers)
// ======================================================================
function paintRange(r) {
  const p = (r.value - r.min) / (r.max - r.min) * 100;
  r.style.setProperty('--p', p + '%');
}
function setRange(r, v) { r.value = v; paintRange(r); r.nextElementSibling.value = fmt(r, v); }
function fmt(r, v) { return ['roll', 'pitch', 'yaw'].includes(r.dataset.k) ? `${v}°` : `${v} mm`; }
$$('input[type=range]').forEach(r => {
  paintRange(r);
  r.addEventListener('input', () => { paintRange(r); r.nextElementSibling.value = fmt(r, r.value); });
});

// Envía como mucho cada `ms`, garantizando que el último valor siempre se manda.
// Cada envío espera su respuesta antes de mandar el siguiente (como el Qt, que era síncrono).
function throttledSender(build, onResult, ms = 50) {
  let busy = false, dirty = false, timer = null;
  const fire = () => {
    if (busy) { dirty = true; return; }
    busy = true; dirty = false;
    const ok = send(build(), r => {
      onResult(r);
      setTimeout(() => { busy = false; if (dirty) fire(); }, ms);
    });
    if (!ok) busy = false;
  };
  return () => { clearTimeout(timer); timer = setTimeout(fire, 0); };
}

function flashBad(container) {
  container.classList.add('flash-bad');
  setTimeout(() => container.classList.remove('flash-bad'), 600);
}

// ======================================================================
// Postura del cuerpo
// ======================================================================
const bodyR = Object.fromEntries($$('#bodySliders input').map(r => [r.dataset.k, r]));
let lastValidBody = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
const readBody = () => Object.fromEntries(Object.entries(bodyR).map(([k, r]) => [k, +r.value]));
function syncBodySliders(b) { for (const k in bodyR) setRange(bodyR[k], Math.round(b[k] ?? 0)); lastValidBody = readBody(); }

const sendBody = throttledSender(
  () => ({ cmd: 'bodyPose', ...readBody() }),
  r => {
    if (r.ok) { lastValidBody = { ...r.pose }; return; }
    for (const k in bodyR) setRange(bodyR[k], lastValidBody[k]);
    flashBad($('#bodySliders'));
    toast('La postura solicitada no es factible o causaría una colisión.');
  });
Object.values(bodyR).forEach(r => r.addEventListener('input', sendBody));

$('#btnResetPose').onclick = () => {
  onPoseReset({});
  send({ cmd: 'resetPose' });
};
// Reset de pose (botón, o automático antes de arrancar la marcha): cuerpo a cero e IK de pata a nominal
function onPoseReset(m) {
  syncBodySliders(m.body || {});
  setRange(legR.x, 495); setRange(legR.y, 0); setRange(legR.z, 0);
}

// ======================================================================
// Control IK de pata
// ======================================================================
const legR = Object.fromEntries($$('#legSliders input').map(r => [r.dataset.k, r]));
let currentLeg = 1;
const readLeg = () => Object.fromEntries(Object.entries(legR).map(([k, r]) => [k, +r.value]));

function selectLeg(leg) {
  currentLeg = leg;
  $$('#segLeg button').forEach(b => b.setAttribute('aria-checked', String(+b.dataset.leg === leg)));
  $$('.leg-card').forEach(c => c.classList.toggle('sel', +c.dataset.leg === leg));
  spider.ready.then(() => spider.setHighlight(leg));
  requestFootSync();
}
segmented($('#segLeg'), b => selectLeg(+b.dataset.leg));

function requestFootSync() {
  send({ cmd: 'footPos', leg: currentLeg }, r => {
    if (!r.ok) return;
    let { x, y, z } = r;
    // misma regla que MainWindow::resetIndividualLegSliders
    if (x < 150 || x > 500 || y < -300 || y > 300 || z < -350 || z > 150) { x = 495; y = 0; z = 0; }
    setRange(legR.x, x); setRange(legR.y, y); setRange(legR.z, z);
  });
}

const sendLeg = throttledSender(
  () => ({ cmd: 'legIK', leg: currentLeg, ...readLeg() }),
  r => {
    if (r.ok) return;
    requestFootSync();
    flashBad($('#legSliders'));
    toast('El movimiento de la pata no es factible o provocaría una colisión.');
  });
Object.values(legR).forEach(r => r.addEventListener('input', sendLeg));

$('#btnLegNominal').onclick = () => {
  setRange(legR.x, 495); setRange(legR.y, 0); setRange(legR.z, 0);
  sendLeg();
};

// ======================================================================
// Articulaciones (las 4 patas) y motor individual
// ======================================================================
$('#btnSendGrid').onclick = () => send({
  cmd: 'allJoints',
  pos: [1, 2, 3].map(i => +$('#jp' + i).value || 0),
  rig: [1, 2, 3].map(i => Math.trunc(+$('#jr' + i).value || 0)),
});

$('#mvId').innerHTML = MOTOR_IDS.map(id => `<option value="${id}" title="Pata ${Math.floor(id / 10)}, articulación q${id % 10}">${id}</option>`).join('');
$('#btnMvEnable').onclick = () => send({ cmd: 'enableMotor', id: +$('#mvId').value });
$('#btnMvDisable').onclick = () => send({ cmd: 'disableMotor', id: +$('#mvId').value });
$('#btnResetLog').onclick = (e) => {
  e.stopPropagation();
  send({ cmd: 'resetLog' }, r => toast(r.ok ? 'Log CAN reseteado (can_log.csv)' : 'No se pudo resetear el log', r.ok ? 'info' : 'bad'));
};
$('#btnMvSend').onclick = () => {
  const pos = parseFloat($('#mvPos').value), rig = parseInt($('#mvRig').value, 10);
  if (!Number.isFinite(pos) || !Number.isFinite(rig)) return toast('Posición o rigidez no válidas');
  send({ cmd: 'moveMotor', id: +$('#mvId').value, pos, rig });
};

// ======================================================================
// Utilidades
// ======================================================================
function segmented(el, onPick) {
  el.addEventListener('click', e => {
    const b = e.target.closest('button'); if (!b) return;
    $$('button', el).forEach(x => x.setAttribute('aria-checked', String(x === b)));
    onPick(b);
  });
}

function toast(text, kind = 'bad') {
  const t = document.createElement('div');
  t.className = 'toast ' + kind;
  t.textContent = text;
  $('#toasts').append(t);
  while ($('#toasts').children.length > 3) $('#toasts').firstChild.remove();
  setTimeout(() => { t.classList.add('out'); setTimeout(() => t.remove(), 300); }, 3200);
}

buildMotorCards();
selectLeg(1);
document.body.classList.add('offline');
connect();
