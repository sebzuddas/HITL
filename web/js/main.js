import { simulate } from './sim.js';
import { designLQR } from './design.js';
import { plot } from './plot.js';
import { createView } from './view3d.js';

const D = 180 / Math.PI;
const $ = id => document.getElementById(id);
const ctrl = await (await fetch('data/controller.json')).json();

// ---- scenario sliders ----
const inputs = {
  elevStep: [$('elevStep'), $('o-elev'), v => `${v}°`],
  travStep: [$('travStep'), $('o-trav'), v => `${v}°`],
  noise: [$('noise'), $('o-noise'), v => `${v} mrad`],
  thrust: [$('thrust'), $('o-thrust'), v => `${v}%`],
  tau: [$('tau'), $('o-tau'), v => `${v} ms`],
  dist: [$('dist'), $('o-dist'), v => `${v} mN·m`],
};
const defaults = Object.fromEntries(Object.entries(inputs).map(([k, [el]]) => [k, el.value]));

// ---- LQR weight sliders (log10 scale) ----
// Diagonal weights found by fitting lqrd() on the linearised model to the gains that flew on the rig.
const FITTED = { qx: [850, 22000, 670, 1100, 190, 22000, 190, 150], qu: [1, 1] };
const Q_LABELS = ['elevation', 'pitch', 'travel', 'elevation rate', 'pitch rate', 'travel rate', '∫ elevation error', '∫ travel error'];
const R_LABELS = ['fan A voltage (R)', 'fan B voltage (R)'];
const weightEls = [];
function addSlider(parent, label, value) {
  const wrap = document.createElement('label');
  const out = document.createElement('output');
  const input = Object.assign(document.createElement('input'), { type: 'range', min: -2, max: 6, step: 0.1 });
  wrap.append(label, out, input); parent.append(wrap);
  const set = v => { input.value = Math.log10(v); out.textContent = (10 ** input.value).toPrecision(3); };
  set(value);
  input.addEventListener('input', () => { out.textContent = (10 ** input.value).toPrecision(3); $('mode').value = 'custom'; recompute(); });
  weightEls.push({ input, set });
}
Q_LABELS.forEach((l, i) => addSlider($('qsliders'), `Q: ${l}`, FITTED.qx[i]));
R_LABELS.forEach((l, i) => addSlider($('rsliders'), l, FITTED.qu[i]));
const weights = () => { const v = weightEls.map(w => 10 ** w.input.value); return { qx: v.slice(0, 8), qu: v.slice(8) }; };

// ---- state ----
let run, playing = true, tNow = 0, last = performance.now();
let view;
try { view = createView($('view3d')); }
catch (e) {   // no WebGL: keep the rest of the page working
  console.warn('3D view unavailable:', e.message);
  $('view3d').replaceWith(Object.assign(document.createElement('p'), { className: 'hint', textContent: '3D view needs WebGL, which is unavailable in this browser. Plots and controls still work.' }));
  view = { update() {} };
}

function recompute() {
  const v = Object.fromEntries(Object.entries(inputs).map(([k, [el, out, fmt]]) => { out.textContent = fmt(el.value); return [k, +el.value]; }));
  const plant = { thrustScale: v.thrust / 100, fanTau: v.tau / 1000 };
  let c = ctrl, err = '';
  if ($('mode').value === 'custom') {
    try { c = { ...ctrl, ...designLQR(ctrl, weights(), plant) }; } catch (e) { err = `LQR design failed: ${e.message}. Using original gains.`; }
  }
  $('designError').hidden = !err; $('designError').textContent = err;
  const f = m => m.map(r => r.map(x => x.toFixed(2).padStart(8)).join('')).join('\n');
  $('gains').textContent = `Kd (state feedback, 2x6)\n${f(c.Kd)}\n\nKdi (integral gain, 2x2)\n${f(c.Kdi)}`;

  run = simulate(c, {
    ...plant,
    elevRef: [[0, 0], [30, v.elevStep]],
    travelRef: [[0, 0], [10, v.travStep], [40, 0]],
    noise: v.noise / 1000,
    distTorque: v.dist / 1000,
  });
  draw(0);
}

const idx = () => Math.min(run.t.length - 1, Math.floor(tNow / ctrl.params.T));

function drawPlots() {
  const deg = a => a.map(v => v * D);
  const c = tNow;
  plot($('p-elev'), run.t, [{ y: deg(run.elevRef), color: '--ref' }, { y: deg(run.elev), color: '--sig' }], { cursor: c, ylabel: 'deg' });
  plot($('p-trav'), run.t, [{ y: deg(run.travelRef), color: '--ref' }, { y: deg(run.travel), color: '--sig' }], { cursor: c, ylabel: 'deg' });
  plot($('p-pitch'), run.t, [{ y: deg(run.pitch), color: '--sig' }], { cursor: c, ylabel: 'deg' });
  plot($('p-volt'), run.t, [{ y: run.va, color: '--ref' }, { y: run.vb, color: '--sig' }], { cursor: c, ylabel: 'V' });
}

function draw(dt) {
  const i = idx();
  view.update({ E: run.elev[i], P: run.pitch[i], T: run.travel[i], va: run.va[i], vb: run.vb[i] }, dt);
  $('clock').textContent = `${tNow.toFixed(1)} s`;
  $('scrub').value = tNow / run.t[run.t.length - 1];
  drawPlots();
}

function tick(now) {
  const dt = (now - last) / 1000; last = now;
  if (playing) {
    tNow += dt * 2;                                  // 2x real time
    if (tNow >= run.t[run.t.length - 1]) tNow = 0;
    draw(dt);
  } else {
    view.update({ E: run.elev[idx()], P: run.pitch[idx()], T: run.travel[idx()], va: run.va[idx()], vb: run.vb[idx()] }, 0);
  }
  requestAnimationFrame(tick);
}

for (const [el] of Object.values(inputs)) el.addEventListener('input', recompute);
$('mode').addEventListener('change', recompute);
$('fitPreset').onclick = () => { [...FITTED.qx, ...FITTED.qu].forEach((v, i) => weightEls[i].set(v)); $('mode').value = 'custom'; recompute(); };
$('reset').onclick = () => { for (const [k, [el]] of Object.entries(inputs)) el.value = defaults[k]; $('mode').value = 'original'; recompute(); };
$('play').onclick = () => { playing = !playing; $('play').textContent = playing ? 'Pause' : 'Play'; };
$('scrub').addEventListener('input', e => { playing = false; $('play').textContent = 'Play'; tNow = +e.target.value * run.t[run.t.length - 1]; draw(0); });

recompute();
requestAnimationFrame(tick);
