import { simulate, DEFAULTS } from './sim.js';
import { plot } from './plot.js';

const D = 180 / Math.PI;
const $ = id => document.getElementById(id);
const ctrl = await (await fetch('data/controller.json')).json();

const inputs = {
  elevStep: [$('elevStep'), $('o-elev'), v => `${v}°`],
  travStep: [$('travStep'), $('o-trav'), v => `${v}°`],
  noise: [$('noise'), $('o-noise'), v => `${v} mrad`],
  thrust: [$('thrust'), $('o-thrust'), v => `${v}%`],
  tau: [$('tau'), $('o-tau'), v => `${v} ms`],
  dist: [$('dist'), $('o-dist'), v => `${v} mN·m`],
};
const defaults = Object.fromEntries(Object.entries(inputs).map(([k, [el]]) => [k, el.value]));

let run, playing = true, tNow = 0, last = performance.now();

function recompute() {
  const v = Object.fromEntries(Object.entries(inputs).map(([k, [el, out, fmt]]) => { out.textContent = fmt(el.value); return [k, +el.value]; }));
  run = simulate(ctrl, {
    elevRef: [[0, 0], [30, v.elevStep]],
    travelRef: [[0, 0], [10, v.travStep], [40, 0]],
    noise: v.noise / 1000,
    thrustScale: v.thrust / 100,
    fanTau: v.tau / 1000,
    distTorque: v.dist / 1000,
  });
  draw();
}

function idx() { return Math.min(run.t.length - 1, Math.floor(tNow / ctrl.params.T)); }

function drawPlots() {
  const deg = a => a.map(v => v * D);
  const c = tNow;
  plot($('p-elev'), run.t, [{ y: deg(run.elevRef), color: '--ref' }, { y: deg(run.elev), color: '--sig' }], { cursor: c, ylabel: 'deg' });
  plot($('p-trav'), run.t, [{ y: deg(run.travelRef), color: '--ref' }, { y: deg(run.travel), color: '--sig' }], { cursor: c, ylabel: 'deg' });
  plot($('p-pitch'), run.t, [{ y: deg(run.pitch), color: '--sig' }], { cursor: c, ylabel: 'deg' });
  plot($('p-volt'), run.t, [{ y: run.va, color: '--ref' }, { y: run.vb, color: '--sig' }], { cursor: c, ylabel: 'V' });
}

function drawAnim() {
  const cv = $('anim'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height;
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const i = idx(), E = run.elev[i], P = run.pitch[i], T = run.travel[i];
  ctx.clearRect(0, 0, W, H);
  ctx.lineWidth = 3; ctx.lineCap = 'round';
  // Side view (elevation): pivot on the left, arm length to fan assembly.
  const px = 170, py = 160, L = 110, S = 1.0;
  ctx.fillStyle = css('--muted'); ctx.font = '12px system-ui'; ctx.fillText('Side view (elevation, pitch)', 12, 20);
  ctx.strokeStyle = css('--line'); ctx.beginPath(); ctx.moveTo(px - 140, py + 60); ctx.lineTo(px + 140, py + 60); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py + 60); ctx.stroke();
  const hx = px + L * 1.2 * Math.cos(E), hy = py - L * 1.2 * Math.sin(E);
  const cx = px - L * 0.9 * Math.cos(E), cy = py + L * 0.9 * Math.sin(E);
  ctx.strokeStyle = css('--fg'); ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy); ctx.stroke();
  ctx.fillStyle = css('--muted'); ctx.beginPath(); ctx.arc(cx, cy, 9, 0, 7); ctx.fill();
  // Fan bar (pitch exaggerated 4x so small angles are visible)
  const pa = P * 4, bx = 22 * Math.cos(pa), by = 22 * Math.sin(pa);
  ctx.strokeStyle = css('--sig'); ctx.beginPath(); ctx.moveTo(hx - bx, hy - by); ctx.lineTo(hx + bx, hy + by); ctx.stroke();
  // Top view (travel)
  const tx = 480, ty = 160, R = 100;
  ctx.fillStyle = css('--muted'); ctx.fillText('Top view (travel)', 380, 20);
  ctx.strokeStyle = css('--line'); ctx.beginPath(); ctx.arc(tx, ty, R, 0, 7); ctx.stroke();
  const ax = tx + R * Math.cos(E) * Math.sin(T), ay = ty - R * Math.cos(E) * Math.cos(T);
  ctx.strokeStyle = css('--fg'); ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(ax, ay); ctx.stroke();
  ctx.fillStyle = css('--sig'); ctx.beginPath(); ctx.arc(ax, ay, 8, 0, 7); ctx.fill();
  ctx.fillStyle = css('--muted'); ctx.fillText('0°', tx - 8, ty - R - 6);
  $('clock').textContent = `${tNow.toFixed(1)} s`;
  $('scrub').value = tNow / run.t[run.t.length - 1];
}

function draw() { drawAnim(); drawPlots(); }

function tick(now) {
  const dt = (now - last) / 1000; last = now;
  if (playing) {
    tNow += dt * 2;                                  // 2x real time
    if (tNow >= run.t[run.t.length - 1]) tNow = 0;
    draw();
  }
  requestAnimationFrame(tick);
}

for (const [el] of Object.values(inputs)) el.addEventListener('input', recompute);
$('reset').onclick = () => { for (const [k, [el]] of Object.entries(inputs)) el.value = defaults[k]; recompute(); };
$('play').onclick = () => { playing = !playing; $('play').textContent = playing ? 'Pause' : 'Play'; };
$('scrub').addEventListener('input', e => { playing = false; $('play').textContent = 'Play'; tNow = +e.target.value * run.t[run.t.length - 1]; draw(); });

recompute();
requestAnimationFrame(tick);
