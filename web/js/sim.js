// Browser port of the HITL helicopter loop: nonlinear 3-DOF plant (Simulink
// "Helicopter Continuous Time Dynamics") + discrete Kalman filter + LQR with
// integral action, using the gains exported from MATLAB (data/ -> controller.json).
// Pure ES module, no DOM, so it can be run in Node for testing.

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

function interp(xs, ys, x) {
  if (x <= xs[0]) return ys[0];
  if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
  let i = 1;
  while (xs[i] < x) i++;
  const t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

const matVec = (M, v) => M.map(r => r.reduce((s, m, j) => s + m * v[j], 0));
const add = (a, b) => a.map((x, i) => x + b[i]);

export const DEFAULTS = {
  duration: 60,        // s
  elevRef: [[0, 0], [30, 10]],       // [time s, degrees] step schedule
  travelRef: [[0, 0], [10, 180], [40, 0]],
  fanTau: 0.1,         // s, first-order fan lag (Simulink: tau)
  pitchK: 0.01,        // N m/rad pitch spring (tuned; the heli.m placeholder of 4 makes pitch near-rigid)
  pitchD: 0.002,       // N m s/rad pitch damping (tuned)
  elevDamping: 0.002,  // N m s/rad (Simulink: k_e)
  noise: 0,            // encoder noise std-dev (rad)
  thrustScale: 1,      // multiplies fan thrust (e.g. weaker/stronger fans)
  distTorque: 0,       // N m constant elevation disturbance
};

function schedule(points, t) {
  let v = points[0][1];
  for (const [ts, val] of points) if (t >= ts) v = val;
  return v * Math.PI / 180;
}

/** Plant derivatives. x = [E, P, T, Ed, Pd, Td]; F = [Fa, Fb]. */
function plantDeriv(x, F, p, o) {
  const [E, P, , Ed, Pd, Td] = x;
  const Fs = F[0] + F[1];
  const Edd = (p.l1 * Fs * Math.cos(P)
    + p.g * (p.m2 * p.l3 - 2 * p.m1 * p.l1) * Math.cos(E)
    + o.distTorque
    - o.elevDamping * Ed) / p.Je
    + Td * Td * Math.sin(E) * Math.cos(E);
  const Pdd = (p.l2 * (F[0] - F[1]) - o.pitchK * P - o.pitchD * Pd) / p.Jp;
  const Tdd = -p.l1 * Fs * Math.sin(P) / (p.Jt * Math.cos(E)) - 2 * Td * Ed * Math.tan(E);
  return [Ed, Pd, Td, Edd, Pdd, Tdd];
}

function rk4(x, F, h, p, o) {
  const k1 = plantDeriv(x, F, p, o);
  const k2 = plantDeriv(add(x, k1.map(v => v * h / 2)), F, p, o);
  const k3 = plantDeriv(add(x, k2.map(v => v * h / 2)), F, p, o);
  const k4 = plantDeriv(add(x, k3.map(v => v * h)), F, p, o);
  return x.map((xi, i) => xi + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

/** Seeded PRNG so runs are repeatable. */
function rng(seed = 1) {
  let s = seed >>> 0;
  return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296);
}
function gauss(r) {
  return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r());
}

/** Run the full closed loop. Returns arrays sampled once per controller period T. */
export function simulate(ctrl, overrides = {}) {
  const o = { ...DEFAULTS, ...overrides };
  const p = ctrl.params;
  const T = p.T;
  const SUB = 15;
  const h = T / SUB;
  const n = Math.round(o.duration / T);
  const encRes = 2 * Math.PI / 500;
  const quant = v => Math.round(v / encRes) * encRes;
  const rand = rng(7);

  // Start resting on the lower elevation stop (as in the lab runs).
  let x = [p.e_lim_l, 0, 0, 0, 0, 0];
  let z = ctrl.kfInitial.slice();      // Kalman filter state
  let integ = ctrl.kdiInitial.slice(); // integrator state
  let F = [0, 0];                      // fan thrust (lagged)
  let u = [0, 0];                      // last applied deviation control

  const out = { t: [], elev: [], pitch: [], travel: [], elevRef: [], travelRef: [], va: [], vb: [] };

  for (let k = 0; k < n; k++) {
    const t = k * T;
    const r = [schedule(o.elevRef, t), schedule(o.travelRef, t)];

    // --- sensors (encoders) ---
    const y = [x[0], x[1], x[2]].map(v => quant(v + (o.noise ? o.noise * gauss(rand) : 0)));

    // --- Kalman filter: xhat = c z + d [u; y], z+ = a z + b [u; y] ---
    const zin = [...u, ...y];
    const xhat = add(matVec(ctrl.kf.c, z), matVec(ctrl.kf.d, zin));
    z = add(matVec(ctrl.kf.a, z), matVec(ctrl.kf.b, zin));

    // --- LQR with integral action ---
    const yr = matVec(ctrl.Cr, xhat);
    integ = integ.map((v, i) => v + T * (r[i] - yr[i]));
    const kx = matVec(ctrl.Kd, xhat);
    const ki = matVec(ctrl.Kdi, integ);
    u = [ki[0] - kx[0], ki[1] - kx[1]];

    // --- DAC -> power amp -> fan voltage ---
    const V = u.map(ui => clamp(clamp(p.U_e + ui, p.DAC_lim_l, p.DAC_lim_u) * p.k_amp, p.amp_sat_l, p.amp_sat_u));
    const Fss = V.map(v => o.thrustScale * interp(ctrl.fan.volts, ctrl.fan.thrust, v));

    out.t.push(t);
    out.elev.push(x[0]); out.pitch.push(x[1]); out.travel.push(x[2]);
    out.elevRef.push(r[0]); out.travelRef.push(r[1]);
    out.va.push(V[0]); out.vb.push(V[1]);

    // --- continuous plant for one sample period (ZOH on V) ---
    for (let s = 0; s < SUB; s++) {
      F = F.map((f, i) => f + (Fss[i] - f) * (1 - Math.exp(-h / o.fanTau)));
      x = rk4(x, F, h, p, o);
      x[0] = clamp(x[0], p.e_lim_l, p.e_lim_u);   // mechanical stops
      if (x[0] === p.e_lim_l || x[0] === p.e_lim_u) x[3] = 0;
      if (Math.abs(x[1]) >= p.p_lim_u) { x[1] = clamp(x[1], p.p_lim_l, p.p_lim_u); x[4] = 0; }
    }
  }
  return out;
}

/** Hover voltage / fan slope used to linearise the plant (shared with design.js). */
export function hoverPoint(ctrl, o = {}) {
  const p = ctrl.params, { volts, thrust } = ctrl.fan;
  const F0 = -p.g * (p.m2 * p.l3 - 2 * p.m1 * p.l1) / (2 * p.l1); // per fan, level hover
  let i = 1; while (i < thrust.length - 1 && thrust[i] < F0) i++;
  const slope = (thrust[i] - thrust[i - 1]) / (volts[i] - volts[i - 1]);
  const Vh = volts[i - 1] + (F0 - thrust[i - 1]) / slope;
  return { F0, Vh, slope };
}
