// Controller design: linearise the plant at hover, then LQR with integral action.
import { lqrd, diag, zeros } from './lqr.js';
import { hoverPoint, DEFAULTS } from './sim.js';

/** State order x = [elev, pitch, travel, elev_dot, pitch_dot, travel_dot]; inputs = fan voltage deviations (V). */
export function linearModel(ctrl, o = {}) {
  const p = ctrl.params, d = { ...DEFAULTS, ...o };
  const { F0, slope } = hoverPoint(ctrl);
  const b = slope * p.k_amp * d.thrustScale;           // N per control volt
  const A = zeros(6, 6), B = zeros(6, 2);
  for (let i = 0; i < 3; i++) A[i][i + 3] = 1;
  A[3][3] = -d.elevDamping / p.Je;
  A[4][1] = -d.pitchK / p.Jp; A[4][4] = -d.pitchD / p.Jp;
  A[5][1] = -p.l1 * 2 * F0 / p.Jt;
  B[3] = [p.l1 * b / p.Je, p.l1 * b / p.Je];
  B[4] = [p.l2 * b / p.Jp, -p.l2 * b / p.Jp];
  return { A, B };
}

/** weights: {qx: 8 diagonal state weights [x(6), int_elev, int_trav], qu: [r1, r2]}. Returns {Kd, Kdi}. */
export function designLQR(ctrl, weights, o = {}) {
  const { A, B } = linearModel(ctrl, o), p = ctrl.params;
  const Cr = ctrl.Cr, n = 6, r = 2;
  const Aaug = zeros(n + r, n + r), Baug = zeros(n + r, 2);
  for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) Aaug[i][j] = A[i][j]; Baug[i] = [...B[i]]; }
  for (let i = 0; i < r; i++) for (let j = 0; j < n; j++) Aaug[n + i][j] = -Cr[i][j];
  const K = lqrd(Aaug, Baug, diag(weights.qx), diag(weights.qu), p.T);
  return { Kd: K.map(row => row.slice(0, n)), Kdi: K.map(row => row.slice(n).map(v => -v)) };
}
