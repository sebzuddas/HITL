// Small dense-matrix toolkit + discrete LQR design (port of MATLAB lqrd) so the
// browser can re-design the controller from user-chosen Q and R weights.
// Matrices are arrays of row arrays.

export const eye = n => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => +(i === j)));
export const zeros = (r, c) => Array.from({ length: r }, () => new Array(c).fill(0));
export const T = A => A[0].map((_, j) => A.map(r => r[j]));
export const add = (A, B) => A.map((r, i) => r.map((v, j) => v + B[i][j]));
export const sub = (A, B) => A.map((r, i) => r.map((v, j) => v - B[i][j]));
export const scale = (A, s) => A.map(r => r.map(v => v * s));
export function mul(A, B) {
  const n = A.length, m = B[0].length, k = B.length, C = zeros(n, m);
  for (let i = 0; i < n; i++) for (let l = 0; l < k; l++) { const a = A[i][l]; if (a) for (let j = 0; j < m; j++) C[i][j] += a * B[l][j]; }
  return C;
}
export function inv(A) {
  const n = A.length, M = A.map((r, i) => [...r, ...eye(n)[i]]);
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-300) throw new Error('singular matrix');
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c]; for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; if (f) for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[c][j]; }
  }
  return M.map(r => r.slice(n));
}
const norm1 = A => Math.max(...A[0].map((_, j) => A.reduce((s, r) => s + Math.abs(r[j]), 0)));

/** Matrix exponential: scaling and squaring with a Taylor series. */
export function expm(A) {
  const n = A.length;
  const s = Math.max(0, Math.ceil(Math.log2(Math.max(norm1(A), 1e-12))) + 1);
  const As = scale(A, 1 / 2 ** s);
  let term = eye(n), sum = eye(n);
  for (let k = 1; k <= 18; k++) { term = scale(mul(term, As), 1 / k); sum = add(sum, term); }
  for (let i = 0; i < s; i++) sum = mul(sum, sum);
  return sum;
}

const block = (A, r0, r1, c0, c1) => A.slice(r0, r1).map(r => r.slice(c0, c1));

/** Discretise plant and continuous cost (Van Loan). Returns {Ad, Bd, Qd, Nd, Rd}. */
export function discretiseCost(A, B, Q, R, Ts) {
  const n = A.length, m = B[0].length, k = n + m;
  const Aa = zeros(k, k), Qa = zeros(k, k);
  for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) { Aa[i][j] = A[i][j]; Qa[i][j] = Q[i][j]; } for (let j = 0; j < m; j++) Aa[i][n + j] = B[i][j]; }
  for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) Qa[n + i][n + j] = R[i][j];
  const F = zeros(2 * k, 2 * k), AaT = T(Aa);
  for (let i = 0; i < k; i++) for (let j = 0; j < k; j++) { F[i][j] = -AaT[i][j] * Ts; F[i][k + j] = Qa[i][j] * Ts; F[k + i][k + j] = Aa[i][j] * Ts; }
  const E = expm(F);
  const Phi = block(E, k, 2 * k, k, 2 * k);
  const Qd = mul(T(Phi), block(E, 0, k, k, 2 * k));
  return {
    Ad: block(Phi, 0, n, 0, n), Bd: block(Phi, 0, n, n, k),
    Qd: block(Qd, 0, n, 0, n), Nd: block(Qd, 0, n, n, k), Rd: block(Qd, n, k, n, k),
  };
}

/** Solve the DARE with the structured doubling algorithm; returns P. */
export function dare(A, B, Q, R) {
  const n = A.length;
  let Ak = A, Gk = mul(mul(B, inv(R)), T(B)), Hk = Q;
  for (let it = 0; it < 60; it++) {
    const Winv = inv(add(eye(n), mul(Gk, Hk)));
    const AW = mul(Ak, Winv);
    const Hn = add(Hk, mul(mul(T(Ak), Hk), mul(Winv, Ak)));
    const Gn = add(Gk, mul(mul(AW, Gk), T(Ak)));
    Ak = mul(AW, Ak);
    const d = Math.max(...Hn.map((r, i) => Math.max(...r.map((v, j) => Math.abs(v - Hk[i][j])))));
    Gk = Gn; Hk = Hn;
    if (d < 1e-12 * (1 + Math.max(...Hn.flat().map(Math.abs)))) break;
  }
  return Hk;
}

/** MATLAB lqrd(A,B,Q,R,Ts): LQR gain K (u = -Kx) for a continuous cost sampled at Ts. */
export function lqrd(A, B, Q, R, Ts) {
  const { Ad, Bd, Qd, Nd, Rd } = discretiseCost(A, B, Q, R, Ts);
  const Rinv = inv(Rd);
  const Ae = sub(Ad, mul(mul(Bd, Rinv), T(Nd)));
  const Qe = sub(Qd, mul(mul(Nd, Rinv), T(Nd)));
  const P = dare(Ae, Bd, Qe, Rd);
  // K = (Rd + B'PB)^-1 (B'PA + N') with the cross term folded in
  const BtP = mul(T(Bd), P);
  return mul(inv(add(Rd, mul(BtP, Bd))), add(mul(BtP, Ad), T(Nd)));
}

export const diag = v => v.map((x, i) => v.map((_, j) => (i === j ? x : 0)));
