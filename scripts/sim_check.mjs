// Headless sanity check of the web simulation:  node scripts/sim_check.mjs
import { readFileSync } from 'node:fs';
import { simulate } from '../web/js/sim.js';
const ctrl = JSON.parse(readFileSync(new URL('../web/data/controller.json', import.meta.url)));
const o = simulate(ctrl);
const d = 180 / Math.PI;
for (const s of [0, 5, 10, 20, 29, 35, 45, 59]) {
  const i = Math.min(Math.round(s / ctrl.params.T), o.t.length - 1);
  console.log(`t=${s}s elev=${(o.elev[i]*d).toFixed(1)} pitch=${(o.pitch[i]*d).toFixed(1)} travel=${(o.travel[i]*d).toFixed(1)} Va=${o.va[i].toFixed(2)} Vb=${o.vb[i].toFixed(2)}`);
}
if (o.elev.some(Number.isNaN)) { console.error('NaN!'); process.exit(1); }

// LQR redesign check: weights fitted to the original gains must reproduce them closely and still fly.
import { designLQR } from '../web/js/design.js';
const g = designLQR(ctrl, { qx: [850, 22000, 670, 1100, 190, 22000, 190, 150], qu: [1, 1] });
const rel = Math.abs(g.Kd[0][1] / ctrl.Kd[0][1] - 1);
const o2 = simulate({ ...ctrl, ...g });
const fin = o2.travel.at(-1) * d;
console.log(`LQR redesign: pitch gain within ${(rel * 100).toFixed(1)}% of original, final travel ${fin.toFixed(1)} deg`);
if (rel > 0.1 || Math.abs(fin) > 15 || o2.elev.some(Number.isNaN)) { console.error('LQR redesign check failed'); process.exit(1); }
