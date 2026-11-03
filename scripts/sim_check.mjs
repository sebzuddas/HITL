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
