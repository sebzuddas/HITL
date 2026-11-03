// Tiny dependency-free canvas line plotter.
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export function plot(canvas, t, series, { cursor, ylabel = '' } = {}) {
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height, m = { l: 52, r: 12, t: 8, b: 24 };
  ctx.clearRect(0, 0, W, H);
  let lo = Infinity, hi = -Infinity;
  for (const s of series) for (const v of s.y) { if (v < lo) lo = v; if (v > hi) hi = v; }
  if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
  const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
  const t1 = t[t.length - 1];
  const X = x => m.l + (x / t1) * (W - m.l - m.r);
  const Y = y => H - m.b - ((y - lo) / (hi - lo)) * (H - m.t - m.b);

  ctx.font = '12px system-ui,sans-serif';
  ctx.fillStyle = css('--muted'); ctx.strokeStyle = css('--line'); ctx.lineWidth = 1;
  const step = niceStep((hi - lo) / 5);
  for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
    ctx.beginPath(); ctx.moveTo(m.l, Y(v)); ctx.lineTo(W - m.r, Y(v)); ctx.stroke();
    ctx.textAlign = 'right'; ctx.fillText(+v.toFixed(2), m.l - 6, Y(v) + 4);
  }
  ctx.textAlign = 'center';
  for (let s = 0; s <= t1; s += 10) ctx.fillText(s + ' s', X(s), H - 6);
  if (ylabel) { ctx.save(); ctx.translate(12, H / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(ylabel, 0, 0); ctx.restore(); }

  for (const s of series) {
    ctx.strokeStyle = css(s.color); ctx.lineWidth = s.width || 1.6; ctx.beginPath();
    const stride = Math.max(1, Math.floor(t.length / (W * 2)));
    for (let i = 0; i < t.length; i += stride) {
      i === 0 ? ctx.moveTo(X(t[i]), Y(s.y[i])) : ctx.lineTo(X(t[i]), Y(s.y[i]));
    }
    ctx.stroke();
  }
  if (cursor != null) {
    ctx.strokeStyle = css('--fg'); ctx.globalAlpha = 0.4; ctx.beginPath();
    ctx.moveTo(X(cursor), m.t); ctx.lineTo(X(cursor), H - m.b); ctx.stroke(); ctx.globalAlpha = 1;
  }
}

function niceStep(raw) {
  const p = 10 ** Math.floor(Math.log10(raw)), f = raw / p;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p;
}
