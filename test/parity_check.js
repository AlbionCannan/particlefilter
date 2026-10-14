// Replays the C++ trace through the JavaScript port and reports any difference.
//   node test/parity_check.js trace.json [seed]   (seed must match the one given to parity_trace)
const fs = require("fs");
const loc = require("../sim/mcl_port.js");
const T = JSON.parse(fs.readFileSync(process.argv[2] || "trace.json", "utf8"));
let fails = 0;
const report = (ok, msg) => { console.log(`[${ok ? "PASS" : "FAIL"}] ${msg}`); fails += !ok; };
// The browser's Math.sin/cos/exp/log may differ from C++'s in the last binary digit, so
// numbers are compared to a rounding-error tolerance. Random numbers and decisions must be identical.
const TOL = 1e-9;  // inches

{ const g = new loc.MT19937(12345); let bad = -1;
  for (let i = 0; i < T.mt.length; i++) if (g.next() !== T.mt[i]) { bad = i; break; }
  report(bad < 0, `mt19937: ${T.mt.length} outputs identical` + (bad >= 0 ? ` (first diff at ${bad})` : "")); }

{ let maxDiff = 0;
  for (const [vl, hf, dv, dh, t0, t1, ex, ey] of T.odom) {
    const d = loc.odomStep({ vertLeftOffset: vl, horzFwdOffset: hf }, dv, dh, t0, t1);
    maxDiff = Math.max(maxDiff, Math.abs(d.dx - ex), Math.abs(d.dy - ey)); }
  report(maxDiff < TOL, `odomStep: ${T.odom.length} cases, max difference ${maxDiff.toExponential(1)} in`); }

{ const sensors = [{ fwd: 0, left: 6.5, angle: loc.kPi / 2 }, { fwd: 0, left: -6.5, angle: -loc.kPi / 2 },
                   { fwd: -7, left: 0, angle: loc.kPi }];
  const seed = process.argv[3] ? Number(process.argv[3]) : 2024;
  const m = new loc.MCL(loc.defaultMCLConfig(), sensors, seed);
  const ck = () => m.particles.reduce((c, p) => c + p.x * 1.0001 + p.y * 2.0003 + p.w * 1000.0 + (p.scored ? 0.5 : 0), 0);
  let maxEst = 0, maxCk = 0, firstDiff = -1, recov = 0, decisionMismatch = 0, resamples = 0;
  T.ops.forEach((o, i) => {
    if (o.op === "init") m.init(o.x, o.y, o.s);
    else if (o.op === "predict") m.predict(o.dx, o.dy);
    else { m.update(o.r.map(([v, d]) => ({ valid: !!v, dist: d })), o.th);
           const inj = m.particles.some(p => !p.scored); recov += inj;
           const rs = m.particles.every(p => p.w === 1.0 / m.particles.length);
           resamples += rs;
           if ((rs ? 1 : 0) !== o.rs || (inj ? 1 : 0) !== o.inj) decisionMismatch++; }
    const de = Math.max(Math.abs(m.estX - o.est[0]), Math.abs(m.estY - o.est[1]));
    const dc = Math.abs(ck() - o.ck);
    if ((de > TOL || dc > 1e-6) && firstDiff < 0) firstDiff = i;
    maxEst = Math.max(maxEst, de); maxCk = Math.max(maxCk, dc);
  });
  report(firstDiff < 0,
    `full filter: ${T.ops.length} operations, max estimate diff ${maxEst.toExponential(1)} in, ` +
    `max particle-checksum diff ${maxCk.toExponential(1)}` +
    (firstDiff >= 0 ? `, first diff beyond tolerance at op ${firstDiff} (${T.ops[firstDiff].op})` : ""));
  report(decisionMismatch === 0, `resample/recovery decisions: ${decisionMismatch} mismatches (${resamples} resamples)`);
  report(recov === T.recoveries && recov > 0, `recovery fired ${recov} times in JS vs ${T.recoveries} in C++`); }
process.exit(fails);