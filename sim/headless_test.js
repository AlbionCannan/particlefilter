// Headless test of the simulator's world + localizer port (no browser).
const loc = require("./mcl_port.js");
global.loc = loc;
const S = require("./sim_core.js");
let fails = 0;
const check = (ok, msg) => { console.log(`[${ok ? "PASS" : "FAIL"}] ${msg}`); fails += !ok; };
const square = [{ x: 36, y: -36 }, { x: 36, y: 36 }, { x: -36, y: 36 }, { x: -36, y: -36 }];

function run(opts) {
  const w = new S.World(opts.world || {}, opts.mcl || {}, opts.seed || 1);
  if (opts.correction === false) w.localizer.setSensorCorrection(false);
  if (opts.objects) w.objects = opts.objects;
  let wp = 0, worst = 0, sum = 0, n = 0, osum = 0;
  for (let t = 0; t < (opts.ticks || 6000); t++) {
    const pose = opts.steerBy === "truth" ? w.truth : w.localizer.getPose();
    const cmd = S.autopilot(pose, square[wp]);
    if (cmd.dist < 3) wp = (wp + 1) % 4;
    w.tick(cmd.v, cmd.w);
    if (opts.slipAt && t === opts.slipAt) w.slip(opts.slip[0], opts.slip[1]);
    const e = w.errors();
    if (t > 300) { worst = Math.max(worst, e.mcl); sum += e.mcl; osum += e.odom; n++; }
  }
  return { w, worst, mean: sum / n, odomMean: osum / n, final: w.errors() };
}

// 1. Perfect hardware: odometry must reconstruct the truth almost exactly (checks the
//    wheel kinematics, unit conversions and IMU sign convention all agree).
{ const r = run({ world: { wheelScaleError: 0, wheelNoise: 0, garbageProb: 0 }, correction: false, ticks: 3000 });
  check(r.final.odom < 0.3 && r.final.heading < 0.01,
        `perfect wheels + no sensors: odometry error ${r.final.odom.toFixed(3)} in, heading error ${r.final.heading.toFixed(4)}°`); }
// 2. Default world (3% wheel error, 5% junk readings). Compare AVERAGE error: on a closed
//    square a wheel-scale error partly cancels, so the final odometry error alone is misleading.
{ const r = run({ seed: 3 });
  check(r.mean < 1.5 && r.worst < 3 && r.odomMean > 1.5 * r.mean,
        `60 s autopilot: MCL mean ${r.mean.toFixed(2)} in (worst ${r.worst.toFixed(2)}) vs odometry mean ${r.odomMean.toFixed(2)} in`);
  check(r.w.localizer.stats.updates > 1000, `sensor updates ran: ${r.w.localizer.stats.updates}, skipped while turning: ${r.w.localizer.stats.skippedTurning}`); }
// 3. Slip of 8": filter recovers, odometry can't.
{ const r = run({ seed: 4, slipAt: 2000, slip: [8, 0], ticks: 2400 });
  check(r.final.mcl < 2.5 && r.final.odom > 6, `after 8" slip + 4 s: MCL ${r.final.mcl.toFixed(2)} in vs odometry ${r.final.odom.toFixed(2)} in (recoveries: ${r.w.localizer.stats.recoveries})`); }
// 4. Game objects blocking sensors: filter must stay usable.
{ const objs = [{ x: 30, y: -10 }, { x: -50, y: 0 }, { x: 0, y: 50 }, { x: 10, y: -55 }, { x: -20, y: -20 }];
  const r = run({ seed: 5, objects: objs });
  check(r.final.mcl < 3 && r.worst < 6, `with 5 game objects: MCL final ${r.final.mcl.toFixed(2)} in, worst ${r.worst.toFixed(2)} in`); }
// 5. Many seeds with game objects (the case that exposed the false-recovery bug).
{ let worstAll = 0, badSeeds = 0;
  for (let seed = 10; seed < 110; seed++) {
    const objs = [];
    const g = new loc.MT19937(seed * 31);
    for (let k = 0; k < 6; k++) objs.push({ x: (loc.canonical(g) * 2 - 1) * 60, y: (loc.canonical(g) * 2 - 1) * 60 });
    const r = run({ seed, objects: objs, ticks: 4000 });
    worstAll = Math.max(worstAll, r.worst); if (r.worst > 4) badSeeds++;
  }
  check(badSeeds === 0, `100 random object layouts: worst error anywhere ${worstAll.toFixed(2)} in, layouts over 4 in: ${badSeeds}`); }
process.exit(fails);