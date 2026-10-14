// sim_core.js — the simulated world plus a port of localizer.cpp's logic.
// No drawing code here, so it can run headless in Node for testing.
"use strict";
const simCore = ((loc) => {
  const { kPi } = loc;
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  const clampV = (v, lo, hi) => Math.min(Math.max(v, lo), hi);

  // ---------- Fake hardware ----------
  // Each class mimics the PROS device the robot code reads, including units and error values.
  class FakeImu {          // pros::Imu::get_rotation(): degrees, CLOCKWISE positive, unbounded
    constructor() { this.cwDeg = 0; }
    get_rotation() { return this.cwDeg; }
  }
  class FakeRotation {     // pros::Rotation::get_position(): centidegrees (integer)
    constructor() { this.centideg = 0; }
    get_position() { return Math.round(this.centideg); }
  }
  class FakeDistance {     // pros::Distance::get(): millimeters (integer), 9999 = nothing in range
    constructor() { this.mm = 9999; }
    get() { return this.mm; }
  }

  // trackingWheel() from robot_config.cpp
  function trackingWheel(sensor, diameterIn) {
    let last = 0.0;
    return () => { const cd = sensor.get_position(); last = cd / 36000.0 * kPi * diameterIn; return last; };
  }

  // ---------- Port of localizer.cpp ----------
  class Localizer {
    constructor(cfg, imu, vertInches, horzInches, sensors) {
      this.cfg = cfg; this.imu = imu; this.vert = vertInches; this.horz = horzInches;
      this.sensors = sensors;
      this.mcl = new loc.MCL(cfg.mcl, sensors.map(b => b.mount), cfg.seed ?? 12345);
      this.odomPose = { x: 0, y: 0, theta: 0 };
      this.headingOffset = 0; this.lastImuRad = 0;
      this.lastTheta = 0; this.lastVert = 0; this.lastHorz = 0;
      this.sensorCorrection = true; this.tick = 0;
      this.readings = [];
      this.stats = { updates: 0, skippedTurning: 0, recoveries: 0 };  // sim-only instrumentation
    }
    imuRadians() {
      const deg = this.imu.get_rotation();
      if (Number.isFinite(deg)) this.lastImuRad = -deg * kPi / 180.0;
      return this.lastImuRad;
    }
    setPose(p) {
      this.headingOffset = p.theta - this.imuRadians();
      this.lastTheta = p.theta;
      this.lastVert = this.vert();
      this.lastHorz = this.horz ? this.horz() : 0.0;
      this.odomPose = { ...p };
      this.mcl.init(p.x, p.y, this.cfg.initSpread);
    }
    getPose() { return this.mcl.estimate(this.lastTheta); }
    getOdomPose() { return { ...this.odomPose }; }
    setSensorCorrection(on) { this.sensorCorrection = on; }
    loopOnce() {   // one iteration of Localizer::loop()
      const doSensors = (++this.tick % this.cfg.sensorEveryN) === 0;
      this.step(doSensors);
    }
    step(doSensors) {
      const v = this.vert();
      const h = this.horz ? this.horz() : 0.0;
      const theta = this.headingOffset + this.imuRadians();
      const d = loc.odomStep(this.cfg.odom, v - this.lastVert, h - this.lastHorz, this.lastTheta, theta);
      const turnRate = Math.abs(theta - this.lastTheta) / (this.cfg.loopMs / 1000.0);
      this.lastVert = v; this.lastHorz = h; this.lastTheta = theta;
      this.odomPose.x += d.dx; this.odomPose.y += d.dy; this.odomPose.theta = theta;
      this.mcl.predict(d.dx, d.dy);
      if (!doSensors || !this.sensorCorrection || this.sensors.length === 0) return;
      if (turnRate > this.cfg.maxTurnRateForUpdate) { this.stats.skippedTurning++; return; }
      this.readings = this.sensors.map(b => {
        const mm = b.sensor.get();
        return { valid: mm > 20 && mm < 2000, dist: mm / 25.4 };
      });
      this.mcl.update(this.readings, theta);
      this.stats.updates++;
      if (this.mcl.lastRecover) this.stats.recoveries++;
    }
  }

  // ---------- The simulated world ----------
  const defaultWorldCfg = () => ({
    robotHalf: 8,            // in, robot is 16" square
    wheelDiameter: 2.0,      // what the robot CODE thinks the tracking wheels are
    wheelScaleError: 0.03,   // real wheels roll 3% further than the code thinks
    wheelNoise: 0.002,       // in of random error per 10 ms tick per wheel
    imuDriftDegPerMin: 0,    // gyro drift
    sensorNoiseFrac: 0.01, sensorNoiseMin: 0.2,  // real distance-sensor noise (in)
    garbageProb: 0.05,       // chance a reading is random junk
    objectRadius: 3.5,       // in, game objects that block sensors
    vertLeftOffset: 0.0, horzFwdOffset: -2.0,
    sensorMounts: [
      { name: "Left", fwd: 0.0, left: 6.5, angle: kPi / 2 },
      { name: "Right", fwd: 0.0, left: -6.5, angle: -kPi / 2 },
      { name: "Back", fwd: -7.0, left: 0.0, angle: kPi },
    ],
  });

  class World {
    constructor(wcfg, mclOverrides = {}, seed = 1) {
      this.w = { ...defaultWorldCfg(), ...wcfg };
      this.rng = new loc.MT19937(seed);
      this.n01 = new loc.Normal(0, 1);
      this.objects = [];
      this.time = 0;
      this.truth = { x: -36, y: -36, theta: kPi / 2 };
      this.headingTrue0 = this.truth.theta;
      this.imuDriftRad = 0;
      this.imu = new FakeImu();
      this.vertRot = new FakeRotation(); this.horzRot = new FakeRotation();
      this.sensorsEnabled = this.w.sensorMounts.map(() => true);
      this.dist = this.w.sensorMounts.map(() => new FakeDistance());
      this.rays = this.w.sensorMounts.map(() => null);   // for drawing
      const mcl = { ...loc.defaultMCLConfig(), ...mclOverrides };
      this.locCfg = { odom: { vertLeftOffset: this.w.vertLeftOffset, horzFwdOffset: this.w.horzFwdOffset },
                      mcl, loopMs: 10, sensorEveryN: 3, maxTurnRateForUpdate: 3.0, initSpread: 1.5, seed: seed + 100 };
      this.buildLocalizer();
    }
    buildLocalizer() {
      const bindings = this.w.sensorMounts
        .map((m, i) => ({ sensor: this.dist[i], mount: { fwd: m.fwd, left: m.left, angle: m.angle }, i }))
        .filter(b => this.sensorsEnabled[b.i]);
      this.sensorMap = bindings.map(b => b.i);
      this.localizer = new Localizer(this.locCfg,
        this.imu, trackingWheel(this.vertRot, this.w.wheelDiameter),
        trackingWheel(this.horzRot, this.w.wheelDiameter), bindings);
      this.localizer.setPose({ ...this.truth });   // like autonomous(): start pose is known
    }
    gauss() { return this.n01.sample(this.rng); }
    uni() { return loc.canonical(this.rng); }

    // Distance from a point along a ray to the first wall OR game object.
    trueRay(ox, oy, a) {
      const c = Math.cos(a), s = Math.sin(a), W = this.locCfg.mcl.fieldHalf;
      let t = 1e9;
      if (c > 1e-9) t = Math.min(t, (W - ox) / c); else if (c < -1e-9) t = Math.min(t, (-W - ox) / c);
      if (s > 1e-9) t = Math.min(t, (W - oy) / s); else if (s < -1e-9) t = Math.min(t, (-W - oy) / s);
      for (const o of this.objects) {        // ray-circle intersection
        const fx = ox - o.x, fy = oy - o.y;
        const b = fx * c + fy * s, cc = fx * fx + fy * fy - this.w.objectRadius ** 2;
        const disc = b * b - cc;
        if (disc < 0) continue;
        const hit = -b - Math.sqrt(disc);
        if (hit > 0 && hit < t) t = hit;
      }
      return Math.max(0, t);
    }

    // Advance the world by 10 ms with drive command v (in/s, forward) and w (rad/s, CCW).
    tick(v, w) {
      const dt = 0.01, T = this.truth, W = this.w;
      const dTheta = w * dt;
      let ds = v * dt;
      // Exact arc motion of the robot center.
      const m = T.theta + dTheta / 2;
      const chord = Math.abs(dTheta) < 1e-9 ? ds : 2 * Math.sin(dTheta / 2) * (ds / dTheta);
      let nx = T.x + chord * Math.cos(m), ny = T.y + chord * Math.sin(m);
      const lim = this.locCfg.mcl.fieldHalf - W.robotHalf;
      if (Math.abs(nx) > lim || Math.abs(ny) > lim) { nx = T.x; ny = T.y; ds = 0; }  // against the wall: no translation
      T.x = nx; T.y = ny; T.theta += dTheta;

      // Tracking wheels: true travel of each wheel's contact point, then real-world error.
      const vertTravel = ds - W.vertLeftOffset * dTheta;
      const horzTravel = W.horzFwdOffset * dTheta;
      const toCentideg = (inches) => inches / (kPi * W.wheelDiameter) * 36000;
      this.vertRot.centideg += toCentideg(vertTravel * (1 + W.wheelScaleError) + W.wheelNoise * this.gauss());
      this.horzRot.centideg += toCentideg(horzTravel * (1 + W.wheelScaleError) + W.wheelNoise * this.gauss());

      // IMU: reports rotation since start, clockwise degrees, plus drift.
      this.imuDriftRad += (W.imuDriftDegPerMin * kPi / 180 / 60) * dt;
      this.imu.cwDeg = -((T.theta - this.headingTrue0) + this.imuDriftRad) * 180 / kPi;

      // Distance sensors (computed from the TRUE pose, walls + game objects).
      const ct = Math.cos(T.theta), st = Math.sin(T.theta);
      W.sensorMounts.forEach((s, i) => {
        const sx = T.x + s.fwd * ct - s.left * st, sy = T.y + s.fwd * st + s.left * ct, a = T.theta + s.angle;
        const d = this.trueRay(sx, sy, a);
        let meas = d + this.gauss() * (W.sensorNoiseMin + W.sensorNoiseFrac * d);
        let garbage = false;
        if (this.uni() < W.garbageProb) { meas = this.uni() * d; garbage = true; }
        const mm = Math.round(meas * 25.4);
        this.dist[i].mm = (mm < 20 || mm > 2000) ? 9999 : mm;
        this.rays[i] = { sx, sy, a, d, meas, inRange: this.dist[i].mm !== 9999, garbage };
      });

      this.localizer.loopOnce();
      this.time += dt;
    }

    // Moves the robot WITHOUT the wheels or IMU noticing (wheel slip / bounce in a collision).
    slip(dx, dy) {
      const lim = this.locCfg.mcl.fieldHalf - this.w.robotHalf;
      this.truth.x = clampV(this.truth.x + dx, -lim, lim);
      this.truth.y = clampV(this.truth.y + dy, -lim, lim);
    }
    errors() {
      const e = this.localizer.getPose(), o = this.localizer.getOdomPose(), T = this.truth;
      return { mcl: Math.hypot(e.x - T.x, e.y - T.y), odom: Math.hypot(o.x - T.x, o.y - T.y),
               heading: Math.abs(wrap(e.theta - T.theta)) * 180 / kPi };
    }
  }

  // ---------- Autopilot: drives waypoints using a pose the ROBOT believes ----------
  function autopilot(pose, target) {
    const dx = target.x - pose.x, dy = target.y - pose.y, dist = Math.hypot(dx, dy);
    const err = wrap(Math.atan2(dy, dx) - pose.theta);
    const w = clampV(5 * err, -4, 4);
    const v = Math.abs(err) > 0.8 ? 0 : clampV(dist * 3, 0, 40) * Math.cos(err);
    return { v, w, dist };
  }

  return { FakeImu, FakeRotation, FakeDistance, trackingWheel, Localizer, World, defaultWorldCfg, autopilot, wrap };
})(typeof loc !== "undefined" ? loc : require("./mcl_port.js"));
if (typeof module !== "undefined") module.exports = simCore;