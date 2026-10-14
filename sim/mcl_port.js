// mcl_port.js — line-for-line JavaScript port of mcl.hpp + odometry.hpp.
// Includes an exact replica of GCC's std::mt19937 and the libstdc++
// distributions the C++ uses, so with the same seed and inputs this produces
// the SAME numbers as the C++ (verified by test/parity_check.js).
// If you change mcl.hpp or odometry.hpp, change this file to match and rerun the parity check.
"use strict";
const loc = (() => {
  const kPi = 3.14159265358979323846;

  // ---------- std::mt19937 (exact) ----------
  class MT19937 {
    constructor(seed) {
      this.mt = new Uint32Array(624); this.idx = 624;
      this.mt[0] = seed >>> 0;
      for (let i = 1; i < 624; i++) {
        const p = this.mt[i - 1] ^ (this.mt[i - 1] >>> 30);
        // 1812433253 * p + i, mod 2^32, without losing precision
        this.mt[i] = (((((p & 0xffff0000) >>> 16) * 1812433253) << 16) + (p & 0x0000ffff) * 1812433253 + i) >>> 0;
      }
    }
    next() {
      const mt = this.mt;
      if (this.idx >= 624) {
        for (let k = 0; k < 624; k++) {
          const y = (mt[k] & 0x80000000) | (mt[(k + 1) % 624] & 0x7fffffff);
          mt[k] = (mt[(k + 397) % 624] ^ (y >>> 1) ^ ((y & 1) ? 0x9908b0df : 0)) >>> 0;
        }
        this.idx = 0;
      }
      let y = mt[this.idx++];
      y ^= y >>> 11;
      y = (y ^ ((y << 7) & 0x9d2c5680)) >>> 0;
      y = (y ^ ((y << 15) & 0xefc60000)) >>> 0;
      y ^= y >>> 18;
      return y >>> 0;
    }
  }
  // std::generate_canonical<double, 53>(mt19937): two 32-bit draws.
  function canonical(rng) {
    let sum = 0, tmp = 1;
    for (let k = 0; k < 2; k++) { sum += rng.next() * tmp; tmp *= 4294967296; }
    let ret = sum / tmp;
    if (ret >= 1) ret = 1 - Number.EPSILON / 2;  // nextafter(1, 0)
    return ret;
  }
  // std::normal_distribution<double> (Marsaglia polar, caches 2nd value).
  // Each C++ distribution OBJECT has its own cache, so create one per C++ scope.
  class Normal {
    constructor(mean, sd) { this.mean = mean; this.sd = sd; this.saved = 0; this.has = false; }
    sample(rng) {
      let ret;
      if (this.has) { this.has = false; ret = this.saved; }
      else {
        let x, y, r2;
        do { x = 2.0 * canonical(rng) - 1.0; y = 2.0 * canonical(rng) - 1.0; r2 = x * x + y * y; }
        while (r2 > 1.0 || r2 === 0.0);
        const mult = Math.sqrt(-2 * Math.log(r2) / r2);
        this.saved = x * mult; this.has = true; ret = y * mult;
      }
      return ret * this.sd + this.mean;
    }
  }
  // std::uniform_real_distribution<double>(a, b)
  const uniformReal = (rng, a, b) => canonical(rng) * (b - a) + a;
  // std::uniform_int_distribution<int>(0, hi) with mt19937: Lemire's method (GCC 13).
  function uniformInt(rng, hi) {
    const range = BigInt(hi) + 1n, M = 1n << 32n;
    let product = BigInt(rng.next()) * range, low = product % M;
    if (low < range) {
      const threshold = (M - range) % range;
      while (low < threshold) { product = BigInt(rng.next()) * range; low = product % M; }
    }
    return Number(product >> 32n);
  }

  // ---------- odometry.hpp ----------
  function odomStep(c, dVert, dHorz, thetaPrev, thetaNow) {
    const dTheta = thetaNow - thetaPrev;
    let fwd, left;
    if (Math.abs(dTheta) < 1e-6) { fwd = dVert; left = dHorz; }
    else {
      const chord = 2.0 * Math.sin(dTheta / 2.0);
      fwd = chord * (dVert / dTheta + c.vertLeftOffset);
      left = chord * (dHorz / dTheta - c.horzFwdOffset);
    }
    const m = thetaPrev + dTheta / 2.0;
    const cm = Math.cos(m), sm = Math.sin(m);
    return { dx: fwd * cm - left * sm, dy: fwd * sm + left * cm };
  }

  // ---------- mcl.hpp ----------
  const defaultMCLConfig = () => ({
    numParticles: 250, fieldHalf: 70.2, maxRange: 78.0,
    transNoiseFrac: 0.05, transNoiseMin: 0.02,
    sensorSigmaFrac: 0.05, sensorSigmaMin: 0.6, outlierProb: 0.10,
    resampleJitter: 0.1, resampleRatio: 0.5,
    recoveryResidual: 3.0, recoveryMaxResidual: 12.0, recoveryCount: 4, recoveryCountShort: 6, recoveryFrac: 0.5, recoverySpread: 8.0,
  });
  // NOTE: Math.sin/cos/exp/log can differ from C++'s math library in the LAST binary digit
  // (~1e-16 relative). The C++ itself also differs between your laptop (glibc) and the
  // V5 (newlib). So results match the C++ to rounding error, not bit-for-bit.
  const hypot = Math.hypot;

  class MCL {
    constructor(cfg, sensors, seed = 12345) {
      this.cfg = { ...cfg }; this.sensors = sensors.map(s => ({ ...s }));
      this.longCnt = new Array(this.sensors.length).fill(0);
      this.shortCnt = new Array(this.sensors.length).fill(0);
      this.particles = []; this.geo = []; this.rng = new MT19937(seed);
      this.estX = 0; this.estY = 0;
      this.lastRecover = false;  // (sim-only instrumentation, not in C++)
      this.lastResampled = false;
    }
    init(x, y, spread) {
      const n = new Normal(0.0, spread), N = this.cfg.numParticles, w = 1.0 / N;
      this.particles = [];
      for (let i = 0; i < N; i++) {
        const px = this.clamp(x + n.sample(this.rng)); const py = this.clamp(y + n.sample(this.rng));
        this.particles.push({ x: px, y: py, w, scored: true });
      }
      this.computeEstimate();
      this.longCnt.fill(0); this.shortCnt.fill(0);
    }
    predict(dx, dy) {
      const dist = hypot(dx, dy);
      if (dist < 1e-4) return;
      const n = new Normal(0.0, this.cfg.transNoiseFrac * dist + this.cfg.transNoiseMin);
      for (const p of this.particles) {
        p.x = this.clamp(p.x + dx + n.sample(this.rng));
        p.y = this.clamp(p.y + dy + n.sample(this.rng));
      }
      this.computeEstimate();
    }
    update(readings, theta) {
      this.lastRecover = false; this.lastResampled = false;
      if (this.particles.length === 0) return false;
      this.geo = [];
      const ct = Math.cos(theta), st = Math.sin(theta);
      for (let i = 0; i < this.sensors.length && i < readings.length; i++) {
        if (!readings[i].valid) continue;
        const sc = this.sensors[i], a = theta + sc.angle;
        this.geo.push({ ox: sc.fwd * ct - sc.left * st, oy: sc.fwd * st + sc.left * ct,
                        c: Math.cos(a), s: Math.sin(a), meas: readings[i].dist, idx: i });
      }
      if (this.geo.length === 0) return false;

      let recover = false;
      for (const g of this.geo) {
        const expected = this.raycast(this.estX + g.ox, this.estY + g.oy, g.c, g.s);
        const tol = Math.max(this.cfg.recoveryResidual, 3.0 * this.sigmaFor(expected));
        const residual = Math.abs(g.meas - expected);
        const plausible = expected <= this.cfg.maxRange && residual <= this.cfg.recoveryMaxResidual;
        if (residual <= tol) {
          this.longCnt[g.idx] = 0;
          this.shortCnt[g.idx] = 0;
        } else if (plausible) {
          if (g.meas > expected) this.longCnt[g.idx]++;
          else                   this.shortCnt[g.idx]++;
        }
        if (this.longCnt[g.idx] >= this.cfg.recoveryCount || this.shortCnt[g.idx] >= this.cfg.recoveryCountShort) {
          recover = true;
          this.longCnt[g.idx] = 0;
          this.shortCnt[g.idx] = 0;
        }
      }

      let sum = 0.0;
      for (const p of this.particles) {
        let like = 1.0;
        for (const g of this.geo) like *= this.likelihood(g.meas, this.raycast(p.x + g.ox, p.y + g.oy, g.c, g.s));
        p.w *= like; p.scored = true; sum += p.w;
      }
      if (!(sum > 1e-300) || !Number.isFinite(sum)) {
        const w = 1.0 / this.particles.length;
        for (const p of this.particles) p.w = w;
      } else {
        for (const p of this.particles) p.w /= sum;
      }
      let sumSq = 0.0;
      for (const p of this.particles) sumSq += p.w * p.w;
      this.nEff = 1.0 / sumSq;  // (sim-only instrumentation)
      const nInject = recover ? Math.trunc(this.cfg.recoveryFrac * this.particles.length) : 0;
      if (nInject > 0 || 1.0 / sumSq < this.cfg.resampleRatio * this.particles.length) {
        this.computeEstimate();
        const cx = this.estX, cy = this.estY;
        this.resample();
        this.lastResampled = true;
        const n = new Normal(0.0, this.cfg.recoverySpread);
        for (let k = 0; k < nInject; k++) {
          const p = this.particles[uniformInt(this.rng, this.particles.length - 1)];
          p.x = this.clamp(cx + n.sample(this.rng));
          p.y = this.clamp(cy + n.sample(this.rng));
          p.scored = false;
        }
      }
      this.lastRecover = recover;
      this.computeEstimate();
      return true;
    }
    estimate(theta) { return { x: this.estX, y: this.estY, theta }; }
    raycast(ox, oy, c, s) {
      const W = this.cfg.fieldHalf;
      let t = 1e9;
      if (c > 1e-9) t = Math.min(t, (W - ox) / c);
      else if (c < -1e-9) t = Math.min(t, (-W - ox) / c);
      if (s > 1e-9) t = Math.min(t, (W - oy) / s);
      else if (s < -1e-9) t = Math.min(t, (-W - oy) / s);
      return Math.max(0.0, t);
    }
    sigmaFor(expected) { return Math.max(this.cfg.sensorSigmaMin, this.cfg.sensorSigmaFrac * expected); }
    likelihood(measured, expected) {
      const sigma = this.sigmaFor(expected);
      const z = (measured - expected) / sigma;
      const gauss = Math.exp(-0.5 * z * z) / (sigma * Math.sqrt(2.0 * kPi));
      return (1.0 - this.cfg.outlierProb) * gauss + this.cfg.outlierProb / this.cfg.maxRange;
    }
    resample() {
      const N = this.particles.length, out = [];
      const jitter = new Normal(0.0, this.cfg.resampleJitter);
      const r = uniformReal(this.rng, 0.0, 1.0 / N);
      let cum = this.particles[0].w, i = 0;
      for (let m = 0; m < N; m++) {
        const U = r + m / N;
        while (U > cum && i < N - 1) cum += this.particles[++i].w;
        const p = { ...this.particles[i] };
        p.x = this.clamp(p.x + jitter.sample(this.rng));
        p.y = this.clamp(p.y + jitter.sample(this.rng));
        p.w = 1.0 / N;
        out.push(p);
      }
      this.particles = out;
    }
    computeEstimate() {
      let x = 0, y = 0, wsum = 0;
      for (const p of this.particles) {
        if (!p.scored) continue;
        x += p.w * p.x; y += p.w * p.y; wsum += p.w;
      }
      if (wsum > 0) { this.estX = x / wsum; this.estY = y / wsum; }
    }
    clamp(v) { return Math.min(Math.max(v, -this.cfg.fieldHalf), this.cfg.fieldHalf); }
  }
  return { kPi, MT19937, Normal, canonical, uniformInt, odomStep, defaultMCLConfig, MCL };
})();
if (typeof module !== "undefined") module.exports = loc;