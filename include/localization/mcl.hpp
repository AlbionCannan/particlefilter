// mcl.hpp — Monte Carlo Localization (particle filter). Pure C++, no PROS.
//
// Frame: origin at field center, +x right, +y away from the driver station,
// inches, theta = radians counterclockwise from +x.
// Heading is NOT filtered (it comes from the IMU); particles carry (x, y).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace loc {

constexpr double kPi = 3.14159265358979323846;

struct Pose { double x = 0, y = 0, theta = 0; };
struct Particle { double x = 0, y = 0, w = 0; bool scored = true; };  // scored=false: injected, not yet checked by sensors

// Mounting of a distance sensor relative to the robot's TRACKING CENTER,
// measured to the sensor's FACE.
struct DistanceSensorConfig {
  double fwd;    // in, forward of tracking center (negative = behind)
  double left;   // in, left of tracking center (negative = right)
  double angle;  // rad, CCW from robot-forward (left = +pi/2, right = -pi/2, back = pi)
};

struct Reading { bool valid; double dist; };  // dist in inches

struct MCLConfig {
  int numParticles = 250;
  double fieldHalf = 70.2;        // HALF the inside wall-to-wall width. MEASURE YOUR FIELD.
  double maxRange = 78.0;         // V5 distance sensor max ~2000 mm
  double transNoiseFrac = 0.05;   // motion noise std = 5% of distance moved...
  double transNoiseMin = 0.02;    // ...plus this floor (only while moving)
  double sensorSigmaFrac = 0.05;  // sensor noise std = 5% of expected distance...
  double sensorSigmaMin = 0.6;    // ...never below this
  double outlierProb = 0.10;      // chance a reading is garbage (ball, robot, goal)
  double resampleJitter = 0.1;    // in, noise added after resampling
  double resampleRatio = 0.5;     // resample when N_eff < ratio * N
  // Recovery: if the SAME sensor disagrees with the current estimate by more
  // than recoveryResidual for recoveryCount updates IN A ROW, the pose is
  // probably wrong (collision, wheel slip), not just a noisy reading. Then
  // inject fresh particles around the estimate. A single garbage reading
  // (ball, robot) almost never repeats, so it won't trigger this.
  // Injection is LOCAL on purpose: the field is symmetric, so field-wide
  // injection could jump to a mirror-image pose.
  double recoveryResidual = 3.0;  // in
  // A disagreement only counts as evidence of a pose error if a pose error
  // could explain it: the wall must be within sensor range from the estimate,
  // and the disagreement can't exceed a plausible collision shift. Otherwise
  // it's almost certainly a game object or robot in front of the sensor.
  double recoveryMaxResidual = 12.0;  // in (tuned in sim/headless_test.js)
  // An object can only make a reading SHORTER than the wall, so a reading
  // that's too LONG is strong evidence of a pose error and triggers quickly.
  // A too-short reading might just be a ball or robot passing in front of the
  // sensor, so it needs to persist longer before triggering recovery.
  int recoveryCount = 4;          // consecutive too-LONG disagreements
  int recoveryCountShort = 6;     // consecutive too-SHORT disagreements (~180 ms)
  double recoveryFrac = 0.5;      // fraction of particles to replace
  double recoverySpread = 8.0;    // in, std dev of injected particles
};

class MCL {
 public:
  MCL(const MCLConfig& cfg, std::vector<DistanceSensorConfig> sensors, std::uint32_t seed = 12345)
      : cfg_(cfg), sensors_(std::move(sensors)), longCnt_(sensors_.size(), 0), shortCnt_(sensors_.size(), 0), rng_(seed) {}

  void init(double x, double y, double spread) {
    std::normal_distribution<double> n(0.0, spread);
    particles_.assign(cfg_.numParticles, Particle{});
    const double w = 1.0 / cfg_.numParticles;
    for (auto& p : particles_) { p.x = clamp(x + n(rng_)); p.y = clamp(y + n(rng_)); p.w = w; }
    computeEstimate();
    std::fill(longCnt_.begin(), longCnt_.end(), 0);
    std::fill(shortCnt_.begin(), shortCnt_.end(), 0);
  }

  // Field-frame displacement since the last call.
  void predict(double dx, double dy) {
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-4) return;
    std::normal_distribution<double> n(0.0, cfg_.transNoiseFrac * dist + cfg_.transNoiseMin);
    for (auto& p : particles_) { p.x = clamp(p.x + dx + n(rng_)); p.y = clamp(p.y + dy + n(rng_)); }
    computeEstimate();
  }

  // readings[i] corresponds to sensor i. theta from the IMU. Returns true if
  // any valid reading was used.
  bool update(const std::vector<Reading>& readings, double theta) {
    if (particles_.empty()) return false;
    geo_.clear();
    const double ct = std::cos(theta), st = std::sin(theta);
    for (size_t i = 0; i < sensors_.size() && i < readings.size(); ++i) {
      if (!readings[i].valid) continue;
      const auto& sc = sensors_[i];
      const double a = theta + sc.angle;
      geo_.push_back({sc.fwd * ct - sc.left * st, sc.fwd * st + sc.left * ct,
                      std::cos(a), std::sin(a), readings[i].dist, i});
    }
    if (geo_.empty()) return false;

    // Persistent-disagreement check, evaluated at the current estimate.
    bool recover = false;
    for (const auto& g : geo_) {
      const double expected = raycast(estX_ + g.ox, estY_ + g.oy, g.c, g.s);
      const double tol = std::max(cfg_.recoveryResidual, 3.0 * sigmaFor(expected));
      const double residual = std::fabs(g.meas - expected);
      const bool plausible = expected <= cfg_.maxRange && residual <= cfg_.recoveryMaxResidual;
      // Separate streaks for too-long and too-short readings, so one stray short
      // reading (junk, a passing ball) doesn't erase real too-long evidence.
      // A reading that agrees ends both streaks. One that disagrees implausibly
      // (an object far from any wall) is no evidence either way: both unchanged.
      int& longCnt = longCnt_[g.idx];
      int& shortCnt = shortCnt_[g.idx];
      if (residual <= tol) {
        longCnt = 0;
        shortCnt = 0;
      } else if (plausible) {
        if (g.meas > expected) ++longCnt;
        else                   ++shortCnt;
      }
      if (longCnt >= cfg_.recoveryCount || shortCnt >= cfg_.recoveryCountShort) {
        recover = true;
        longCnt = 0;
        shortCnt = 0;
      }
    }

    double sum = 0.0;
    for (auto& p : particles_) {
      double like = 1.0;
      for (const auto& g : geo_) like *= likelihood(g.meas, raycast(p.x + g.ox, p.y + g.oy, g.c, g.s));
      p.w *= like;
      p.scored = true;
      sum += p.w;
    }
    if (!(sum > 1e-300) || !std::isfinite(sum)) {
      const double w = 1.0 / particles_.size();
      for (auto& p : particles_) p.w = w;
    } else {
      for (auto& p : particles_) p.w /= sum;
    }
    double sumSq = 0.0;
    for (const auto& p : particles_) sumSq += p.w * p.w;
    const int nInject = recover ? static_cast<int>(cfg_.recoveryFrac * particles_.size()) : 0;
    if (nInject > 0 || 1.0 / sumSq < cfg_.resampleRatio * particles_.size()) {
      computeEstimate();
      const double cx = estX_, cy = estY_;
      resample();
      std::normal_distribution<double> n(0.0, cfg_.recoverySpread);
      std::uniform_int_distribution<int> pick(0, static_cast<int>(particles_.size()) - 1);
      for (int k = 0; k < nInject; ++k) {
        auto& p = particles_[pick(rng_)];
        p.x = clamp(cx + n(rng_));
        p.y = clamp(cy + n(rng_));
        p.scored = false;
      }
    }
    computeEstimate();
    return true;
  }

  Pose estimate(double theta) const { return {estX_, estY_, theta}; }
  const std::vector<Particle>& particles() const { return particles_; }
  const MCLConfig& config() const { return cfg_; }

  // Distance from (ox, oy) along direction (c, s) to the field perimeter.
  double raycast(double ox, double oy, double c, double s) const {
    const double W = cfg_.fieldHalf;
    double t = 1e9;
    if (c > 1e-9)       t = std::min(t, ( W - ox) / c);
    else if (c < -1e-9) t = std::min(t, (-W - ox) / c);
    if (s > 1e-9)       t = std::min(t, ( W - oy) / s);
    else if (s < -1e-9) t = std::min(t, (-W - oy) / s);
    return std::max(0.0, t);
  }

 private:
  struct Geo { double ox, oy, c, s, meas; size_t idx; };

  double sigmaFor(double expected) const {
    return std::max(cfg_.sensorSigmaMin, cfg_.sensorSigmaFrac * expected);
  }

  double likelihood(double measured, double expected) const {
    const double sigma = sigmaFor(expected);
    const double z = (measured - expected) / sigma;
    const double gauss = std::exp(-0.5 * z * z) / (sigma * std::sqrt(2.0 * kPi));
    return (1.0 - cfg_.outlierProb) * gauss + cfg_.outlierProb / cfg_.maxRange;
  }

  void resample() {  // low-variance (systematic) resampling
    const int N = static_cast<int>(particles_.size());
    out_.clear();
    out_.reserve(N);
    std::uniform_real_distribution<double> u(0.0, 1.0 / N);
    std::normal_distribution<double> jitter(0.0, cfg_.resampleJitter);
    const double r = u(rng_);
    double cum = particles_[0].w;
    int i = 0;
    for (int m = 0; m < N; ++m) {
      const double U = r + static_cast<double>(m) / N;
      while (U > cum && i < N - 1) cum += particles_[++i].w;
      Particle p = particles_[i];
      p.x = clamp(p.x + jitter(rng_));
      p.y = clamp(p.y + jitter(rng_));
      p.w = 1.0 / N;
      out_.push_back(p);
    }
    particles_.swap(out_);
  }

  void computeEstimate() {
    double x = 0, y = 0, wsum = 0;
    for (const auto& p : particles_) {
      if (!p.scored) continue;  // don't let unverified guesses move the estimate
      x += p.w * p.x; y += p.w * p.y; wsum += p.w;
    }
    if (wsum > 0) { estX_ = x / wsum; estY_ = y / wsum; }
  }

  double clamp(double v) const { return std::clamp(v, -cfg_.fieldHalf, cfg_.fieldHalf); }

  MCLConfig cfg_;
  std::vector<DistanceSensorConfig> sensors_;
  std::vector<int> longCnt_, shortCnt_;  // per-sensor disagreement streaks
  std::vector<Particle> particles_, out_;
  std::vector<Geo> geo_;
  std::mt19937 rng_;
  double estX_ = 0, estY_ = 0;

};

}  // namespace loc