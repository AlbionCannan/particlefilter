// Laptop test for the localization math. NOT compiled by PROS (it's outside src/).
//   g++ -std=c++20 -O2 -I include test/mcl_sim.cpp -o sim && ./sim
//   ./sim 7        # run with a different random seed
#include <cstdio>
#include <cstdlib>
#include <random>

#include "localization/mcl.hpp"
#include "localization/odometry.hpp"
using namespace loc;

static int fails = 0;
static void check(bool ok, const char* what) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
  fails += !ok;
}

struct Result { double odoFinal, mclFinal, worstOutsideShove, errAfterRecovery; };

// Drives a 60" square twice at 40 in/s with 3% odometry scale error, noisy
// sensors, 10% garbage readings, and one shove that odometry doesn't see.
static Result runSquare(unsigned seed, int shoveSide, int shoveK, double sdx, double sdy,
                        int recoverWithinSteps) {
  MCLConfig cfg;
  std::vector<DistanceSensorConfig> sensors = {
      {0.0, 6.5, kPi / 2}, {0.0, -6.5, -kPi / 2}, {-7.0, 0.0, kPi}};
  MCL mcl(cfg, sensors, seed);
  std::mt19937 rng(seed + 1);
  std::normal_distribution<double> n01(0, 1);
  std::uniform_real_distribution<double> u01(0, 1);

  double tx = -30, ty = -30, th = 0, ox = tx, oy = ty;
  mcl.init(tx + 3, ty - 3, 3.0);  // start deliberately 4.2" off
  const double dt = 0.01, speed = 40;
  Result res{0, 0, 0, 1e9};
  int step = 0, shoveStep = -1;
  for (int side = 0; side < 8; ++side, th += kPi / 2) {
    for (int k = 0; k < 150; ++k, ++step) {
      const double dx = speed * dt * std::cos(th), dy = speed * dt * std::sin(th);
      tx += dx; ty += dy;
      const double mdx = dx * 1.03 + 0.01 * n01(rng), mdy = dy * 1.03 + 0.01 * n01(rng);
      ox += mdx; oy += mdy;
      mcl.predict(mdx, mdy);
      if (side == shoveSide && k == shoveK) { tx += sdx; ty += sdy; shoveStep = step; }
      if (step % 3 == 0) {
        std::vector<Reading> r;
        for (auto& s : sensors) {
          const double sx = tx + s.fwd * std::cos(th) - s.left * std::sin(th);
          const double sy = ty + s.fwd * std::sin(th) + s.left * std::cos(th);
          const double a = th + s.angle;
          const double truth = mcl.raycast(sx, sy, std::cos(a), std::sin(a));
          if (truth > cfg.maxRange) { r.push_back({false, 0}); continue; }
          const double meas = u01(rng) < 0.10 ? u01(rng) * truth  // blocked by an object
                                              : truth + n01(rng) * (0.3 + 0.01 * truth);
          r.push_back({true, meas});
        }
        mcl.update(r, th);
      }
      const Pose e = mcl.estimate(th);
      const double err = std::hypot(e.x - tx, e.y - ty);
      const bool inShove = shoveStep >= 0 && step < shoveStep + recoverWithinSteps;
      if (step > 300 && !inShove) res.worstOutsideShove = std::max(res.worstOutsideShove, err);
      if (shoveStep >= 0 && step == shoveStep + recoverWithinSteps) res.errAfterRecovery = err;
    }
  }
  const Pose e = mcl.estimate(th);
  res.mclFinal = std::hypot(e.x - tx, e.y - ty);
  res.odoFinal = std::hypot(ox - tx, oy - ty);
  return res;
}

int main(int argc, char** argv) {
  const unsigned seed = argc > 1 ? std::atoi(argv[1]) : 42;

  {  // Pure rotation with offset wheels must produce zero translation.
    const OdomConfig oc{3.0, 2.0};
    const double dth = 0.5;
    const Delta d = odomStep(oc, -3.0 * dth, 2.0 * dth, 0.0, dth);
    check(std::fabs(d.dx) < 1e-9 && std::fabs(d.dy) < 1e-9, "odometry: pure rotation -> no translation");
  }
  {  // CCW quarter circle of radius 10 from heading 0 ends at (10, 10).
    const OdomConfig oc{0.0, 0.0};
    double x = 0, y = 0, th = 0;
    for (int i = 0; i < 100; ++i) {
      const double nth = th + (kPi / 2) / 100;
      const Delta d = odomStep(oc, 10.0 * (kPi / 2) / 100, 0.0, th, nth);
      x += d.dx; y += d.dy; th = nth;
    }
    check(std::fabs(x - 10) < 1e-6 && std::fabs(y - 10) < 1e-6, "odometry: quarter arc ends at (10, 10)");
  }
  {  // 4" shove during a leg where the error is small relative to sensor noise:
     // corrected gradually; must be under 2.5" within 2.5 s and never spike elsewhere.
    const Result r = runSquare(seed, 3, 75, 0.0, 4.0, 250);
    std::printf("       small shove: odometry final %.2f in, MCL final %.2f in, "
                "worst outside shove %.2f in, 2.5 s after shove %.2f in\n",
                r.odoFinal, r.mclFinal, r.worstOutsideShove, r.errAfterRecovery);
    check(r.mclFinal < 2.0 && r.mclFinal < 0.6 * r.odoFinal, "MCL final error < 2\" and well below odometry");
    check(r.worstOutsideShove < 2.5, "no error spikes outside the shove");
    check(r.errAfterRecovery < 2.5, "small shove corrected within 2.5 s");
  }
  {  // 8" sideways shove on a leg where side sensors see the walls clearly:
     // the persistent-disagreement detector should fix it fast.
    // Typical recovery is ~150 ms; the 1.5 s bound covers the slow tail.
    const Result r = runSquare(seed, 1, 75, 8.0, 0.0, 150);
    std::printf("       big shove: 1.5 s after shove error %.2f in, worst outside %.2f in\n",
                r.errAfterRecovery, r.worstOutsideShove);
    check(r.errAfterRecovery < 2.0, "8\" shove recovered within 1.5 s");
    check(r.worstOutsideShove < 2.5, "no error spikes outside the shove");
  }
  return fails;
}