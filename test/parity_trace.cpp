// Records the REAL C++ filter's outputs so test/parity_check.js can confirm the
// browser simulator's JavaScript port computes identical numbers.
//   g++ -std=c++20 -O2 -I include test/parity_trace.cpp -o parity_trace
//   ./parity_trace [seed] > trace.json && node test/parity_check.js trace.json [seed]
#include <cstdio>
#include <cstdlib>
#include <random>

#include "localization/mcl.hpp"
#include "localization/odometry.hpp"
using namespace loc;

static double checksum(const MCL& m) {  // detects any particle-level difference
  double c = 0;
  for (const auto& p : m.particles()) c += p.x * 1.0001 + p.y * 2.0003 + p.w * 1000.0 + (p.scored ? 0.5 : 0);
  return c;
}

int main(int argc, char** argv) {
  const unsigned seed = argc > 1 ? static_cast<unsigned>(std::atoi(argv[1])) : 2024;
  std::printf("{\n");
  // 1. Raw generator output.
  {
    std::mt19937 g(12345);
    std::printf("\"mt\":[");
    for (int i = 0; i < 2000; ++i) std::printf("%s%u", i ? "," : "", static_cast<unsigned>(g()));
    std::printf("],\n");
  }
  // 2. odomStep on varied inputs (including the no-turn branch).
  {
    std::mt19937 g(99);
    std::uniform_real_distribution<double> u(-1, 1);
    std::printf("\"odom\":[");
    for (int i = 0; i < 500; ++i) {
      OdomConfig c{u(g) * 5, u(g) * 5};
      double dv = u(g) * 2, dh = u(g) * 2, t0 = u(g) * 7, t1 = (i % 5 == 0) ? t0 : t0 + u(g) * 0.3;
      Delta d = odomStep(c, dv, dh, t0, t1);
      std::printf("%s[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]", i ? "," : "",
                  c.vertLeftOffset, c.horzFwdOffset, dv, dh, t0, t1, d.dx, d.dy);
    }
    std::printf("],\n");
  }
  // 3. Full filter run: square with drift, garbage readings, two shoves.
  MCLConfig cfg;
  std::vector<DistanceSensorConfig> sensors = {
      {0.0, 6.5, kPi / 2}, {0.0, -6.5, -kPi / 2}, {-7.0, 0.0, kPi}};
  MCL mcl(cfg, sensors, seed);
  std::mt19937 world(seed + 7);
  std::normal_distribution<double> n01(0, 1);
  std::uniform_real_distribution<double> u01(0, 1);
  double tx = -30, ty = -30, th = 0;
  std::printf("\"ops\":[\n");
  mcl.init(tx + 3, ty - 3, 3.0);
  std::printf("{\"op\":\"init\",\"x\":%.17g,\"y\":%.17g,\"s\":3,\"est\":[%.17g,%.17g],\"ck\":%.17g}",
              tx + 3, ty - 3, mcl.estimate(0).x, mcl.estimate(0).y, checksum(mcl));
  int step = 0, recoveries = 0;
  for (int side = 0; side < 8; ++side, th += kPi / 2) {
    for (int k = 0; k < 150; ++k, ++step) {
      const double dx = 0.4 * std::cos(th), dy = 0.4 * std::sin(th);
      tx += dx; ty += dy;
      const double mdx = dx * 1.03 + 0.01 * n01(world), mdy = dy * 1.03 + 0.01 * n01(world);
      mcl.predict(mdx, mdy);
      std::printf(",\n{\"op\":\"predict\",\"dx\":%.17g,\"dy\":%.17g,\"est\":[%.17g,%.17g],\"ck\":%.17g}",
                  mdx, mdy, mcl.estimate(0).x, mcl.estimate(0).y, checksum(mcl));
      if (side == 1 && k == 75) tx += 8.0;
      if (side == 3 && k == 75) ty += 4.0;
      if (step % 3 == 0) {
        std::vector<Reading> r;
        for (auto& s : sensors) {
          const double sx = tx + s.fwd * std::cos(th) - s.left * std::sin(th);
          const double sy = ty + s.fwd * std::sin(th) + s.left * std::cos(th);
          const double a = th + s.angle;
          const double truth = mcl.raycast(sx, sy, std::cos(a), std::sin(a));
          if (truth > cfg.maxRange) { r.push_back({false, 0}); continue; }
          r.push_back({true, u01(world) < 0.10 ? u01(world) * truth : truth + n01(world) * (0.3 + 0.01 * truth)});
        }
        // count recoveries by detecting particles that became unscored
        mcl.update(r, th);
        bool injected = false;
        for (auto& p : mcl.particles()) if (!p.scored) { injected = true; break; }
        recoveries += injected;
        // After a resample every weight is exactly 1/N; otherwise they differ.
        bool resampled = true;
        for (auto& p : mcl.particles()) if (p.w != 1.0 / mcl.particles().size()) { resampled = false; break; }
        std::printf(",\n{\"op\":\"update\",\"rs\":%d,\"inj\":%d,\"th\":%.17g,\"r\":[", resampled ? 1 : 0, injected ? 1 : 0, th);
        for (size_t i = 0; i < r.size(); ++i) std::printf("%s[%d,%.17g]", i ? "," : "", r[i].valid ? 1 : 0, r[i].dist);
        std::printf("],\"est\":[%.17g,%.17g],\"ck\":%.17g}", mcl.estimate(0).x, mcl.estimate(0).y, checksum(mcl));
      }
    }
  }
  std::printf("\n],\n\"recoveries\":%d\n}\n", recoveries);
  return 0;
}