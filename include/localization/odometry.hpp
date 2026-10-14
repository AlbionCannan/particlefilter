// odometry.hpp — tracking-wheel odometry math (arc-corrected). Pure C++, no PROS.
#pragma once
#include <cmath>

namespace loc {

struct OdomConfig {
  // How far LEFT of the tracking center the vertical (forward) wheel is.
  // Negative = right of center. 0 if it's on the centerline.
  double vertLeftOffset = 0.0;
  // How far FORWARD of the tracking center the horizontal (sideways) wheel is.
  // Negative = behind center. Ignored if you have no horizontal wheel.
  double horzFwdOffset = 0.0;
};

struct Delta { double dx = 0, dy = 0; };  // field frame, inches

// dVert: forward wheel travel (in, + = robot moved forward)
// dHorz: sideways wheel travel (in, + = robot moved LEFT)
// thetaPrev/thetaNow: heading in radians, CCW positive
inline Delta odomStep(const OdomConfig& c, double dVert, double dHorz,
                      double thetaPrev, double thetaNow) {
  const double dTheta = thetaNow - thetaPrev;
  double fwd, left;
  if (std::fabs(dTheta) < 1e-6) {
    fwd = dVert;
    left = dHorz;
  } else {
    // Each wheel traced an arc; convert to the tracking center's chord.
    const double chord = 2.0 * std::sin(dTheta / 2.0);
    fwd = chord * (dVert / dTheta + c.vertLeftOffset);
    left = chord * (dHorz / dTheta - c.horzFwdOffset);
  }
  // The chord points along the average heading over the step.
  const double m = thetaPrev + dTheta / 2.0;
  const double cm = std::cos(m), sm = std::sin(m);
  return {fwd * cm - left * sm, fwd * sm + left * cm};
}

}  // namespace loc