#include "arm_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace arm_kin {
namespace {

constexpr double kShoulderZ = 0.1 + 0.375;
constexpr double kElbowX = -0.0175;
constexpr double kElbowZ = 0.275;
constexpr double kL2 = 0.2;
constexpr double kTool = 0.08;
constexpr double kShoulderOffset = 1.0;
constexpr double kElbowOffset = 1.15;

const double kL1 = std::hypot(kElbowX, kElbowZ);
const double kPhi0 = std::atan2(kElbowX, kElbowZ);

}

Vec3 forward(const JointAngles& q) {
    const double th1 = kShoulderOffset + q.shoulder;
    const double th2 = kShoulderOffset + kElbowOffset + q.shoulder + q.elbow;
    const double th3 = th2 + q.wrist;
    const double r = kL1 * std::sin(th1 + kPhi0) + kL2 * std::sin(th2) + kTool * std::sin(th3);
    const double z = kShoulderZ + kL1 * std::cos(th1 + kPhi0) + kL2 * std::cos(th2) + kTool * std::cos(th3);
    return {r * std::cos(q.hip), r * std::sin(q.hip), z};
}

IKResult inverse(const Vec3& p, double tool_pitch) {
    IKResult out{};

    out.q.hip = std::atan2(p.y, p.x);
    const double r = std::hypot(p.x, p.y);

    const double wr = r - kTool * std::sin(tool_pitch);
    const double wz = p.z - kTool * std::cos(tool_pitch) - kShoulderZ;

    const double c = (wr * wr + wz * wz - kL1 * kL1 - kL2 * kL2) / (2.0 * kL1 * kL2);
    out.reachable = std::fabs(c) <= 1.0;
    const double delta = std::acos(std::clamp(c, -1.0, 1.0));
    const double t1 = std::atan2(wz, wr) - std::atan2(-kL2 * std::sin(delta), kL1 + kL2 * std::cos(delta));
    const double alpha = M_PI / 2.0 - t1;
    const double beta = alpha + delta;

    out.q.shoulder = alpha - kPhi0 - kShoulderOffset;
    out.q.elbow = beta - kShoulderOffset - kElbowOffset - out.q.shoulder;
    out.q.wrist = tool_pitch - beta;

    out.within_limits = out.q.shoulder >= kShoulderMin && out.q.shoulder <= kShoulderMax &&
                        out.q.elbow >= kElbowMin && out.q.elbow <= kElbowMax &&
                        out.q.wrist >= kWristMin && out.q.wrist <= kWristMax;
    return out;
}

}