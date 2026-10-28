#ifndef ARM_KINEMATICS_HPP
#define ARM_KINEMATICS_HPP

namespace arm_kin {

struct Vec3 {
    double x, y, z;
};

struct JointAngles {
    double hip, shoulder, elbow, wrist;
};

struct IKResult {
    JointAngles q;
    bool reachable;
    bool within_limits;
};

constexpr double kShoulderMin = -1.5708, kShoulderMax = 0.6981;
constexpr double kElbowMin = -2.25, kElbowMax = 1.0;
constexpr double kWristMin = -1.5708, kWristMax = 1.5708;

Vec3 forward(const JointAngles& q);

IKResult inverse(const Vec3& p, double tool_pitch);

}

#endif