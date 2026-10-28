#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include "arm_kinematics.hpp"

class InverseKinematics {
public:
    arm_kin::IKResult solve(const arm_kin::Vec3& target, double tool_pitch) const;
};

#endif
