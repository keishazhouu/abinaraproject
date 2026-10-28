#include "inverse_kinematics.hpp"

arm_kin::IKResult InverseKinematics::solve(const arm_kin::Vec3& target,
                                           double tool_pitch) const {
    return arm_kin::inverse(target, tool_pitch);
}
