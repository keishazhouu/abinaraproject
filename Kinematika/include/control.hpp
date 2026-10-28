#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <mujoco/mujoco.h>

double get_joint_position(const mjModel* m, const mjData* d, const char* name);

void print_feedback(const mjModel* m, const mjData* d);

void control_gravity_compensation(const mjModel* m, mjData* d);

#endif