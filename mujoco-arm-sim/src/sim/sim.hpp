#ifndef SIM_HPP
#define SIM_HPP

#include <mujoco/mujoco.h>

bool sim_init(const char* path, mjModel** m, mjData** d);

void sim_advance(const mjModel* m, mjData* d);

void sim_reset(const mjModel* m, mjData* d);

void sim_cleanup(mjModel* m, mjData* d);

#endif