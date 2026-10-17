#include <mujoco/mujoco.h>

void control_robot(const mjModel* m, mjData* d) {
    if (m == nullptr || d == nullptr) {
        return;
    }

    if (m->nu == 0) {
        return;
    }
}