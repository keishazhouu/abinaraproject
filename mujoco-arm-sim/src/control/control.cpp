#include "control/control.hpp"
#include <cstdio>

static const char* JOINT_NAMES[] = {"hip", "shoulder", "elbow", "wrist", "left", "right"};
static const int NUM_JOINTS = 6;

double get_joint_position(const mjModel* m, const mjData* d, const char* name) {
    int sensor = mj_name2id(m, mjOBJ_SENSOR, name);
    if (sensor >= 0) return d->sensordata[m->sensor_adr[sensor]];

    int jnt = mj_name2id(m, mjOBJ_JOINT, name);
    if (jnt >= 0) return d->qpos[m->jnt_qposadr[jnt]];
    return 0.0;
}

void print_feedback(const mjModel* m, const mjData* d) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        printf("%s=%.3f  ", JOINT_NAMES[i], get_joint_position(m, d, JOINT_NAMES[i]));
    }
    printf("\n");
}