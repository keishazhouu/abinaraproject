#include "sim.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>

static std::chrono::steady_clock::time_point wall_start = std::chrono::steady_clock::now();

bool sim_init(const char* path, mjModel** m, mjData** d) {
    char error[1000] = "";
    *m = mj_loadXML(path, nullptr, error, sizeof(error));
    if (!*m) {
        fprintf(stderr, "Gagal load model: %s\n", error);
        return false;
    }
    *d = mj_makeData(*m);
    wall_start = std::chrono::steady_clock::now();
    return true;
}

void sim_advance(const mjModel* m, mjData* d) {
    double wall = std::chrono::duration<double>(
                      std::chrono::steady_clock::now() - wall_start).count();
    double target = std::min(wall, d->time + 0.05);
    while (d->time < target) mj_step(m, d);
}

void sim_reset(const mjModel* m, mjData* d) {
    mj_resetData(m, d);
    mj_forward(m, d);
    wall_start = std::chrono::steady_clock::now();
}

void sim_cleanup(mjModel* m, mjData* d) {
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
}