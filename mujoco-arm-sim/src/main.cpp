#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;

void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
void glfw_error_callback(int error, const char* description);


int main () {
char error[1000] = "Could not load model";
m = mj_loadXML("models/arm_final.xml", nullptr, error, sizeof(error));
if(!m) {
fprintf(stderr, "%s\n", error);
return 1;
    }

d = mj_makeData(m);

glfwSetErrorCallback(glfw_error_callback);
if (!glfwInit()) {
fprintf(stderr, "Could not initialize GLFW\n");
return 1;
    }

GLFWwindow* window = glfwCreateWindow(800, 600, "Abinara Arm Simulation", nullptr, nullptr);
if (!window) {
fprintf(stderr, "Could not create GLFW window\n");
glfwTerminate();
return 1;
    }

glfwMakeContextCurrent(window);
glfwSetMouseButtonCallback(window, mouse_button_callback);

glfwSetMouseButtonCallback(window, mouse_button_callback);
glfwSetCursorPosCallback(window, cursor_pos_callback);
glfwSetScrollCallback(window, scroll_callback);
glfwSetKeyCallback(window, keyboard_callback);

mjv_defaultCamera(&cam);
mjv_defaultOption(&opt);
mjv_defaultScene(&scn);
mjr_defaultContext(&con);

mjv_makeScene(m, &scn, 2000);
mjr_makeContext(m, &con, mjFONTSCALE_150);

cam.distance = 1.5;
cam.azimuth = 90;
cam.elevation = -20;

while (!glfwWindowShouldClose(window)) {
mj_step(m, d);

printf("Joint positions: ");
for (int i = 0; i < m->nq; i++) {
printf("%.3f ", d->qpos[i]);
        }
printf("\n");

mjrRect viewport = {0, 0, 0, 0};
glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
mjr_render(viewport, &scn, &con);

glfwSwapBuffers(window);
glfwPollEvents();
    }

mjv_freeScene(&scn);
mjr_freeContext(&con);
mj_deleteData(d);
mj_deleteModel(m);
glfwTerminate();

return 0;
} 
