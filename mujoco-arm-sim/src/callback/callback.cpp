#include <cstdio>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

extern mjModel* m;
extern mjData* d;
extern mjvCamera cam;
extern mjvScene scn;

bool button_left = false;
bool button_middle = false;
bool button_right = false;

double lastx = 0;
double lasty = 0;

void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act != GLFW_PRESS) return;

    double step = 0.1;
    switch (key) {
        case GLFW_KEY_Q: d->ctrl[0] += step; printf("q key pressed"); break;
        case GLFW_KEY_A: d->ctrl[0] -= step; printf("a key pressed"); break;
        case GLFW_KEY_W: d->ctrl[1] += step; printf("w key pressed"); break;
        case GLFW_KEY_S: d->ctrl[1] -= step; printf("s key pressed"); break;
        case GLFW_KEY_E: d->ctrl[2] += step; printf("e key pressed"); break;
        case GLFW_KEY_D: d->ctrl[2] -= step; printf("d key pressed"); break;
        case GLFW_KEY_R: d->ctrl[3] += step; printf("r key pressed"); break;
        case GLFW_KEY_F: d->ctrl[3] -= step; printf("f key pressed"); break;
        case GLFW_KEY_SPACE:
            mj_resetData(m, d);
            break;
    }
}
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}


void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos){
double dx = xpos - lastx;
double dy = ypos - lasty;
int width, height;
glfwGetWindowSize(window, &width, &height);

if (button_left) {
    mjv_moveCamera(m, mjMOUSE_ROTATE_V, dx / height, dy / height, &scn, &cam);
}
else if (button_right) {
    mjv_moveCamera(m, mjMOUSE_MOVE_V, dx / height, dy / height, &scn, &cam);
}
else if (button_middle) {
    mjv_moveCamera(m, mjMOUSE_MOVE_H, dx / height, dy / height, &scn, &cam);
}

    lastx = xpos;
    lasty = ypos;
}

void mouse_button_callback(GLFWwindow* window, int button, int act, int mods) {
    button_left     = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_middle   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right    = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);
}