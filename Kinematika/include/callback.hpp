#ifndef CALLBACK_HPP
#define CALLBACK_HPP

#include <GLFW/glfw3.h>

void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
void glfw_error_callback(int error, const char* description);

#endif