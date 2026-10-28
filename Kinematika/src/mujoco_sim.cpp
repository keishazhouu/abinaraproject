#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <ros/ros.h>

#include <cstdio>
#include <string>
#include <vector>

#include "callback.hpp"
#include "control.hpp"
#include "ros_bridge.hpp"
#include "sim.hpp"

mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;

int main(int argc, char** argv) {
    ros::init(argc, argv, "mujoco_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string model_path;
    if (!pnh.getParam("model_path", model_path)) {
        ROS_FATAL("Parameter ~model_path wajib diisi.");
        return 1;
    }
    std::vector<std::string> robots;
    pnh.param<std::vector<std::string>>("robots", robots, std::vector<std::string>{"arm1"});
    bool use_prefix = false, headless = false, gravity_comp = true, verbose = false;
    double cam_distance = 1.5;
    pnh.param("use_prefix", use_prefix, false);
    pnh.param("headless", headless, false);
    pnh.param("gravity_comp", gravity_comp, true);
    pnh.param("print_feedback", verbose, false);
    pnh.param("cam_distance", cam_distance, 1.5);

    if (!use_prefix && robots.size() != 1) {
        ROS_FATAL("Model tanpa prefix hanya bisa untuk 1 robot. Set ~use_prefix:=true untuk multi-robot.");
        return 1;
    }

    if (!sim_init(model_path.c_str(), &m, &d)) return 1;
    if (gravity_comp) mjcb_control = control_gravity_compensation;

    RosBridge bridge(nh, m, d);
    for (const std::string& name : robots) {
        if (!bridge.addRobot(name, use_prefix ? name + "_" : "")) return 1;
    }

    GLFWwindow* window = nullptr;
    if (!headless) {
        glfwSetErrorCallback(glfw_error_callback);
        if (!glfwInit()) {
            ROS_FATAL("glfwInit() gagal. Jalankan dengan headless:=true bila tidak ada display.");
            return 1;
        }
        window = glfwCreateWindow(1200, 900, "Arm 4DOF Sim (ROS)", nullptr, nullptr);
        if (!window) {
            ROS_FATAL("glfwCreateWindow() gagal");
            glfwTerminate();
            return 1;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        glfwSetMouseButtonCallback(window, mouse_button_callback);
        glfwSetCursorPosCallback(window, mouse_move_callback);
        glfwSetScrollCallback(window, scroll_callback);
        glfwSetKeyCallback(window, keyboard_callback);

        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        cam.distance = cam_distance;
        cam.azimuth = 90;
        cam.elevation = -20;
    }

    ros::Rate rate(100);
    double last_print = 0;
    while (ros::ok() && !(window && glfwWindowShouldClose(window))) {
        ros::spinOnce();
        sim_advance(m, d);
        bridge.publish();

        if (verbose && ros::Time::now().toSec() - last_print > 0.2) {
            print_feedback(m, d);
            last_print = ros::Time::now().toSec();
        }

        if (window) {
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();
        } else {
            rate.sleep();
        }
    }

    if (window) {
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        glfwTerminate();
    }
    mjcb_control = nullptr;
    sim_cleanup(m, d);
    return 0;
}