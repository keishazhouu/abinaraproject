#include <iostream>
#include "robot_sar.hpp"

robot_sar::robot_sar(std::string robot_name, int robot_battery, int rescue) : robot(robot_name, robot_battery) {
    rescue_count = rescue;
}

void robot_sar::rescue() {
    rescue_count++;
}

int robot_sar::getrescue_count() {
    return rescue_count;
}

void robot_sar::display() {
    robot::display();
    std::cout << "" << robot::getName() << " have rescued " << rescue_count << " people" << std::endl;
}
