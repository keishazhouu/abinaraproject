#include <iostream>
#include "robot.hpp"

robot::robot(std::string robot_name, int robot_battery) {
    name = robot_name;
    battery = robot_battery;
}

void robot::setName(std::string robot_name) {
    name = robot_name;
}

std::string robot::getName() {
    return name;
}

void robot::setBattery(int level) {
    if (level >= 0 && level <= 100) {
        battery = level;
    }
    else {
        std::cout << "Battery level must be between 0 and 100." << std::endl;
    }
}

int robot::getBattery() {
    return battery;
}

void robot::display() {
    std::cout << "My name is " << name << " and my battery level is " << battery << std::endl;
}

robot::~robot() {}