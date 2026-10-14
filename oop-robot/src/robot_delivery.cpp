#include <iostream>
#include "robot_delivery.hpp"

robot_delivery::robot_delivery(std::string robot_name, int robot_battery, int weight) : robot(robot_name, robot_battery) {
    delivery_weight = weight;
}

void robot_delivery::weight(int kg) {
    delivery_weight = delivery_weight + kg; //add some weight to the current weight
}

int robot_delivery::getdelivery_weight() {
    return delivery_weight;
}

void robot_delivery::display() {
    robot::display();
    std::cout << "" << getName() << " is carrying " << delivery_weight << " kg of weight" << std::endl;
}