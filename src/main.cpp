#include <iostream>
#include "robot.hpp"

int main () {
    robot robot1 ("Abinara", 100);
    robot robot2 ("Keii", 80);

    robot1.display();
    robot2.display();

    robot1.setBattery(120); //Invalid battery level, error message will be printed out
    robot1.setBattery(90); //valid battery level, battery level will be updated to 90
    robot1.display(); //printed out the updated one

    std::cout << robot1.getName() << "'s battery level is " << robot1.getBattery() << std::endl;

    return 0;
}