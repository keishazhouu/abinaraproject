#include <iostream>
#include <string>


class robot {
    private:
        std::string name;
        int battery;
    public:
        robot(std::string robot_name, int robot_battery) : name(robot_name), battery (robot_battery) {}

        void display() {
            std::cout << "My name is " << name << " and my battery level is " << battery << std::endl;
        }
};

int main () {
    robot robot1("Abinara", 100);
    robot robot2("Kei", 90);
    robot1.display();
    robot2.display();
    return 0;
}