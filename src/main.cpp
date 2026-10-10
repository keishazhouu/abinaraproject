#include <iostream>
#include <string>


class robot {
    private:
        std::string name;
        int battery;
    public:
        robot(std::string robot_name, int robot_battery) {
            name = robot_name;
            battery = robot_battery;
        }
    
        void setName(std::string robot_name) {
            name = robot_name;
        }

        std::string getName() {
            return name;
        }

        void setBattery(int level) {
            if (level >= 0 && level <= 100) {
                battery = level;
            }
            else {
                std::cout << "Battery level must be between 0 and 100." << std::endl;
            }
        }

        int getBattery() {
            return battery;
        }


        void display() {
            std::cout << "My name is " << name << " and my battery level is " << battery << std::endl;
        }
};

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