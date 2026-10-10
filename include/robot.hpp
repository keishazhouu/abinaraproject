#ifndef ROBOT_HPP
#define ROBOT_HPP
#include <string.h>

class robot {
    private:
        std::string name;
        int battery;
    public:
        robot(std::string robot_name, int robot_battery);
        void setName(std::string robot_name);
        std::string getName();
        void setBattery (int level);
        int getBattery();
        void display();
};

#endif