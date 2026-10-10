#ifndef ROBOT_SAR_HPP
#define ROBOT_SAR_HPP
#include <string>
#include "robot.hpp"

class robot_sar : public robot {
    private:
        int rescue_count; //new attribute for the robot_sar

    public:
        robot_sar(std::string robot_name, int robot_battery, int rescue);
        void rescue();
        int getrescue_count();
        void display() override; 

};

#endif