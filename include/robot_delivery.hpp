#ifndef ROBOT_DELIVERY_HPP
#define ROBOT_DELIVERY_HPP
#include <string>
#include "robot.hpp"

class robot_delivery : public robot {
    private:
        int delivery_weight;

    public:
        robot_delivery(std::string robot_name, int robot_battery, int weight);
        void weight(int kg);
        int getdelivery_weight();
        void display() override;

};

#endif