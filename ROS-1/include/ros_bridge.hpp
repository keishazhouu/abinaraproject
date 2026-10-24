#ifndef ROS_BRIDGE_HPP
#define ROS_BRIDGE_HPP

#include <mujoco/mujoco.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <string>
#include <vector>

class RosBridge {
public:
    RosBridge(ros::NodeHandle& nh, const mjModel* m, mjData* d);
    RosBridge(const RosBridge&) = delete;
    RosBridge& operator=(const RosBridge&) = delete;

    bool addRobot(const std::string& name, const std::string& prefix);

    void publish();

private:
    static constexpr int kNumJoints = 6;
    static constexpr int kNumArmJoints = 4;

    struct Robot {
        std::string name;
        int joint_id[kNumJoints];
        int actuator_id[kNumJoints];
        int site_id;
        int torso_id;
        ros::Subscriber cmd_sub;
        ros::Publisher state_pub;
        ros::Publisher ee_pub;
    };

    void onCommand(size_t idx, const std_msgs::Float64MultiArray::ConstPtr& msg);

    ros::NodeHandle& nh_;
    const mjModel* m_;
    mjData* d_;
    std::vector<Robot> robots_;
};

#endif