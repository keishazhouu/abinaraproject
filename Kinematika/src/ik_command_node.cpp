#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <cmath>
#include "inverse_kinematics.hpp"

class IKCommandNode {
public:
    IKCommandNode() : pnh_("~") {
        pnh_.param("target_x", target_.x, 0.375);
        pnh_.param("target_y", target_.y, 0.0);
        pnh_.param("target_z", target_.z, 0.625);
        pnh_.param("tool_pitch", pitch_, 0.75);
        pnh_.param("rate", rate_, 10.0);

        if (rate_ <= 0.0) {
            ROS_WARN("Parameter rate harus > 0; menggunakan 10 Hz.");
            rate_ = 10.0;
        }

        cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_cmd", 10, true);
        target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("ee_target", 10, true);
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &IKCommandNode::onTimer, this);

        const auto result = ik_.solve(target_, pitch_);
        if (!result.reachable || !result.within_limits) {
            ROS_ERROR("Target IK tidak valid / melanggar limit. Tidak mengirim command. "
                      "Coba sesuaikan target_x, target_y, target_z, atau tool_pitch.");
            valid_ = false;
            return;
        }
        solution_ = result.q;
        valid_ = true;
        ROS_INFO("IK valid. Target=(%.3f, %.3f, %.3f) m, pitch=%.1f deg",
                 target_.x, target_.y, target_.z, pitch_ * 180.0 / M_PI);
        ROS_INFO("Joint target (deg): hip=%.2f, shoulder=%.2f, elbow=%.2f, wrist=%.2f",
                 solution_.hip * 180.0 / M_PI, solution_.shoulder * 180.0 / M_PI,
                 solution_.elbow * 180.0 / M_PI, solution_.wrist * 180.0 / M_PI);
    }

private:
    void onTimer(const ros::TimerEvent&) {
        if (!valid_) return;
        std_msgs::Float64MultiArray cmd;
        cmd.data = {solution_.hip, solution_.shoulder, solution_.elbow, solution_.wrist};
        cmd_pub_.publish(cmd);

        geometry_msgs::PointStamped target_msg;
        target_msg.header.stamp = ros::Time::now();
        target_msg.header.frame_id = "arm1_base";
        target_msg.point.x = target_.x;
        target_msg.point.y = target_.y;
        target_msg.point.z = target_.z;
        target_pub_.publish(target_msg);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Publisher cmd_pub_, target_pub_;
    ros::Timer timer_;
    InverseKinematics ik_;
    arm_kin::Vec3 target_{};
    arm_kin::JointAngles solution_{};
    double pitch_{0.0}, rate_{10.0};
    bool valid_{false};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ik_command_node");
    IKCommandNode node;
    ros::spin();
    return 0;
}
