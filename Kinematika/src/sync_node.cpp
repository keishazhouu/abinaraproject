#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh, pnh("~");
    double period = 6.0, rate = 100.0;
    pnh.param("period", period, 6.0);
    pnh.param("rate", rate, 100.0);

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/sync/phase", 10);
    const ros::Time t0 = ros::Time::now();
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / rate), [&](const ros::TimerEvent&) {
        const double t = (ros::Time::now() - t0).toSec() / period;
        std_msgs::Float64 msg;
        msg.data = t - std::floor(t);
        pub.publish(msg);
    });
    ROS_INFO("sync_node: /sync/phase, period=%.1f s", period);
    ros::spin();
    return 0;
}