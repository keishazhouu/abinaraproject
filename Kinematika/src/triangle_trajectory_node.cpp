#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "arm_kinematics.hpp"

using arm_kin::Vec3;

class TriangleTrajectory {
public:
    TriangleTrajectory() : pnh_("~") {
        double cx, cy, cz, size, rate;
        std::string plane;
        pnh_.param("center_x", cx, 0.33);
        pnh_.param("center_y", cy, 0.0);
        pnh_.param("center_z", cz, 0.30);
        pnh_.param("size", size, 0.08);
        pnh_.param("plane", plane, std::string("xz"));
        pnh_.param("period", period_, 6.0);
        pnh_.param("tool_pitch", tool_pitch_, M_PI);
        pnh_.param("phase_offset", offset_, 0.0);
        pnh_.param("use_external_phase", use_external_phase_, false);
        pnh_.param("rate", rate, 100.0);

        Vec3 u{1, 0, 0}, v{0, 0, 1};
        if (plane == "xy") v = {0, 1, 0};
        else if (plane == "yz") u = {0, 1, 0};
        else if (plane != "xz") ROS_WARN("plane '%s' tidak dikenal, dipakai 'xz'.", plane.c_str());
        for (int k = 0; k < 3; ++k) {
            const double a = M_PI / 2.0 + k * 2.0 * M_PI / 3.0;
            const double cu = size * std::cos(a), sv = size * std::sin(a);
            vertex_[k] = {cx + cu * u.x + sv * v.x, cy + cu * u.y + sv * v.y, cz + cu * u.z + sv * v.z};
        }
        checkWorkspace();

        cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_cmd", 10);
        target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("ee_target", 10);
        enable_sub_ = nh_.subscribe("enable", 1, &TriangleTrajectory::onEnable, this);
        if (use_external_phase_) phase_sub_ = nh_.subscribe("/sync/phase", 1, &TriangleTrajectory::onPhase, this);

        t0_ = ros::Time::now();
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &TriangleTrajectory::onTimer, this);
        ROS_INFO("[%s] segitiga %s, size=%.2f m, period=%.1f s, external_phase=%d", ros::this_node::getNamespace().c_str(),
                 plane.c_str(), size, period_, use_external_phase_);
    }

private:
    Vec3 trianglePoint(double phase) const {
        const double s = phase * 3.0;
        const int i = std::min(static_cast<int>(std::floor(s)), 2);
        const double t = s - i;
        const Vec3 &a = vertex_[i], &b = vertex_[(i + 1) % 3];
        return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    }

    void checkWorkspace() const {
        int bad = 0;
        for (int i = 0; i < 90; ++i) {
            const arm_kin::IKResult r = arm_kin::inverse(trianglePoint(i / 90.0), tool_pitch_);
            if (!r.reachable || !r.within_limits) ++bad;
        }
        if (bad > 0) {
            ROS_ERROR("%d/90 titik lintasan di luar workspace/limit joint. Perkecil size atau geser center.", bad);
        } else {
            ROS_INFO("Cek workspace: seluruh lintasan dapat dijangkau (IK valid, di dalam limit joint).");
        }
    }

    void onEnable(const std_msgs::Bool::ConstPtr& msg) { enabled_ = msg->data; }
    void onPhase(const std_msgs::Float64::ConstPtr& msg) {
        ext_phase_ = msg->data;
        have_phase_ = true;
    }

    void onTimer(const ros::TimerEvent&) {
        if (!enabled_) return;
        double phase;
        if (use_external_phase_) {
            if (!have_phase_) return;
            phase = ext_phase_;
        } else {
            phase = (ros::Time::now() - t0_).toSec() / period_;
        }
        phase += offset_;
        phase -= std::floor(phase);

        const Vec3 p = trianglePoint(phase);
        const arm_kin::IKResult r = arm_kin::inverse(p, tool_pitch_);
        if (!r.reachable || !r.within_limits) ROS_WARN_THROTTLE(2.0, "Target di luar workspace/limit joint.");

        std_msgs::Float64MultiArray cmd;
        cmd.data = {r.q.hip, r.q.shoulder, r.q.elbow, r.q.wrist};
        cmd_pub_.publish(cmd);

        geometry_msgs::PointStamped tgt;
        tgt.header.stamp = ros::Time::now();
        tgt.point.x = p.x;
        tgt.point.y = p.y;
        tgt.point.z = p.z;
        target_pub_.publish(tgt);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Publisher cmd_pub_, target_pub_;
    ros::Subscriber enable_sub_, phase_sub_;
    ros::Timer timer_;
    ros::Time t0_;
    double period_, tool_pitch_, offset_;
    bool use_external_phase_{false}, enabled_{true}, have_phase_{false};
    double ext_phase_{0.0};
    std::array<Vec3, 3> vertex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "triangle_trajectory_node");
    TriangleTrajectory node;
    ros::spin();
    return 0;
}