#include "ros_bridge.hpp"

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>

#include <algorithm>

namespace {
const char* const kJointBase[] = {"hip", "shoulder", "elbow", "wrist", "left", "right"};
}

RosBridge::RosBridge(ros::NodeHandle& nh, const mjModel* m, mjData* d) : nh_(nh), m_(m), d_(d) {
    robots_.reserve(8);
}

bool RosBridge::addRobot(const std::string& name, const std::string& prefix) {
    Robot r;
    r.name = name;
    for (int i = 0; i < kNumJoints; ++i) {
        const std::string full = prefix + kJointBase[i];
        r.joint_id[i] = mj_name2id(m_, mjOBJ_JOINT, full.c_str());
        r.actuator_id[i] = mj_name2id(m_, mjOBJ_ACTUATOR, full.c_str());
        if (r.joint_id[i] < 0 || r.actuator_id[i] < 0) {
            ROS_ERROR("Joint/aktuator '%s' tidak ditemukan di model MuJoCo.", full.c_str());
            return false;
        }
    }
    r.site_id = mj_name2id(m_, mjOBJ_SITE, (prefix + "ee").c_str());
    r.torso_id = mj_name2id(m_, mjOBJ_BODY, (prefix + "torso").c_str());
    
    if (r.site_id < 0 || r.torso_id < 0) {
        ROS_ERROR("Site '%see' atau body '%storso' tidak ditemukan di model.", prefix.c_str(), prefix.c_str());
        return false;
    }

    const size_t idx = robots_.size();
    r.cmd_sub = nh_.subscribe<std_msgs::Float64MultiArray>(
        name + "/joint_cmd", 10,
        [this, idx](const std_msgs::Float64MultiArray::ConstPtr& msg) { onCommand(idx, msg); });
    r.state_pub = nh_.advertise<sensor_msgs::JointState>(name + "/joint_states", 10);
    r.ee_pub = nh_.advertise<geometry_msgs::PointStamped>(name + "/ee_position", 10);
    robots_.push_back(r);

    ROS_INFO("Robot '%s' siap: /%s/{joint_cmd, joint_states, ee_position}", name.c_str(), name.c_str());
    return true;
}

void RosBridge::onCommand(size_t idx, const std_msgs::Float64MultiArray::ConstPtr& msg) {
    const Robot& r = robots_[idx];
    const int n = std::min<int>(static_cast<int>(msg->data.size()), kNumJoints);
    if (n < kNumArmJoints) {
        ROS_WARN_THROTTLE(2.0, "[%s] joint_cmd butuh minimal %d nilai (hip, shoulder, elbow, wrist), diterima %zu.",
                          r.name.c_str(), kNumArmJoints, msg->data.size());
        return;
    }
    for (int i = 0; i < n; ++i) {
        const int a = r.actuator_id[i];
        double v = msg->data[i];
        if (m_->actuator_ctrllimited[a]) {
            v = std::clamp(v, m_->actuator_ctrlrange[2 * a], m_->actuator_ctrlrange[2 * a + 1]);
        }
        d_->ctrl[a] = v;
    }
}

void RosBridge::publish() {
    const ros::Time now = ros::Time::now();
    for (Robot& r : robots_) {
        sensor_msgs::JointState js;
        js.header.stamp = now;
        js.header.frame_id = r.name + "_base";
        for (int i = 0; i < kNumJoints; ++i) {
            js.name.push_back(mj_id2name(m_, mjOBJ_JOINT, r.joint_id[i]));
            js.position.push_back(d_->qpos[m_->jnt_qposadr[r.joint_id[i]]]);
            js.velocity.push_back(d_->qvel[m_->jnt_dofadr[r.joint_id[i]]]);
            js.effort.push_back(d_->actuator_force[r.actuator_id[i]]);
        }
        r.state_pub.publish(js);

        geometry_msgs::PointStamped ee;
        ee.header = js.header;
        ee.point.x = d_->site_xpos[3 * r.site_id + 0] - d_->xpos[3 * r.torso_id + 0];
        ee.point.y = d_->site_xpos[3 * r.site_id + 1] - d_->xpos[3 * r.torso_id + 1];
        ee.point.z = d_->site_xpos[3 * r.site_id + 2];
        r.ee_pub.publish(ee);
    }
}