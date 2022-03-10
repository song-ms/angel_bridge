#ifndef FAKE_ETH_BRIDGE_H_
#define FAKE_ETH_BRIDGE_H_

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"



const std::string& node_name = "fake_eth_bridge";

const std::string& inner_voice_topic_name = "inner_voice_topic";
const std::string& joint_state_topic_name = "joint_state_topic";
const std::string& cartesian_state_topic_name = "cartesian_state_topic";
const std::string& joint_command_topic_name = "joint_command_topic";
const std::string& cartesian_command_topic_name = "cartesian_command_topic";


const std::vector<std::string>& joint_names = {"test"};//{"left_hip", "left_knee", "right_hip", "right_knee"};
const std::vector<std::string>& part_names = {"left", "right"};

const int n_joint_axis = (int)joint_names.size();
const int n_cartesian_axis = 3;

typedef std_msgs::msg::String StringMsg;
typedef trajectory_msgs::msg::JointTrajectory JointMsg;
typedef trajectory_msgs::msg::JointTrajectory CartesianMsg;

typedef enum {
    Position,
    Velocity,
    Effort,
}ControlTarget;

typedef enum {
    SetPoint,
    Trajectory,
}ControlMode;


class FakeEthernetBridge : public rclcpp::Node
{
public:
    FakeEthernetBridge();

private:
    void TimerCallback();
    void JointCommandCallback(const JointMsg::ConstSharedPtr cmd);
    void CartesianCommandCallback(const CartesianMsg::ConstSharedPtr cmd);
    
    void TrajectoryMsgInitializer(trajectory_msgs::msg::JointTrajectory& msg, const std::vector<std::string>& joint_names);
    void UpdateJointDynamics();
    void UpdateKinematics();
    
    size_t count_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    ControlTarget control_target_;
    ControlMode control_mode_;

    rclcpp::Publisher<StringMsg>::SharedPtr inner_voice_pub_;

    rclcpp::Publisher<JointMsg>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<CartesianMsg>::SharedPtr cartesian_state_pub_;

    rclcpp::Subscription<JointMsg>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<CartesianMsg>::SharedPtr cartesian_command_sub_;

    JointMsg joint_state_;
    JointMsg joint_command_;

    CartesianMsg cartesian_state_;
    CartesianMsg cartesian_command_;

};


#endif // FAKE_ETH_BRIDGE_H_