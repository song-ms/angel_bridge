#ifndef ETH_BRIDGE_H_
#define ETH_BRIDGE_H_

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <bitset>

#include <cerrno>
#include <cstring>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "eth_packet/eth_packet.hpp"


typedef std_msgs::msg::String StringMsg;
typedef std_msgs::msg::UInt16MultiArray Uint16ArrMsg;
typedef trajectory_msgs::msg::JointTrajectory TrajMsg;
typedef sensor_msgs::msg::BatteryState BatteryMsg;
typedef std_srvs::srv::SetBool SetBoolSrv;


static const std::string& node_name = "eth_bridge";

namespace topic_name{
static const std::string& inner_voice = "inner_voice_topic";
static const std::string& cm_status = "cm_status_topic";
static const std::string& cm_command = "cm_command_topic";
static const std::string& joint_state = "joint_state_topic";
static const std::string& cartesian_state = "cartesian_state_topic";
static const std::string& joint_target = "joint_target_topic";
static const std::string& cartesian_target = "cartesian_target_topic";
static const std::string& battery_state = "battery_state_topic";
} //namespace topic_name

namespace param_name{
static const std::string& cm_ip_addr = "cm_ip_addr"; //Control Module IP Address
static const std::string& cm_port = "cm_port"; //Control Module Port Number
} //namespace param_name

namespace srv_name{
static const std::string& eth_connect = "eth_connect_srv";
static const std::string& eth_streaming = "eth_streaming_srv";
} //namespace srv_name


static const std::vector<std::string>& joint_names = {"left_hip", "left_knee", "right_hip", "right_knee"};
static const std::vector<std::string>& part_names = {"left", "right"};

const int n_joint_axis = (int)joint_names.size();
const int n_cartesian_axis = 3;

static const std::string& default_ip = "192.168.0.181";
static const int default_port = 7;



typedef enum {
    Position,
    Velocity,
    Effort,
}ControlTarget;

typedef enum {
    SetPoint,
    Trajectory,
}ControlMode;


typedef enum {
    Streaming =  3,
    Connected =  2,
    OK        =  0, //Ready to connect
    SocketErr = -1,
    AddrErr   = -2,
    ConnErr   = -3,
    SendErr   = -4,
    RecvErr   = -5,
}ETH_Status;


class EthernetBridge : public rclcpp::Node
{
public:
    EthernetBridge();

private:
    /* RCL Timer */
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback();

    /* RCL Pub/Sub */
    rclcpp::Publisher<StringMsg>::SharedPtr inner_voice_pub_;    
    rclcpp::Publisher<TrajMsg>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<TrajMsg>::SharedPtr cartesian_state_pub_;
    rclcpp::Publisher<Uint16ArrMsg>::SharedPtr cm_state_pub_;
    rclcpp::Publisher<BatteryMsg>::SharedPtr battery_state_pub_;

    rclcpp::Subscription<TrajMsg>::SharedPtr joint_target_sub_;
    void JointCommandCallback(const TrajMsg::ConstSharedPtr cmd);
    rclcpp::Subscription<TrajMsg>::SharedPtr cartesian_target_sub_;
    void CartesianCommandCallback(const TrajMsg::ConstSharedPtr cmd);
    rclcpp::Subscription<Uint16ArrMsg>::SharedPtr cm_command_sub_;
    void CM_CommandCallback(const Uint16ArrMsg::ConstSharedPtr cmd);

    /* RCL Srv/Cli */
    rclcpp::Service<SetBoolSrv>::SharedPtr eth_connect_srv_;
    void ConnectSrvCallbck(const std::shared_ptr<SetBoolSrv::Request> req, std::shared_ptr<SetBoolSrv::Response> res);
    rclcpp::Service<SetBoolSrv>::SharedPtr eth_streaming_srv_;
    void StreamingSrvCallbck(const std::shared_ptr<SetBoolSrv::Request> req, std::shared_ptr<SetBoolSrv::Response> res);

    /* RCL Params */
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
    rcl_interfaces::msg::SetParametersResult SetParametersCallback(const std::vector<rclcpp::Parameter>& params);

    /* RCL Msgs */
    void TrajectoryMsgInitializer(trajectory_msgs::msg::JointTrajectory& msg, const std::vector<std::string>& joint_names);

    // /* CM Data */
    // ControlTarget control_target_;
    // ControlMode control_mode_;

    Uint16ArrMsg cm_state_;
    TrajMsg joint_state_;
    TrajMsg cartesian_state_;
    BatteryMsg battery_state_;

    // TrajMsg joint_target_;
    // TrajMsg cartesian_target_;
    
    /* ETH Communications */
    int hsock_;
    struct sockaddr_in sockaddr_;
    ETH_Status eth_status_;

    eth::Packet tx_;
    eth::Packet rx_;

    ETH_Status ETH_Configure();
    ETH_Status ETH_Connect();
    ETH_Status ETH_SendReceive();
    ETH_Status ETH_Disconnect();

    bool ETH_PrintStatus(ETH_Status status) const;
    
    /* Misc. */
    size_t count_;

public:
    void TestConnection();
};


#endif // ETH_BRIDGE_H_