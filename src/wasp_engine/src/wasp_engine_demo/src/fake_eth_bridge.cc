#include "wasp_engine_demo/fake_eth_bridge.h"

using namespace std;
using namespace rclcpp;

using std::placeholders::_1;

FakeEthernetBridge::FakeEthernetBridge() : Node(node_name), count_(0)
{
    inner_voice_pub_ = this->create_publisher<StringMsg>(node_name + "/" + inner_voice_topic_name, 10);

    joint_state_pub_   = this->create_publisher<JointMsg>(node_name + "/" + joint_state_topic_name, 10);
    joint_command_sub_ = this->create_subscription<JointMsg>(node_name + "/" + joint_command_topic_name, 10, 
                                    bind(&FakeEthernetBridge::JointCommandCallback, this, _1));

    cartesian_state_pub_   = this->create_publisher<JointMsg>(node_name + "/" + cartesian_state_topic_name, 10);
    cartesian_command_sub_ = this->create_subscription<CartesianMsg>(node_name + "/" + cartesian_command_topic_name, 10, 
                                    bind(&FakeEthernetBridge::CartesianCommandCallback, this, _1));

    timer_ = this->create_wall_timer(1ms, bind(&FakeEthernetBridge::TimerCallback, this));

    control_target_ = Effort;
    control_mode_ = SetPoint;

    TrajectoryMsgInitializer(joint_state_,   joint_names);
    TrajectoryMsgInitializer(joint_command_, joint_names);
    TrajectoryMsgInitializer(cartesian_state_,   part_names);
    TrajectoryMsgInitializer(cartesian_command_, part_names);
    UpdateKinematics();
}


void FakeEthernetBridge::TimerCallback()
{
    ++count_;
    UpdateJointDynamics();
    UpdateKinematics();
    joint_state_pub_->publish(joint_state_);
    cartesian_state_pub_->publish(cartesian_state_);
}


void FakeEthernetBridge::JointCommandCallback(const JointMsg::ConstSharedPtr cmd)
{
    switch (control_target_) {
    case Position: joint_command_.points[0].positions = cmd->points[0].positions;   break;
    case Velocity: joint_command_.points[0].velocities = cmd->points[0].velocities; break;
    case Effort:   joint_command_.points[0].effort = cmd->points[0].effort;         break;    
    default: RCLCPP_ERROR(this->get_logger(), "Invalid control target!"); break;
    }
}


void FakeEthernetBridge::CartesianCommandCallback(const CartesianMsg::ConstSharedPtr cmd)
{
    switch (control_target_) {
    case Position: cartesian_command_.points[0].positions = cmd->points[0].positions;   break;
    case Velocity: cartesian_command_.points[0].velocities = cmd->points[0].velocities; break;
    case Effort:   cartesian_command_.points[0].effort = cmd->points[0].effort;         break;    
    default: RCLCPP_ERROR(this->get_logger(), "Invalid control target!"); break;
    }
}


void FakeEthernetBridge::TrajectoryMsgInitializer(trajectory_msgs::msg::JointTrajectory& msg, const std::vector<std::string>& names)
{
    msg.joint_names.clear();
    msg.points.clear();
    msg.points.resize(1);    
    for(auto name : names){
        msg.joint_names.push_back(name);
        msg.points[0].positions.push_back(0);
        msg.points[0].velocities.push_back(0);
        msg.points[0].accelerations.push_back(0);
        msg.points[0].effort.push_back(0);
    }
}


void FakeEthernetBridge::UpdateJointDynamics()
{
    // dynamics parameters
    const float tau = 0.01;
    const float b = 0.1;
    const float J = 1;
    const float h = 0.001;
    
    auto joint_state_old = joint_state_;
    
    for(int i = 0; i < n_joint_axis; i++){
        float p_new = 0, v_new = 0, e_new = 0;
        float p_old = 0, v_old = 0, e_old = 0;
        float p_cmd = 0, v_cmd = 0, e_cmd = 0;

        p_old = joint_state_.points[0].positions.at(i);
        v_old = joint_state_.points[0].velocities.at(i);
        e_old = joint_state_.points[0].effort.at(i);

        p_cmd = joint_command_.points[0].positions.at(i);
        v_cmd = joint_command_.points[0].velocities.at(i);
        e_cmd = joint_command_.points[0].effort.at(i);

        // update effort
        if(control_target_ == Effort){
            e_new =   (tau * e_cmd) + ((1-tau) * e_old) // command following term
                     -(b * v_old); // damping term
        }

        // update velocity
        if(control_target_ == Velocity){
            v_new = (tau * v_cmd) + ((1-tau) * v_old);  // command following
        }else if(control_target_ == Effort){
            v_new = v_old + (e_new*h/J);
        }

        // update position
        if(control_target_ == Position){
            p_new = (tau * p_cmd) + ((1-tau) * p_old);
        }else{
            p_new = p_old + (v_new*h);
        }

        joint_state_.points[0].positions.at(i) = p_new;
        joint_state_.points[0].velocities.at(i) = v_new;
        joint_state_.points[0].effort.at(i) = e_new;
    }
}

void FakeEthernetBridge::UpdateKinematics()
{
    // TODO
}


int main(int argc, char** argv)
{
    init(argc, argv);
    auto fake_eth_bridge_node = make_shared<FakeEthernetBridge>();
    RCLCPP_INFO(fake_eth_bridge_node->get_logger(), "Fake ETH Bridge Start!");
    spin(fake_eth_bridge_node);
    RCLCPP_INFO(fake_eth_bridge_node->get_logger(), "Fake ETH Bridge Shutting Down!");
    shutdown();
    return 0;
}