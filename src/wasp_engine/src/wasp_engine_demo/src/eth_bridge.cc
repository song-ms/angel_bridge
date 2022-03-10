#include "wasp_engine_demo/eth_bridge.h"

using namespace std;
using namespace rclcpp;

using std::placeholders::_1;
using std::placeholders::_2;


/*
   ___             _               _           
  / __|___ _ _  __| |_ _ _ _  _ __| |_ ___ _ _ 
 | (__/ _ \ ' \(_-<  _| '_| || / _|  _/ _ \ '_|
  \___\___/_||_/__/\__|_|  \_,_\__|\__\___/_|  
                                               
*/
EthernetBridge::EthernetBridge() : Node(node_name), hsock_(0), eth_status_(OK), count_(0)
{

    inner_voice_pub_ = this->create_publisher<StringMsg>(node_name + "/" + topic_name::inner_voice, 10);

    joint_state_pub_   = this->create_publisher<TrajMsg>(node_name + "/" + topic_name::joint_state, 10);
    joint_target_sub_ = this->create_subscription<TrajMsg>(node_name + "/" + topic_name::joint_target, 10, 
                                    bind(&EthernetBridge::JointCommandCallback, this, _1));

    cartesian_state_pub_   = this->create_publisher<TrajMsg>(node_name + "/" + topic_name::cartesian_state, 10);
    cartesian_target_sub_ = this->create_subscription<TrajMsg>(node_name + "/" + topic_name::cartesian_target, 10, 
                                    bind(&EthernetBridge::CartesianCommandCallback, this, _1));

    cm_state_pub_   = this->create_publisher<Uint16ArrMsg>(node_name + "/" + topic_name::cm_status, 10);
    cm_command_sub_ = this->create_subscription<Uint16ArrMsg>(node_name + "/" + topic_name::cm_command, 10, 
                                    bind(&EthernetBridge::CM_CommandCallback, this, _1));
                                    
    battery_state_pub_ = this->create_publisher<BatteryMsg>(node_name + "/" + topic_name::battery_state, 10);

    timer_ = this->create_wall_timer(1ms, bind(&EthernetBridge::TimerCallback, this));

    eth_connect_srv_   = this->create_service<SetBoolSrv>(node_name + "/" + srv_name::eth_connect,
                                    bind(&EthernetBridge::ConnectSrvCallbck, this, _1, _2));
    eth_streaming_srv_ = this->create_service<SetBoolSrv>(node_name + "/" + srv_name::eth_streaming,
                                    bind(&EthernetBridge::StreamingSrvCallbck, this, _1, _2));

    this->declare_parameter<std::string>(param_name::cm_ip_addr, default_ip);
    this->declare_parameter<int>(param_name::cm_port, default_port);

    param_callback_handle = this->add_on_set_parameters_callback(std::bind(&EthernetBridge::SetParametersCallback, this, _1));

    // control_target_ = Effort;
    // control_mode_ = SetPoint;

    TrajectoryMsgInitializer(joint_state_,   joint_names);
    // TrajectoryMsgInitializer(joint_target_, joint_names);
    TrajectoryMsgInitializer(cartesian_state_,   part_names);
    // TrajectoryMsgInitializer(cartesian_target_, part_names);
    cm_state_.data = {0, 0, 0};
    battery_state_.voltage = 24;
    battery_state_.percentage = 0;
    battery_state_.power_supply_health = battery_state_.POWER_SUPPLY_HEALTH_GOOD;

    eth_status_ = OK;
    RCLCPP_INFO(this->get_logger(), "ETH Bridge Start!");
}

/*
  ___  ___ _      _____ _                  ___      _ _ _             _   
 | _ \/ __| |    |_   _(_)_ __  ___ _ _   / __|__ _| | | |__  __ _ __| |__
 |   / (__| |__    | | | | '  \/ -_) '_| | (__/ _` | | | '_ \/ _` / _| / /
 |_|_\\___|____|   |_| |_|_|_|_\___|_|    \___\__,_|_|_|_.__/\__,_\__|_\_\
                                                                          
*/

void EthernetBridge::TimerCallback()
{
    std::vector<float> offset = {273647, 162905, 291137, 244335};
    ++count_;
    ETH_Status prev_status = eth_status_;

    if(eth_status_ == Streaming){

        eth_status_ = ETH_SendReceive();
        if(rx_.IsMsgRecieved(eth::ServerSysData)){            
            for(int i = 0; i < n_joint_axis; i++){
                joint_state_.points[0].positions[i]  = 
                    (rx_.GetData().sys_data->moduleData.actualLinkPosition[i] - offset[i])/1000;
            }
            joint_state_pub_->publish(joint_state_);
        }

        if(count_%100 == 0){
            battery_state_.voltage = rx_.GetData().sys_data->cartesianTargetPose[0];
            battery_state_.current = rx_.GetData().sys_data->cartesianTargetPose[1];
            if(battery_state_.voltage > 25.2){
                battery_state_.percentage = 100;
                battery_state_.power_supply_health = battery_state_.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            }else{
                battery_state_.percentage = (battery_state_.voltage - 19.0)/6.2*100;
                battery_state_.power_supply_health = battery_state_.POWER_SUPPLY_HEALTH_GOOD;
            }
            battery_state_pub_->publish(battery_state_);
        }
    }
    
    if(prev_status != eth_status_){
        if(ETH_PrintStatus(eth_status_)){
            RCLCPP_WARN(this->get_logger(), "Streaming Stop!");
        }
    }

    cm_state_.data[0] = rx_.GetCommState();
    cm_state_.data[1] = rx_.GetControlMode();
    cm_state_.data[2] = static_cast<uint16_t>(eth_status_);
    cm_state_pub_->publish(cm_state_);
    // cartesian_state_pub_->publish(cartesian_state_);
    
}


/*
  ___  ___ _      ___      _          ___      _ _ _             _       
 | _ \/ __| |    / __|_  _| |__ ___  / __|__ _| | | |__  __ _ __| |__ ___
 |   / (__| |__  \__ \ || | '_ (_-< | (__/ _` | | | '_ \/ _` / _| / /(_-<
 |_|_\\___|____| |___/\_,_|_.__/__/  \___\__,_|_|_|_.__/\__,_\__|_\_\/__/
                                                                                                                                              
*/

void EthernetBridge::JointCommandCallback(const TrajMsg::ConstSharedPtr cmd)
{
    tx_.SetData().j_target->jointTarget[0] = -cmd->points[0].effort[0];
    tx_.SetData().j_target->jointTarget[1] = +cmd->points[0].effort[1];
    tx_.SetData().j_target->jointTarget[2] = +cmd->points[0].effort[2];
    tx_.SetData().j_target->jointTarget[3] = -cmd->points[0].effort[3];
    tx_.SetControlMode(JOINT_MODE);
    tx_.SetMsgType(eth::JointTargetSet);
}


void EthernetBridge::CartesianCommandCallback(const TrajMsg::ConstSharedPtr cmd)
{
    // cartesian_target_ = *cmd;
    (void)(cmd);
}

void EthernetBridge::CM_CommandCallback(const Uint16ArrMsg::ConstSharedPtr cmd)
{
    uint16_t comm_state = cmd->data.at(0);
    uint16_t ctrl_mode = cmd->data.at(1);
    RCLCPP_INFO(this->get_logger(), "Set CommState: %d, CtrlMode: %d", comm_state, ctrl_mode);
    tx_.SetCommState(comm_state);
}

/*
  ___  ___ _      ___            ___      _ _ _             _       
 | _ \/ __| |    / __|_ ___ __  / __|__ _| | | |__  __ _ __| |__ ___
 |   / (__| |__  \__ \ '_\ V / | (__/ _` | | | '_ \/ _` / _| / /(_-<
 |_|_\\___|____| |___/_|  \_/   \___\__,_|_|_|_.__/\__,_\__|_\_\/__/
                                                                                                                               
*/
void EthernetBridge::ConnectSrvCallbck(const std::shared_ptr<SetBoolSrv::Request> req, std::shared_ptr<SetBoolSrv::Response> res)
{
    bool connect = req->data;
    if(eth_status_ > 0){
        /* ETH already connected */
        if(connect == true){
            RCLCPP_INFO(this->get_logger(), "Already connected!");
            res->message = "Already connected";
            res->success = true;
        }else{
            eth_status_ = ETH_Disconnect();
            res->message = "Disconnected";
            res->success = true;
        }
    }else{
        /* Try ETH connection */
        if(connect == true){
            eth_status_ = ETH_Configure();
            eth_status_ = ETH_Connect();
            if(eth_status_ < 0){
                res->message = "Connection failed";
                res->success = false;
            }else{
                res->message = "Connection successed";
                res->success = true;
            }
        }else{
            RCLCPP_INFO(this->get_logger(), "Already disconnected!");
            res->message = "Already disconnected";
            res->success = true;
        }
    }
    ETH_PrintStatus(eth_status_);
}

void EthernetBridge::StreamingSrvCallbck(const std::shared_ptr<SetBoolSrv::Request> req, std::shared_ptr<SetBoolSrv::Response> res)
{
    bool start_streaming = req->data;
    if(eth_status_ == Streaming){
        /* Already streaming */
        if(start_streaming == false){
            RCLCPP_INFO(this->get_logger(), "Streming Stop!");
            eth_status_ = Connected;
            res->message = "Streaming stopepd";
            res->success = true;
        }else{
            RCLCPP_INFO(this->get_logger(), "Already streaming!");
            res->message = "Already streaming";
            res->success = true;
        }
    }else if(eth_status_ == Connected){
        /* Ready to streaming */
        if(start_streaming == true){
            RCLCPP_INFO(this->get_logger(), "Streming Start!");
            eth_status_ = Streaming;
            res->message = "Streaming started";
            res->success = true;
        }else{
            RCLCPP_INFO(this->get_logger(), "Already not streaming!");
            res->message = "Already not streaming";
            res->success = true;
        }
    }else{
        /* Not connected */
        RCLCPP_WARN(this->get_logger(), "Connect First!");
        res->message = "Connect first";
        res->success = false;
    }
}


/*
  ___      _     ___                        ___      _ _ _             _   
 / __| ___| |_  | _ \__ _ _ _ __ _ _ __    / __|__ _| | | |__  __ _ __| |__
 \__ \/ -_)  _| |  _/ _` | '_/ _` | '  \  | (__/ _` | | | '_ \/ _` / _| / /
 |___/\___|\__| |_| \__,_|_| \__,_|_|_|_|  \___\__,_|_|_|_.__/\__,_\__|_\_\
                                                                           
*/
rcl_interfaces::msg::SetParametersResult EthernetBridge::SetParametersCallback(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult res;    
    res.successful = false;
    res.reason = "no parameter has been set";
    for(const auto& param : params){
        (void)(param);
    }
    return res;
}



/*
  ___ _____ _  _   ___             _   _             
 | __|_   _| || | | __|  _ _ _  __| |_(_)___ _ _  ___
 | _|  | | | __ | | _| || | ' \/ _|  _| / _ \ ' \(_-<
 |___| |_| |_||_| |_| \_,_|_||_\__|\__|_\___/_||_/__/
                                                     
*/

ETH_Status EthernetBridge::ETH_Configure()
{
    if(hsock_ == 0){
        RCLCPP_INFO(this->get_logger(), "Configuration start...");
    }else{
        RCLCPP_INFO(this->get_logger(), "Reconfiguring...");
        close(hsock_);
    }

    /* Open socket */
    hsock_ = socket(AF_INET, SOCK_STREAM, 0);
	if (hsock_ < 0){
		return SocketErr;
	}
	RCLCPP_INFO(this->get_logger(), "Successfully opened socket");

    /* Sock Addr */
    std::string ip_addr = "";
	int port;
    this->get_parameter<std::string>(param_name::cm_ip_addr, ip_addr);
    this->get_parameter<int>(param_name::cm_port, port);

	sockaddr_.sin_family = AF_INET;
	sockaddr_.sin_port = htons(port);

	if (inet_pton(AF_INET, ip_addr.c_str(), &sockaddr_.sin_addr) != 1){
		return AddrErr;
	}
    RCLCPP_INFO(this->get_logger(), "Successfully set IP address %s:%d", ip_addr.c_str(), port);
    RCLCPP_INFO(this->get_logger(), "Configuration finished!");

    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 1000*10;
    setsockopt(hsock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));

    return OK;
}

ETH_Status EthernetBridge::ETH_Connect()
{
    /* Connect */
    RCLCPP_INFO(this->get_logger(), "Connecting to server...");
	if(connect(hsock_, (sockaddr*)&sockaddr_, sizeof(sockaddr_)) < 0){
        RCLCPP_ERROR(this->get_logger(), "Connection Failed!");
		return ConnErr;
    }
    RCLCPP_INFO(this->get_logger(), "Connection Successed!");
    // /* Registration */
    // RCLCPP_INFO(this->get_logger(), "Registration...");
    // tx_.SetData().msg_state->commState = REGISTRATION;
    // tx_.PackData();
    // while(true){
    //     ETH_SendReceive();
    //     rx_.UnpackData();
    //     if(rx_.GetCommState() == REGISTRATION_COMPLETE){
    //         break;
    //     }
    // };
    // RCLCPP_INFO(this->get_logger(), "Registration Complete!");
    return Connected;
}

ETH_Status EthernetBridge::ETH_SendReceive()
{
    /* Send */
    tx_.PackData();
    if (send(hsock_, tx_.ToChar(), tx_.Size(), 0) < 0) {
        return SendErr;
    }
    /* Recv */
    int recv_bytes = recv(hsock_, rx_.ToChar(), rx_.MaxSize(), 0);
    if (recv_bytes < 0){
        // std::cout << std::endl;
        // std::cout << "!Err\t#" << count_ << "\t" << std::strerror(errno) << std::endl;
        RCLCPP_ERROR(this->get_logger(), "Recv error: %s", std::strerror(errno));
        // return RecvErr;
    }
    //  else {
        // std::cout << "\rRecv\t#" << count_ << "\t" << recv_bytes << "bytes" << std::flush;
        // RCLCPP_INFO (this->get_logger(), "Recv bytes: %d", recv_bytes);
    // }
    rx_.UnpackData();
    return eth_status_;
}

ETH_Status EthernetBridge::ETH_Disconnect()
{
    RCLCPP_INFO(this->get_logger(), "Disconnecting!");
    close(hsock_);
    return OK;
}

bool EthernetBridge::ETH_PrintStatus(ETH_Status status) const{
    switch (status) {
    case Streaming: RCLCPP_INFO (this->get_logger(), "Streaming!");           break;
    case Connected: RCLCPP_INFO (this->get_logger(), "Socket is Connected!"); break;
    case OK:        RCLCPP_INFO (this->get_logger(), "Socket is Ready!");     break;
    case SocketErr: RCLCPP_ERROR(this->get_logger(), "Socket Error!");        break;
    case AddrErr:   RCLCPP_ERROR(this->get_logger(), "IP Address Error!");    break;
    case ConnErr:   RCLCPP_ERROR(this->get_logger(), "Connection Error!");    break;
    case SendErr:   RCLCPP_ERROR(this->get_logger(), "Send Error!");          break;
    case RecvErr:   RCLCPP_ERROR(this->get_logger(), "Receive Error!");       break;
    default:
        if(status < 0){
            RCLCPP_ERROR(this->get_logger(), "Invalid configuration Error!");
            return true;
        }
        break;
    }
    return (status < 0) ? true : false;
}


/*
  _  _     _                 __  __     _   _            _    
 | || |___| |_ __  ___ _ _  |  \/  |___| |_| |_  ___  __| |___
 | __ / -_) | '_ \/ -_) '_| | |\/| / -_)  _| ' \/ _ \/ _` (_-<
 |_||_\___|_| .__/\___|_|   |_|  |_\___|\__|_||_\___/\__,_/__/
            |_|                                               
*/
void EthernetBridge::TrajectoryMsgInitializer(trajectory_msgs::msg::JointTrajectory& msg, const std::vector<std::string>& names)
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

void EthernetBridge::TestConnection()
{
    /* Self Connection & Streaming for test */
    eth_status_ = ETH_Configure();
    ETH_PrintStatus(eth_status_);
    eth_status_ = ETH_Connect();
    ETH_PrintStatus(eth_status_);

    tx_.SetCommState(REGISTRATION);
    tx_.SetControlMode(JOINT_MODE);
    RCLCPP_INFO(this->get_logger(), "Registration Start...");
    while(1){
        eth_status_ = ETH_SendReceive();
        cm_state_.data[0] = rx_.GetCommState();
        if(cm_state_.data[0] == REGISTRATION_COMPLETE){
            break;
        }
        if(eth_status_ < 0){
            ETH_PrintStatus(eth_status_);
            break;
        }
        usleep(500*1000);        
    }

    tx_.SetCommState(SERVO_ON);
    RCLCPP_INFO(this->get_logger(), "Registration Complete!");

    RCLCPP_INFO(this->get_logger(), "Servo On Start...");
    while(1){
        eth_status_ = ETH_SendReceive();
        cm_state_.data[0] = rx_.GetCommState();
        if(cm_state_.data[0] == SERVO_ON_COMPLETE){
            break;
        } 
        if(eth_status_ < 0){
            ETH_PrintStatus(eth_status_);
            break;
        }
        usleep(500*1000);
    }
    RCLCPP_INFO(this->get_logger(), "Servo On Complete!");

    tx_.SetCommState(FREE_STATE);
    while(1){
        eth_status_ = ETH_SendReceive();
        cm_state_.data[0] = rx_.GetCommState();
        if(cm_state_.data[0] == FREE_STATE){
            break;
        }
        if(eth_status_ < 0){
            ETH_PrintStatus(eth_status_);
            break;
        }
        usleep(500*1000);
    }
    eth_status_ = Streaming;
    RCLCPP_INFO(this->get_logger(), "Free State Start!");
}


/*
  __  __      _      
 |  \/  |__ _(_)_ _  
 | |\/| / _` | | ' \ 
 |_|  |_\__,_|_|_||_|
                     
*/
int main(int argc, char** argv)
{
    init(argc, argv);
    auto eth_bridge_node = make_shared<EthernetBridge>();    
    eth_bridge_node->TestConnection();
    spin(eth_bridge_node);
    RCLCPP_INFO(eth_bridge_node->get_logger(), "ETH Bridge Shutting Down!");
    shutdown();
    return 0;
}