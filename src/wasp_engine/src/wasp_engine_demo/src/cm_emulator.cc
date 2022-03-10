#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <random>

#include <cerrno>
#include <cstring>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "eth_packet/eth_packet.hpp"

class JointEmulator
{
public:
    JointEmulator(float dt, float init_pos, float origin_pos, bool reverse_effort)
         : dt_(dt), dir_(reverse_effort), pos_o_(origin_pos), pos_(init_pos), vel_(0), acc_(0)
    {
        
    }
    
    float pos() {return pos_;}
    float vel() {return vel_;}
    float acc() {return acc_;}

    void Update(float effort)
    {
        float m = 10;
        float c = 80;
        float k = 20;

        float eff = dir_ ? -effort : effort;
        acc_ = (eff - c*vel_ - k*(pos_-pos_o_))/m;
        vel_ += acc_*dt_;
        pos_ += vel_*dt_;
    }
    
private:
    float dt_;
    bool dir_;
    float pos_o_;
    float pos_;
    float vel_;
    float acc_;
};


int main()
{
    eth::Packet tx;
    eth::Packet rx;
    float dt = 0.001;
    auto lh = new JointEmulator(dt, -90, -90, true);
    auto lk = new JointEmulator(dt, 0, 0, false);
    auto rh = new JointEmulator(dt, -90, -90, false);
    auto rk = new JointEmulator(dt, 0, 0, true);
    std::vector<JointEmulator*> joints = {lh, lk, rh, rk};
    std::random_device rd;
    std::mt19937 mersenne(rd());
    std::uniform_int_distribution<> noise(-100, 100);

    int hsock = socket(AF_INET, SOCK_STREAM, 0);

    if (hsock < 0){
        std::cerr << "Socket Creation Error" << std::endl;
		return -1;
	}

    int on = 1;
    if(setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (const char*) &on, sizeof(on)) < 0){
        std::cerr << "Socket Option Error" << std::endl;
        return -1;
    }

    sockaddr_in srvaddr;
    srvaddr.sin_addr.s_addr = INADDR_ANY;
    srvaddr.sin_port = htons(1818);
    srvaddr.sin_family = AF_INET;

    if(bind(hsock, (struct sockaddr *) &srvaddr, sizeof(srvaddr)) < 0){
        std::cerr << "Bind Error: " << std::strerror(errno) << std::endl;
        return -1;
    }

    std::cout << "Waiting for Client..." << std::endl;
    if(listen(hsock, 1) < 0){
        std::cerr << "Listen Error" << std::strerror(errno) << std::endl;
		return -1;
    }

    sockaddr_in cliaddr;
    int cliaddr_size = sizeof(cliaddr);
    while(true){
        int cli = accept(hsock, (struct sockaddr *) &cliaddr, (socklen_t *) &cliaddr_size);
        if(cli < 0){
            std::cerr << "Listen Error"<< std::strerror(errno) << std::endl;
            return 0;
        }

        std::cout << "client " 
                  << inet_ntoa(cliaddr.sin_addr) << ":" << ntohs(cliaddr.sin_port) 
                  << " accepted" << std::endl;

        while(true){
            if(recv(cli, rx.ToChar(), rx.MaxSize(), 0) < 0){
                std::cerr << "Recv Error: " << std::strerror(errno) << std::endl;
            }
            rx.UnpackData();

            int comm_state = rx.GetCommState();
            switch(comm_state){
                case REGISTRATION: tx.SetCommState(REGISTRATION_COMPLETE); break;
                case SERVO_ON:     tx.SetCommState(SERVO_ON_COMPLETE); break;
                case TUNING_STATE: tx.SetCommState(TUNING_STATE_COMPLETE); break;
                case HOMING:       tx.SetCommState(HOMING_COMPLETE); break;
                case FREE_STATE:   tx.SetCommState(FREE_STATE); break;
                default: 
                    std::cerr << "Invalid CommState" << std::endl;
                    return -1;
            }

            if(comm_state == FREE_STATE){
                tx.SetMsgType(eth::MsgType::ServerSysData);
                for (int i = 0; i < JOINT_AXIS; i++){
                    joints.at(i)->Update(rx.GetData().j_target->jointTarget[i]);
                    float new_pos = joints.at(i)->pos();
                    tx.SetData().sys_data->moduleData.actualLinkPosition[0] = new_pos;
                }

                float battvol = 24 + (float)noise(mersenne)/100*3;
                tx.SetData().sys_data->cartesianTargetPose[0] = battvol;
            }

            tx.PackData();
            if(send(cli, tx.ToChar(), tx.Size(), 0) < 0){
                std::cerr << "Send Error: " << std::strerror(errno) << std::endl;
            }

        }
        close(cli);
    }

    close(hsock);

    return 0;
}
