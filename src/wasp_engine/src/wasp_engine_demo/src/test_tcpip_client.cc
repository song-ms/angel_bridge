#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <iostream>
#include <inttypes.h>

#include <string.h>

#include "eth_packet/eth_packet.hpp"

void JointParameterSet(const eth::Packet& pkt);
void JointTrajectorySet(const eth::Packet& pkt);
void CartesianParamSet(const eth::Packet& pkt);
void CartesianTrajectorySet(const eth::Packet& pkt);
void CartesianTargetSet(const eth::Packet& pkt);
void JointTargetSet(const eth::Packet& pkt);
void ServerSystemDataSet(const eth::Packet& pkt);

void printf_jointTraj(const eth::Packet& pkt);
void printf_jointParam(const eth::Packet& pkt);
void printf_CartesianParam(const eth::Packet& pkt);
void printf_CartesianTraj(const eth::Packet& pkt);
void printf_JointTarget(const eth::Packet& pkt);
void printf_CartesianTarget(const eth::Packet& pkt);
void printf_ServerSystemData(const eth::Packet& pkt);


int main()
{
    eth::Packet tx;
    eth::Packet rx;

    std::string ipAddress = "192.168.0.182"; //"192.168.0.117"; 
    int port = 7;
    int hSocket;
    struct sockaddr_in servAddr;

    int strLen;
    std::string state;

    /*
       ___                    ___                      _   _          
      / _ \ _ __  ___ _ _    / __|___ _ _  _ _  ___ __| |_(_)___ _ _  
     | (_) | '_ \/ -_) ' \  | (__/ _ \ ' \| ' \/ -_) _|  _| / _ \ ' \ 
      \___/| .__/\___|_||_|  \___\___/_||_|_||_\___\__|\__|_\___/_||_|
           |_|                                                        
    */

    std::cout << "socket open" << std::endl;
	hSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (hSocket < 0)
	{
		std::cerr << "socket() error" << std::endl;
		return -1;
	}

	// memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_port = htons(port);

    std::cout << "convert IP address" << std::endl;
	int convResult = inet_pton(AF_INET, ipAddress.c_str(), &servAddr.sin_addr);
	if (convResult != 1){
		std::cerr << "Can't convert IP address, Err #" << convResult << std::endl;
		return -1;
	}

    std::cout << "connect" << std::endl;
	if (connect(hSocket, (sockaddr*)&servAddr, sizeof(servAddr)) < 0){
		std::cerr << "connect() error!" << std::endl;
		return -1;
	}
    
    std::cout << "*** Test Start ***" << std::endl;

    /*
      ___          _    _            _   _          
     | _ \___ __ _(_)__| |_ _ _ __ _| |_(_)___ _ _  
     |   / -_) _` | (_-<  _| '_/ _` |  _| / _ \ ' \ 
     |_|_\___\__, |_/__/\__|_| \__,_|\__|_\___/_||_|
             |___/                                  
    */
    state = "Registration";
    tx.SetData().msg_state->commState = REGISTRATION;

    tx.PackData();

    if (send(hSocket, tx.ToChar(), tx.Size(), 0) == -1) {
        printf("Registration Send Error!!!\n");
    }

    strLen = recv(hSocket, rx.ToChar(), rx.MaxSize(), 0);
    if (strLen == -1){
        printf("Registration Recv Error!!!\n");
    }
    rx.UnpackData();

    while(1){
        int cs = rx.GetData().msg_state->commState;
        switch (cs){
        case REGISTRATION_COMPLETE:
            printf("Registration Complete!!\n");
            /*
             ___                     ___       
            / __| ___ _ ___ _____   / _ \ _ _  
            \__ \/ -_) '_\ V / _ \ | (_) | ' \ 
            |___/\___|_|  \_/\___/  \___/|_||_|
                                    
            */
            state = "Servo On";
            tx.SetData().msg_state->commState = SERVO_ON;
            break;

        case SERVO_ON_COMPLETE:
            printf("Servo On Complete!!\n");

            /*
             _____          _             ___ _        _       
            |_   _|  _ _ _ (_)_ _  __ _  / __| |_ __ _| |_ ___ 
              | || || | ' \| | ' \/ _` | \__ \  _/ _` |  _/ -_)
              |_| \_,_|_||_|_|_||_\__, | |___/\__\__,_|\__\___|
                                  |___/                        
            */
            state = "Tunning";
            tx.SetData().msg_state->commState = TUNING_STATE;
            tx.SetMsgType(eth::JointParamSet);
            tx.SetMsgType(eth::JointTrajSet);
            tx.SetMsgType(eth::CartesianParamSet);
            tx.SetMsgType(eth::CartesianTrajSet);
            
            JointParameterSet(tx);
			JointTrajectorySet(tx);
			CartesianParamSet(tx);
			CartesianTrajectorySet(tx);

            break;

        case TUNING_STATE_COMPLETE:
            printf("Tunning Complete!!\n");
            /*
             _  _           _           
            | || |___ _ __ (_)_ _  __ _ 
            | __ / _ \ '  \| | ' \/ _` |
            |_||_\___/_|_|_|_|_||_\__, |
                                  |___/ 
            */
            state = "Homing";
            tx.SetData().msg_state->commState = HOMING;
            tx.SetMsgType(eth::JointTargetSet);
            
            JointTargetSet(tx);
            
            break;

        case HOMING_COMPLETE:
            printf("Homing Complete!!\n");
            /*
             ___              ___ _        _       
            | __| _ ___ ___  / __| |_ __ _| |_ ___ 
            | _| '_/ -_) -_) \__ \  _/ _` |  _/ -_)
            |_||_| \___\___| |___/\__\__,_|\__\___|
                                                    
            */
            state = "Free State";
            tx.SetData().msg_state->commState = FREE_STATE;
            tx.SetMsgType(eth::JointParamSet);
            tx.SetMsgType(eth::JointTrajSet);
            tx.SetMsgType(eth::CartesianParamSet);
            tx.SetMsgType(eth::CartesianTrajSet);
            tx.SetMsgType(eth::JointTargetSet);
            tx.SetMsgType(eth::CartesianTargetSet);

            JointParameterSet(tx);
			JointTrajectorySet(tx);
			CartesianParamSet(tx);
			CartesianTrajectorySet(tx);
			JointTargetSet(tx);
			CartesianTargetSet(tx);

            break;

        case FREE_STATE:
            printf("Free State!!\n");
            state = "Free State";
            tx.SetData().msg_state->commState = FREE_STATE;
            tx.SetMsgType(eth::JointParamSet);
            tx.SetMsgType(eth::JointTrajSet);
            tx.SetMsgType(eth::CartesianParamSet);
            tx.SetMsgType(eth::CartesianTrajSet);
            tx.SetMsgType(eth::JointTargetSet);
            tx.SetMsgType(eth::CartesianTargetSet);

            JointParameterSet(tx);
			JointTrajectorySet(tx);
			CartesianParamSet(tx);
			CartesianTrajectorySet(tx);
			JointTargetSet(tx);
			CartesianTargetSet(tx);

            break;
        default:
            std::cerr << "Invalid CommState: " << +cs << std::endl;
            // throw("shit");
            break;
        }


        tx.PackData();
        if (send(hSocket, tx.ToChar(), tx.Size(), 0) == -1) {
            std::cerr << state << " Send Error!!!" << std::endl;
        }

        strLen = recv(hSocket, rx.ToChar(), rx.MaxSize(), 0);
        if (strLen == -1){
            std::cerr << state << " Recv Error!!!" << std::endl;
        }
        printf("Bytes received: %d\n", strLen);
        rx.UnpackData();

        uint8_t msgs = rx.GetData().msg_state->packetType[0];
        for(int i = 0; i < 8; i++){
            uint8_t tmp = msgs & (1 << i);
            switch (tmp) {
            case 0x01: printf_jointParam(rx);       break;
            case 0x02: printf_jointTraj(rx);        break;
            case 0x04: printf_CartesianParam(rx);   break;
            case 0x08: printf_CartesianTraj(rx);    break;
            case 0x40: printf_ServerSystemData(rx); break;
            case 0x00: break;
            default: 
                std::cerr << "Invalid MsgType: " << +tmp << std::endl;
                // throw("shit");
                break;
            }
        }
        usleep(1000*1);
    }

    return 0;
}


/*
  ___             _   _            ___       __ _      _ _   _             
 | __|  _ _ _  __| |_(_)___ _ _   |   \ ___ / _(_)_ _ (_) |_(_)___ _ _  ___
 | _| || | ' \/ _|  _| / _ \ ' \  | |) / -_)  _| | ' \| |  _| / _ \ ' \(_-<
 |_| \_,_|_||_\__|\__|_\___/_||_| |___/\___|_| |_|_||_|_|\__|_\___/_||_/__/
                                                                           
*/

void JointParameterSet(const eth::Packet& pkt)
{
    pkt.GetData().j_param->jointTorquePgain[0] = 1.11;
    pkt.GetData().j_param->jointTorquePgain[1] = 2.22;
    pkt.GetData().j_param->jointTorquePgain[2] = 3.33;
    pkt.GetData().j_param->jointTorquePgain[3] = 4.44;

    pkt.GetData().j_param->jointTorqueIgain[0] = 5.55;
    pkt.GetData().j_param->jointTorqueIgain[1] = 6.66;
    pkt.GetData().j_param->jointTorqueIgain[2] = 7.77;
    pkt.GetData().j_param->jointTorqueIgain[3] = 8.88;

    pkt.GetData().j_param->jointTorqueDgain[0] = 9.99;
    pkt.GetData().j_param->jointTorqueDgain[1] = 10.111;
    pkt.GetData().j_param->jointTorqueDgain[2] = 11.111;
    pkt.GetData().j_param->jointTorqueDgain[3] = 12.222;

    pkt.GetData().j_param->jointPositionPgain[0] = 13.33;
    pkt.GetData().j_param->jointPositionPgain[1] = 14.44;
    pkt.GetData().j_param->jointPositionPgain[2] = 15.55;
    pkt.GetData().j_param->jointPositionPgain[3] = 16.6666;

    pkt.GetData().j_param->jointPositionIgain[0] = 17.777;
    pkt.GetData().j_param->jointPositionIgain[1] = 18.888;
    pkt.GetData().j_param->jointPositionIgain[2] = 19.999;
    pkt.GetData().j_param->jointPositionIgain[3] = 20.222;

    pkt.GetData().j_param->jointPositionDgain[0] = 21.111;
    pkt.GetData().j_param->jointPositionDgain[1] = 22.222;
    pkt.GetData().j_param->jointPositionDgain[2] = 23.3333;
    pkt.GetData().j_param->jointPositionDgain[3] = 24.4444;

    pkt.GetData().j_param->jointGravityGain[0] = 25.5555;
    pkt.GetData().j_param->jointGravityGain[1] = 26.6666;
    pkt.GetData().j_param->jointGravityGain[2] = 27.7777;
    pkt.GetData().j_param->jointGravityGain[3] = 28.8888;

    pkt.GetData().j_param->jointFrictionGain[0] = 29.9999;
    pkt.GetData().j_param->jointFrictionGain[1] = 30.1111;
    pkt.GetData().j_param->jointFrictionGain[2] = 31.1111;
    pkt.GetData().j_param->jointFrictionGain[3] = 32.22222;

    pkt.GetData().j_param->jointCurrentGain[0] = 33.33333;
    pkt.GetData().j_param->jointCurrentGain[1] = 34.44444;
    pkt.GetData().j_param->jointCurrentGain[2] = 35.555;
    pkt.GetData().j_param->jointCurrentGain[3] = 36.6666;

    pkt.GetData().j_param->jointConstantTorque[0] = 37.7777;
    pkt.GetData().j_param->jointConstantTorque[1] = 38.8888;
    pkt.GetData().j_param->jointConstantTorque[2] = 39.99999;
    pkt.GetData().j_param->jointConstantTorque[3] = 40.0141;

    pkt.GetData().j_param->jointConstantSpring[0] = 41.1111;
    pkt.GetData().j_param->jointConstantSpring[1] = 42.2222;
    pkt.GetData().j_param->jointConstantSpring[2] = 43.3333;
    pkt.GetData().j_param->jointConstantSpring[3] = 44.4444;

    pkt.GetData().j_param->jointConstantEfficiency[0] = 45.5555;
    pkt.GetData().j_param->jointConstantEfficiency[1] = 46.6666;
    pkt.GetData().j_param->jointConstantEfficiency[2] = 47.777;
    pkt.GetData().j_param->jointConstantEfficiency[3] = 48.8;
}

void JointTrajectorySet(const eth::Packet& pkt)
{
    pkt.GetData().j_traj->JointTrajecotryTime[0] = 200.11;
    pkt.GetData().j_traj->JointTrajecotryTime[1] = 321.11;
    pkt.GetData().j_traj->JointTrajecotryTime[2] = 4631.11;
    pkt.GetData().j_traj->JointTrajecotryTime[3] = 567.11;

    pkt.GetData().j_traj->JointTrajectoryAcc[0] = 3123.156;
    pkt.GetData().j_traj->JointTrajectoryAcc[1] = 76345.156;
    pkt.GetData().j_traj->JointTrajectoryAcc[2] = 364523.156;
    pkt.GetData().j_traj->JointTrajectoryAcc[3] = 234123.156;
}

void CartesianParamSet(const eth::Packet& pkt)
{
    for (int i = 0; i < 3; i++) {
        pkt.GetData().c_param->cartesianPositionPgain[i] = 1.11f * (float)(i + 1);
        pkt.GetData().c_param->cartesianPositionIgain[i] = 2.22f * (float)(i + 1);
        pkt.GetData().c_param->cartesianPositionDgain[i] = 3.33f * (float)(i + 1);
    }
}

void CartesianTrajectorySet(const eth::Packet& pkt)
{
    for (int i = 0; i < 3; i++) {
        pkt.GetData().c_traj->cartesianTrajectoryTime[i] = 1.11f * (float)(i + 1);
        pkt.GetData().c_traj->cartesianTrajectoryAcc[i] = 2.22f * (float)(i + 1);
    }
}

void CartesianTargetSet(const eth::Packet& pkt)
{
    pkt.GetData().c_target->pX = 1.11f;
    pkt.GetData().c_target->pY = 2.22f;
    pkt.GetData().c_target->pZ = 3.33f;
    pkt.GetData().c_target->rX = 4.44f;
    pkt.GetData().c_target->rY = 5.55f;
    pkt.GetData().c_target->rZ = 6.66f;
}

void JointTargetSet(const eth::Packet& pkt)
{
    for (int i = 0; i < 4; i++) {
        pkt.GetData().j_target->jointTarget[i] = 1.111f * (float)(i + 1);
    }
}

void ServerSystemDataSet(const eth::Packet& pkt)
{
	static int32_t count = 0;
	static int32_t logCount = 0;
	pkt.GetData().sys_data->cnt = count;
	pkt.GetData().sys_data->logCnt = logCount;
	pkt.GetData().sys_data->gravityMode = 0;
	pkt.GetData().sys_data->targetReached = 0;
	for (int i = 0; i < 3; i++) {
		pkt.GetData().sys_data->cartesianTargetPose[i] = (float)logCount * 1.11;
		pkt.GetData().sys_data->cartesianCurrentPose[i] = (float)logCount * 2.22;
		pkt.GetData().sys_data->targetTrajectoryTime[i] = (float)logCount * 3.33;
		pkt.GetData().sys_data->targetTrajectoryAcc[i] = (float)logCount * 4.44;
	}

	for (int i = 0; i < 4; i++) {
		pkt.GetData().sys_data->moduleData.actualMotorPosition[i] = (float)count * 1.11;
		pkt.GetData().sys_data->moduleData.actualMotorVelocity[i] = (float)count * 2.22;
		pkt.GetData().sys_data->moduleData.actualLinkPosition[i] = (float)count * 3.33;
		pkt.GetData().sys_data->moduleData.actualLinkVelocity[i] = (float)count * 4.44;
		pkt.GetData().sys_data->moduleData.actualCurrent[i] = (float)count * 5.55;
		pkt.GetData().sys_data->moduleData.modeOfOperation[i] = i;
		pkt.GetData().sys_data->moduleData.statusword[i] = i + 5;
	}
	count++;
	logCount++;

	if (count > 200) {
		count = 0;
	}

}


void printf_jointParam(const eth::Packet& pkt)
{
	printf("\nJoint Parameters Get : \n");
	printf("torque_P\ttorque_I\ttorque_D\tposition_P\tposition_I\tposition_D\tgravity_G\tFriction_G\tCurrent_G\tConstant_T\tConstant_S\tConstant_E\n");
	for (int i = 0; i < 4; i++)
	{
		printf("%f\t", pkt.GetData().j_param->jointTorquePgain[i]);
		printf("%f\t", pkt.GetData().j_param->jointTorqueIgain[i]);
		printf("%f\t", pkt.GetData().j_param->jointTorqueDgain[i]);
		printf("%f\t", pkt.GetData().j_param->jointPositionPgain[i]);
		printf("%f\t", pkt.GetData().j_param->jointPositionIgain[i]);
		printf("%f\t", pkt.GetData().j_param->jointPositionDgain[i]);
		printf("%f\t", pkt.GetData().j_param->jointGravityGain[i]);
		printf("%f\t", pkt.GetData().j_param->jointFrictionGain[i]);
		printf("%f\t", pkt.GetData().j_param->jointCurrentGain[i]);
		printf("%f\t", pkt.GetData().j_param->jointConstantTorque[i]);
		printf("%f\t", pkt.GetData().j_param->jointConstantSpring[i]);
		printf("%f\n", pkt.GetData().j_param->jointConstantEfficiency[i]);
	}
	printf("------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
}

void printf_jointTraj(const eth::Packet& pkt)
{
	printf("\nJoint Trajectory Get : \n");
	printf("jTrajectory_T\tjTrajectory_A\n");
	for (int i = 0; i < 4; i++)
	{
		printf("%f\t", pkt.GetData().j_traj->JointTrajecotryTime[i]);
		printf("%f\n", pkt.GetData().j_traj->JointTrajectoryAcc[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_CartesianParam(const eth::Packet& pkt)
{
	printf("\nCartesian Parameters Get : \n");
	printf("cPosition_P\tcPosition_I\tcPosition_D\n");
	for (int i = 0; i < 3; i++) {
		printf("%f\t", pkt.GetData().c_param->cartesianPositionPgain[i]);
		printf("%f\t", pkt.GetData().c_param->cartesianPositionIgain[i]);
		printf("%f\n", pkt.GetData().c_param->cartesianPositionDgain[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_CartesianTraj(const eth::Packet& pkt)
{
	printf("\nCartesian Trajectory Get : \n");
	printf("cTrajectory_T\tcTrajectory_A\n");
	for (int i = 0; i < 3; i++) {
		printf("%f\t", pkt.GetData().c_traj->cartesianTrajectoryTime[i]);
		printf("%f\n", pkt.GetData().c_traj->cartesianTrajectoryAcc[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_JointTarget(const eth::Packet& pkt)
{
	printf("\nJoint Target Get : \n");
	
	for (int i = 0; i < 4; i++)
	{
		printf("Joint%d\t:\t%f\n", i, pkt.GetData().j_target->jointTarget[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_CartesianTarget(const eth::Packet& pkt)
{
	printf("\nCartesian Target Get : \n");
	printf("pX\t:\t%f\n", pkt.GetData().c_target->pX);
	printf("pY\t:\t%f\n", pkt.GetData().c_target->pY);
	printf("pZ\t:\t%f\n", pkt.GetData().c_target->pZ);
	printf("rX\t:\t%f\n", pkt.GetData().c_target->rX);
	printf("rY\t:\t%f\n", pkt.GetData().c_target->rY);
	printf("rZ\t:\t%f\n", pkt.GetData().c_target->rZ);
	printf("----------------------------------------------\n");
}

void printf_ServerSystemData(const eth::Packet& pkt)
{
	printf("\nServer Data : \n");
	printf("Count : %d\n", pkt.GetData().sys_data->cnt);
	printf("Log Count : %d\n", pkt.GetData().sys_data->logCnt);
	printf("Gravity Mode : %d\n", pkt.GetData().sys_data->gravityMode);
	printf("Target Rechead : %d\n", pkt.GetData().sys_data->targetReached);
	printf("\n");
	printf("cTarget_tP\tcTarget_cP\tcTarget_T\tcTarget_A\n");
	for (int i = 0; i < 3; i++) {
		printf("%f\t", pkt.GetData().sys_data->cartesianTargetPose[i]);
		printf("%f\t", pkt.GetData().sys_data->cartesianCurrentPose[i]);
		printf("%f\t", pkt.GetData().sys_data->targetTrajectoryTime[i]);
		printf("%f\n", pkt.GetData().sys_data->targetTrajectoryAcc[i]);
	}
	printf("\n");
	printf("Motor_P\tMotor_V\tLink_P\tLink_V\tCurrent\tMode\tStatusword\n");
	for (int i = 0; i < 4; i++) {
		printf("%f\t", pkt.GetData().sys_data->moduleData.actualMotorPosition[i]);
		printf("%f\t", pkt.GetData().sys_data->moduleData.actualMotorVelocity[i]);
		printf("%f\t", pkt.GetData().sys_data->moduleData.actualLinkPosition[i]);
		printf("%f\t", pkt.GetData().sys_data->moduleData.actualLinkVelocity[i]);
		printf("%f\t", pkt.GetData().sys_data->moduleData.actualCurrent[i]);
		printf("%d\t", pkt.GetData().sys_data->moduleData.modeOfOperation[i]);
		printf("%d\n", pkt.GetData().sys_data->moduleData.statusword[i]);
	}
	printf("--------------------------------------------------------------------------------------\n");
}
