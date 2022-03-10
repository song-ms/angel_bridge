#ifndef ETHERNET_STRUCTURE_H_
#define ETHERNET_STRUCTURE_H_

/*
 *
 *    Created on : Dec 14, 2021
 *        Author : Chanil
 *
 */

#include <stdint.h>


// typedef int bool;
// #define true  1
// #define false 0


/* Define */
/* ROBOT SETTING STATUS */
#define JOINT_PARAMETER_SET 		0
#define JOINT_TRAJECTORY_SET 		1
#define CARTESIAN_PARAMETER_SET 	2
#define CARTESIAN_TRAJECTORY_SET 	3
#define JOINT_TARGET_SET 			4
#define CARTESIAN_TARGET_SET 		5
#define SERVER_SYSTEM_DAYA 			6
#define NONE 						7

/* COMMUNICATION STATUS */
#define REGISTRATION 				0
#define REGISTRATION_COMPLETE		1
#define SERVO_ON					2
#define SERVO_ON_COMPLETE			3
#define TUNING_STATE 				4
#define TUNING_STATE_COMPLETE		5
#define HOMING 						6
#define HOMING_COMPLETE 			7
#define FREE_STATE					8

/* CONTROL MODE */
#define NONE_MODE					0
#define GRAVITY_MODE 				1
#define JOINT_MODE 					2
#define CARTESIAN_MODE 				3
#define GRAVITY_WITH_JOINT_MODE		4
#define GRAVITY_WITH_CARTESIAN_MODE 5
#define STOP_MODE					6
/* End Define */

#define JOINT_AXIS 					4
#define CARTESIAN_AXIS 				3

/* Structure */
/* COMMUNICATION STRUCTURE */
#pragma pack(push,1)
typedef struct
{
	uint8_t packetType[2];
	uint16_t commState;
	uint16_t payloadSize;
	uint8_t controlMode;
}MsgState;

typedef struct AxisServerData
{
	int32_t digitalInput[JOINT_AXIS];
	int32_t analogInput[JOINT_AXIS];
	float actualMotorPosition[JOINT_AXIS];
	float actualMotorVelocity[JOINT_AXIS];
	float actualLinkPosition[JOINT_AXIS];
	float actualLinkVelocity[JOINT_AXIS];
	float actualCurrent[JOINT_AXIS];
	float targetPosition[JOINT_AXIS];
	float targetCurrent[JOINT_AXIS];
	int32_t modeOfOperation[JOINT_AXIS];
	int32_t statusword[JOINT_AXIS];
	int16_t actualIMUroll[JOINT_AXIS];
	int16_t actualIMUpitch[JOINT_AXIS];
	int16_t actualIMUyaw[JOINT_AXIS];
	int16_t actualIMUpx[JOINT_AXIS];
	int16_t actualIMUpy[JOINT_AXIS];
	int16_t actualIMUpz[JOINT_AXIS];
}Axis;

typedef struct
{
	int32_t cnt;
	int32_t logCnt;
	int32_t gravityMode;
	int32_t targetReached;
	// Cartesian Position Info;
	float cartesianTargetPose[CARTESIAN_AXIS];
	float cartesianCurrentPose[CARTESIAN_AXIS];
	// Cartesian Trajectory Info
	float targetTrajectoryTime[CARTESIAN_AXIS];
	float targetTrajectoryAcc[CARTESIAN_AXIS];
	// Module Data (Motor Driver)
	Axis moduleData;
}ServerSystemData;

typedef struct
{
	float jointPositionPgain[JOINT_AXIS];
	float jointPositionIgain[JOINT_AXIS];
	float jointPositionDgain[JOINT_AXIS];

	float jointTorquePgain[JOINT_AXIS];
	float jointTorqueIgain[JOINT_AXIS];
	float jointTorqueDgain[JOINT_AXIS];

	float jointConstantEfficiency[JOINT_AXIS];
	float jointConstantTorque[JOINT_AXIS];
	float jointConstantSpring[JOINT_AXIS];

	float jointGravityGain[JOINT_AXIS];
	float jointCurrentGain[JOINT_AXIS];
	float jointFrictionGain[JOINT_AXIS];
}JointParameterSettingStruct;

typedef struct
{
	float JointTrajecotryTime[JOINT_AXIS];
	float JointTrajectoryAcc[JOINT_AXIS];
}JointTrajectorySetStruct;

typedef struct
{
	float cartesianPositionPgain[CARTESIAN_AXIS];
	float cartesianPositionIgain[CARTESIAN_AXIS];
	float cartesianPositionDgain[CARTESIAN_AXIS];
}CartesianParameterSettingStruct;

typedef struct
{
	float cartesianTrajectoryTime[CARTESIAN_AXIS];
	float cartesianTrajectoryAcc[CARTESIAN_AXIS];
}CartesianTrajectorySetStruct;

typedef struct
{
	float jointTarget[JOINT_AXIS];
}JointTargetStruct;

typedef struct
{
	/* Left Leg */
	float leftPoseX;
	float leftPoseY;
	float leftPoseZ;
	float leftRoll;
	float leftPitch;
	float leftYaw;
	/* right Leg */
	float rightPoseX;
	float rightPoseY;
	float rightPoseZ;
	float rightRoll;
	float rightPitch;
	float rightYaw;
}CartesianTargetStruct;
#pragma pack(pop)
/* End Structure */


#endif //ETHERNET_STRUCTURE_H_