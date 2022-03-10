#pragma once
#include "ethernet_structure.h"
#include "ethernet_parser.h"

#define METHOD_FIX 1
#define METHOD_CONTINUOUS 2

/* extern */
extern struct MsgState* MsgStateSend;
extern struct MsgState* MsgStateRecv;

extern struct ServerSystemData* ServerData;

extern struct JointParameterSettingStruct* jParamSet;
extern struct JointParameterSettingStruct* jParamGet;

extern struct JointTrajectorySetStruct* jTrajSet;
extern struct JointTrajectorySetStruct* jTrajGet;

extern struct CartesianParameterSettingStruct* cParamSet;
extern struct CartesianParameterSettingStruct* cParamGet;

extern struct CartesianTrajectorySetStruct* cTrajSet;
extern struct CartesianTrajectorySetStruct* cTrajGet;

extern struct JointTargetStruct* jTargetSet;
extern struct JointTargetStruct* jTargetGet;

extern struct CartesianTargetStruct* cTargetSet;
extern struct CartesianTargetStruct* cTargetGet;
/* End extern */

/* Function declaration */
void JointParameterSet(int method);
void JointTrajectorySet(int method);
void CartesianParamSet(int method);
void CartesianTrajectorySet(int method);
void CartesianTargetSet(int method);
void JointTargetSet(int method);
void ServerSystemDataSet();

void printf_jointTraj();
void printf_jointParam();
void printf_CartesianParam();
void printf_CartesianTraj();
void printf_JointTarget();
void printf_CartesianTarget();
void printf_ServerSystemData();
/* End Function declaration */

void JointParameterSet(int method)
{
	if (method == METHOD_FIX) {
		jParamSet->jointTorquePgain[0] = 1.11;
		jParamSet->jointTorquePgain[1] = 2.22;
		jParamSet->jointTorquePgain[2] = 3.33;
		jParamSet->jointTorquePgain[3] = 4.44;

		jParamSet->jointTorqueIgain[0] = 5.55;
		jParamSet->jointTorqueIgain[1] = 6.66;
		jParamSet->jointTorqueIgain[2] = 7.77;
		jParamSet->jointTorqueIgain[3] = 8.88;

		jParamSet->jointTorqueDgain[0] = 9.99;
		jParamSet->jointTorqueDgain[1] = 10.111;
		jParamSet->jointTorqueDgain[2] = 11.111;
		jParamSet->jointTorqueDgain[3] = 12.222;

		jParamSet->jointPositionPgain[0] = 13.33;
		jParamSet->jointPositionPgain[1] = 14.44;
		jParamSet->jointPositionPgain[2] = 15.55;
		jParamSet->jointPositionPgain[3] = 16.6666;

		jParamSet->jointPositionIgain[0] = 17.777;
		jParamSet->jointPositionIgain[1] = 18.888;
		jParamSet->jointPositionIgain[2] = 19.999;
		jParamSet->jointPositionIgain[3] = 20.222;

		jParamSet->jointPositionDgain[0] = 21.111;
		jParamSet->jointPositionDgain[1] = 22.222;
		jParamSet->jointPositionDgain[2] = 23.3333;
		jParamSet->jointPositionDgain[3] = 24.4444;

		jParamSet->jointGravityGain[0] = 25.5555;
		jParamSet->jointGravityGain[1] = 26.6666;
		jParamSet->jointGravityGain[2] = 27.7777;
		jParamSet->jointGravityGain[3] = 28.8888;

		jParamSet->jointFrictionGain[0] = 29.9999;
		jParamSet->jointFrictionGain[1] = 30.1111;
		jParamSet->jointFrictionGain[2] = 31.1111;
		jParamSet->jointFrictionGain[3] = 32.22222;

		jParamSet->jointCurrentGain[0] = 33.33333;
		jParamSet->jointCurrentGain[1] = 34.44444;
		jParamSet->jointCurrentGain[2] = 35.555;
		jParamSet->jointCurrentGain[3] = 36.6666;

		jParamSet->jointConstantTorque[0] = 37.7777;
		jParamSet->jointConstantTorque[1] = 38.8888;
		jParamSet->jointConstantTorque[2] = 39.99999;
		jParamSet->jointConstantTorque[3] = 40.0141;

		jParamSet->jointConstantSpring[0] = 41.1111;
		jParamSet->jointConstantSpring[1] = 42.2222;
		jParamSet->jointConstantSpring[2] = 43.3333;
		jParamSet->jointConstantSpring[3] = 44.4444;

		jParamSet->jointConstantEfficiency[0] = 45.5555;
		jParamSet->jointConstantEfficiency[1] = 46.6666;
		jParamSet->jointConstantEfficiency[2] = 47.777;
		jParamSet->jointConstantEfficiency[3] = 48.8;
	}
	else if (method == METHOD_CONTINUOUS) {

	}
}

void JointTrajectorySet(int method)
{
	if (method == METHOD_FIX) {
		jTrajSet->JointTrajecotryTime[0] = 200.11;
		jTrajSet->JointTrajecotryTime[1] = 321.11;
		jTrajSet->JointTrajecotryTime[2] = 4631.11;
		jTrajSet->JointTrajecotryTime[3] = 567.11;

		jTrajSet->JointTrajectoryAcc[0] = 3123.156;
		jTrajSet->JointTrajectoryAcc[1] = 76345.156;
		jTrajSet->JointTrajectoryAcc[2] = 364523.156;
		jTrajSet->JointTrajectoryAcc[3] = 234123.156;
	}
	else if (method == METHOD_CONTINUOUS) {

	}
}

void CartesianParamSet(int method)
{
	if (method == METHOD_FIX) {
		for (int i = 0; i < 3; i++) {
			cParamSet->cartesianPositionPgain[i] = 1.11f * (float)(i + 1);
			cParamSet->cartesianPositionIgain[i] = 2.22f * (float)(i + 1);
			cParamSet->cartesianPositionDgain[i] = 3.33f * (float)(i + 1);
		}
	}
	else if (method == METHOD_CONTINUOUS) {
		for (int i = 0; i < 3; i++) {
			cParamSet->cartesianPositionPgain[i] = 1.11f * (float)(i + 1);
			cParamSet->cartesianPositionIgain[i] = 2.22f * (float)(i + 1);
			cParamSet->cartesianPositionDgain[i] = 3.33f * (float)(i + 1);
		}
	}
}

void CartesianTrajectorySet(int method)
{
	if (method == METHOD_FIX) {
		for (int i = 0; i < 3; i++) {
			cTrajSet->cartesianTrajectoryTime[i] = 1.11f * (float)(i + 1);
			cTrajSet->cartesianTrajectoryAcc[i] = 2.22f * (float)(i + 1);
		}
	}
	else if (method == METHOD_CONTINUOUS) {

	}
}

void CartesianTargetSet(int method)
{
	if (method == METHOD_FIX)
	{
		cTargetSet->pX = 1.11f;
		cTargetSet->pY = 2.22f;
		cTargetSet->pZ = 3.33f;
		cTargetSet->rX = 4.44f;
		cTargetSet->rY = 5.55f;
		cTargetSet->rZ = 6.66f;
	}
	else if (method == METHOD_CONTINUOUS)
	{

	}
}

void JointTargetSet(int method)
{
	if (method == METHOD_FIX)
	{
		for (int i = 0; i < 4; i++) {
			jTargetSet->jointTarget[i] = 1.111f * (float)(i + 1);
		}
	}
	else if (method == METHOD_CONTINUOUS)
	{

	}
}

void ServerSystemDataSet()
{
	static int32_t count = 0;
	static int32_t logCount = 0;
	ServerData->cnt = count;
	ServerData->logCnt = logCount;
	ServerData->gravityMode = 0;
	ServerData->targetReached = 0;
	for (int i = 0; i < 3; i++) {
		ServerData->cartesianTargetPose[i] = (float)logCount * 1.11;
		ServerData->cartesianCurrentPose[i] = (float)logCount * 2.22;
		ServerData->targetTrajectoryTime[i] = (float)logCount * 3.33;
		ServerData->targetTrajectoryAcc[i] = (float)logCount * 4.44;
	}

	for (int i = 0; i < 4; i++) {
		ServerData->moduleData.actualMotorPosition[i] = (float)count * 1.11;
		ServerData->moduleData.actualMotorVelocity[i] = (float)count * 2.22;
		ServerData->moduleData.actualLinkPosition[i] = (float)count * 3.33;
		ServerData->moduleData.actualLinkVelocity[i] = (float)count * 4.44;
		ServerData->moduleData.actualCurrent[i] = (float)count * 5.55;
		ServerData->moduleData.modeOfOperation[i] = i;
		ServerData->moduleData.statusword[i] = i + 5;
	}
	count++;
	logCount++;

	if (count > 200) {
		count = 0;
	}

}


void printf_jointParam()
{
	printf("\nJoint Parameters Get : \n");
	printf("torque_P\ttorque_I\ttorque_D\tposition_P\tposition_I\tposition_D\tgravity_G\tFriction_G\tCurrent_G\tConstant_T\tConstant_S\tConstant_E\n");
	for (int i = 0; i < 4; i++)
	{
		printf("%f\t", jParamGet->jointTorquePgain[i]);
		printf("%f\t", jParamGet->jointTorqueIgain[i]);
		printf("%f\t", jParamGet->jointTorqueDgain[i]);
		printf("%f\t", jParamGet->jointPositionPgain[i]);
		printf("%f\t", jParamGet->jointPositionIgain[i]);
		printf("%f\t", jParamGet->jointPositionDgain[i]);
		printf("%f\t", jParamGet->jointGravityGain[i]);
		printf("%f\t", jParamGet->jointFrictionGain[i]);
		printf("%f\t", jParamGet->jointCurrentGain[i]);
		printf("%f\t", jParamGet->jointConstantTorque[i]);
		printf("%f\t", jParamGet->jointConstantSpring[i]);
		printf("%f\n", jParamGet->jointConstantEfficiency[i]);
	}
	printf("------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
}

void printf_jointTraj()
{
	printf("\nJoint Trajectory Get : \n");
	printf("jTrajectory_T\tjTrajectory_A\n");
	for (int i = 0; i < 4; i++)
	{
		printf("%f\t", jTrajGet->JointTrajecotryTime[i]);
		printf("%f\n", jTrajGet->JointTrajectoryAcc[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_CartesianParam()
{
	printf("\nCartesian Parameters Get : \n");
	printf("cPosition_P\tcPosition_I\tcPosition_D\n");
	for (int i = 0; i < 3; i++) {
		printf("%f\t", cParamGet->cartesianPositionPgain[i]);
		printf("%f\t", cParamGet->cartesianPositionIgain[i]);
		printf("%f\n", cParamGet->cartesianPositionDgain[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_CartesianTraj()
{
	printf("\nCartesian Trajectory Get : \n");
	printf("cTrajectory_T\tcTrajectory_A\n");
	for (int i = 0; i < 3; i++) {
		printf("%f\t", cTrajGet->cartesianTrajectoryTime[i]);
		printf("%f\n", cTrajGet->cartesianTrajectoryAcc[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_JointTarget()
{
	printf("\nJoint Target Get : \n");
	
	for (int i = 0; i < 4; i++)
	{
		printf("Joint%d\t:\t%f\n", i, jTargetGet->jointTarget[i]);
	}
	printf("----------------------------------------------\n");
}

void printf_CartesianTarget()
{
	printf("\nCartesian Target Get : \n");
	printf("pX\t:\t%f\n", cTargetGet->pX);
	printf("pY\t:\t%f\n", cTargetGet->pY);
	printf("pZ\t:\t%f\n", cTargetGet->pZ);
	printf("rX\t:\t%f\n", cTargetGet->rX);
	printf("rY\t:\t%f\n", cTargetGet->rY);
	printf("rZ\t:\t%f\n", cTargetGet->rZ);
	printf("----------------------------------------------\n");
}

void printf_ServerSystemData()
{
	printf("\nServer Data : \n");
	printf("Count : %d\n", ServerData->cnt);
	printf("Log Count : %d\n", ServerData->logCnt);
	printf("Gravity Mode : %d\n", ServerData->gravityMode);
	printf("Target Rechead : %d\n", ServerData->targetReached);
	printf("\n");
	printf("cTarget_tP\tcTarget_cP\tcTarget_T\tcTarget_A\n");
	for (int i = 0; i < 3; i++) {
		printf("%f\t", ServerData->cartesianTargetPose[i]);
		printf("%f\t", ServerData->cartesianCurrentPose[i]);
		printf("%f\t", ServerData->targetTrajectoryTime[i]);
		printf("%f\n", ServerData->targetTrajectoryAcc[i]);
	}
	printf("\n");
	printf("Motor_P\tMotor_V\tLink_P\tLink_V\tCurrent\tMode\tStatusword\n");
	for (int i = 0; i < 4; i++) {
		printf("%f\t", ServerData->moduleData.actualMotorPosition[i]);
		printf("%f\t", ServerData->moduleData.actualMotorVelocity[i]);
		printf("%f\t", ServerData->moduleData.actualLinkPosition[i]);
		printf("%f\t", ServerData->moduleData.actualLinkVelocity[i]);
		printf("%f\t", ServerData->moduleData.actualCurrent[i]);
		printf("%d\t", ServerData->moduleData.modeOfOperation[i]);
		printf("%d\n", ServerData->moduleData.statusword[i]);
	}
	printf("--------------------------------------------------------------------------------------\n");
}
