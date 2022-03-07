#ifndef ETHERNET_PARSER_H_
#define ETHERNET_PARSER_H_
#include "ethernet_structure.h"

#include <malloc.h>

//#ifdef WIN32
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//#endif

/* Variable Declaration */
int MsgStateSize = 0;
int AxisServerDataSize = 0;
int ServerSystemDataSize = 0;
int JointParamSetStructSize = 0;
int JointTrajSetStructSize = 0;
int CartesianParamSetStructSize = 0;
int CartesianTrajSetStructSize = 0;
int JointTargetStructSize = 0;
int CartesianTargetStructSize = 0;
int totalSize = 0;

MsgState* MsgStateSend;
MsgState* MsgStateRecv;

ServerSystemData* ServerData;

JointParameterSettingStruct* jParamSet;
JointParameterSettingStruct* jParamGet;

JointTrajectorySetStruct* jTrajSet;
JointTrajectorySetStruct* jTrajGet;

CartesianParameterSettingStruct* cParamSet;
CartesianParameterSettingStruct* cParamGet;

CartesianTrajectorySetStruct* cTrajSet;
CartesianTrajectorySetStruct* cTrajGet;

JointTargetStruct* jTargetSet;
JointTargetStruct* jTargetGet;

CartesianTargetStruct* cTargetSet;
CartesianTargetStruct* cTargetGet;
/* End Variable Declaration */

/* Bit Pattern */
unsigned char bitPattern[8] = {
	0x01, 0x02, 0x04, 0x08,			// 10000000, ........
	0x10, 0x20, 0x40, 0x80			// ........, 00000001
};
/* End Bit Pattern*/

/* Setting Value */
int ROBOT_AXIS = 4;
int CARTESIAN_AXIS = 6;
/* End Setting Value */

/* Buffer Declaration */
int8_t* msgRecvBuffer;
int8_t* sendDataBuffer;
int8_t* recvDataBuffer;
/* End Buffer Declaration */

/* Function Declaration */
void MsgStateInit(MsgState* _msgStatePtr);
void MsgStateRelease(MsgState* _msgStatePtr);
bool ServerSystemDataInit(ServerSystemData* _msgDataPtr);
bool SeverSystemDataRelease(ServerSystemData* _msgDataPtr);
bool JointParameterSettingStructInit(JointParameterSettingStruct* _msgDataPtr);
bool JointParameterSettingStructRelease(JointParameterSettingStruct* _msgDataPtr);
bool JointTrajectorySetStructInit(JointTrajectorySetStruct* _msgDataPtr);
bool JointTrajectorySetStructRelease(JointTrajectorySetStruct* _msgDataPtr);
bool CartesianParameterSettingStructInit(CartesianParameterSettingStruct* _msgDataPtr);
bool CartesianParameterSettingStructRelease(CartesianParameterSettingStruct* _msgDataPtr);
bool CartesianTrajectorySetStructInit(CartesianTrajectorySetStruct* _msgDataPtr);
bool CartesianTrajectorySetStructRelease(CartesianTrajectorySetStruct* _msgDataPtr);
bool JointTargetStructInit(JointTargetStruct* _msgDataPtr);
bool JointTargetStructRelease(JointTargetStruct* _msgDataPtr);
void CartesianTargetStructInit(CartesianTargetStruct* _msgDataPtr);
void CartesianTargetStructRelease(CartesianTargetStruct* _msgDataPtr);

bool EtherNetStructure_Init();
bool MsgStateToBuffer(MsgState* _msgStatePtr, int8_t* buff);
bool ServerSystemDataToBuffer(ServerSystemData* _msgDataPtr, int8_t* buff);
bool JointParameterSettingStructToBuffer(JointParameterSettingStruct* _msgDataPtr, int8_t* buff);
bool JointTrajectorySetStructToBuffer(JointTrajectorySetStruct* _msgDataPtr, int8_t* buff);
bool CartesianParameterSettingStructToBuffer(CartesianParameterSettingStruct* _msgDataPtr, int8_t* buff);
bool CartesianTrajectorySetStructToBuffer(CartesianTrajectorySetStruct* _msgDataPtr, int8_t* buff);
bool JointTargetStructToBuffer(JointTargetStruct* _msgDataPtr, int8_t* buff);
bool CartesianTargetStructToBuffer(CartesianTargetStruct* _msgDataPtr, int8_t* buff);
bool BufferInit();

bool DataGethering2
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buffOut
);
bool DataGethering3
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buffOut
);
bool DataGethering4
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buffOut
);
bool DataGethering5
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buffOut
);
bool DataGethering6
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buff6,
	int8_t* buffOut
);
bool DataGethering7
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buff6,
	int8_t* buff7,
	int8_t* buffOut
);
bool DataGethering8
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buff6,
	int8_t* buff7,
	int8_t* buff8,
	int8_t* buffOut
);

bool DataDivideMsgState
(
	int8_t* inputBuff,
	MsgState* outputStruct
);
bool DataDivideJointParamSettingStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	JointParameterSettingStruct* outputStruct
);
bool DataDivideJointTrajectorySetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	JointTrajectorySetStruct* outputStruct
);
bool DataDivideCartesianParameterSettingStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	CartesianParameterSettingStruct* outputStruct
);
bool DataDivideCartesianTrajectorySetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	CartesianTrajectorySetStruct* outputStruct
);
bool DataDivideJointTaregetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	JointTargetStruct* outputStruct
);
bool DataDivideJointCartesianTargetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	CartesianTargetStruct* outputStruct
);
bool DataDivideJointServerSystemData
(
	int8_t* inputBuff,
	MsgState* msg,
	ServerSystemData* outputStruct
);

/* End Function Declaration */


/* Initialize & Release Structure  */
void MsgStateInit(MsgState* _msgStatePtr)
{
	_msgStatePtr->packetType[0] = 0;
	_msgStatePtr->packetType[1] = 0;
	_msgStatePtr->commState = REGISTRATION;
	_msgStatePtr->payloadSize = 0;
	_msgStatePtr->controlMode = NONE_MODE;
	MsgStateSize = sizeof(MsgState);
}

void MsgStateRelease(MsgState* _msgStatePtr)
{
	free(_msgStatePtr);
}

bool ServerSystemDataInit(ServerSystemData* _msgDataPtr)
{
	_msgDataPtr->cnt = 0;
	_msgDataPtr->logCnt = 0;
	_msgDataPtr->gravityMode = 0;
	_msgDataPtr->targetReached = 0;
	for (int i = 0; i < 3; i++) {
		_msgDataPtr->cartesianTargetPose[i] = 0.0f;
		_msgDataPtr->cartesianCurrentPose[i] = 0.0f;
		_msgDataPtr->targetTrajectoryTime[i] = 0.0f;
		_msgDataPtr->targetTrajectoryAcc[i] = 0.0f;
	}
	
	for (int i = 0; i < 4; i++) {
		_msgDataPtr->moduleData.actualMotorPosition[i] = 0.0f;
		_msgDataPtr->moduleData.actualMotorVelocity[i] = 0.0f;
		_msgDataPtr->moduleData.actualLinkPosition[i] = 0.0f;
		_msgDataPtr->moduleData.actualLinkVelocity[i] = 0.0f;
		_msgDataPtr->moduleData.actualCurrent[i] = 0.0f;
		_msgDataPtr->moduleData.targetPosition[i] = 0.0f;
		_msgDataPtr->moduleData.targetCurrent[i] = 0.0f;
		_msgDataPtr->moduleData.modeOfOperation[i] = 0;
		_msgDataPtr->moduleData.statusword[i] = 0;
	}
	ServerSystemDataSize = sizeof(ServerSystemData);
	return true;
}

bool SeverSystemDataRelease(ServerSystemData* _msgDataPtr)
{
	free(_msgDataPtr);
	return true;
}

bool JointParameterSettingStructInit(JointParameterSettingStruct* _msgDataPtr)
{
	for (int i = 0; i < 4; i++) {
		_msgDataPtr->jointPositionPgain[i] = 0.0f;
		_msgDataPtr->jointPositionIgain[i] = 0.0f;
		_msgDataPtr->jointPositionDgain[i] = 0.0f;

		_msgDataPtr->jointTorquePgain[i] = 0.0f;
		_msgDataPtr->jointTorqueIgain[i] = 0.0f;
		_msgDataPtr->jointTorqueDgain[i] = 0.0f;

		_msgDataPtr->jointConstantEfficiency[i] = 0.0f;
		_msgDataPtr->jointConstantTorque[i] = 0.0f;
		_msgDataPtr->jointConstantSpring[i] = 0.0f;

		_msgDataPtr->jointGravityGain[i] = 0.0f;
		_msgDataPtr->jointCurrentGain[i] = 0.0f;
		_msgDataPtr->jointFrictionGain[i] = 0.0f;
	}
	JointParamSetStructSize = sizeof(JointParameterSettingStruct);

	return true;
}

bool JointParameterSettingStructRelease(JointParameterSettingStruct* _msgDataPtr)
{
	free(_msgDataPtr);
	return true;
}

bool JointTrajectorySetStructInit(JointTrajectorySetStruct* _msgDataPtr)
{
	for (int i = 0; i < 4; i++){
		_msgDataPtr->JointTrajecotryTime[i] = 0.0f;
		_msgDataPtr->JointTrajectoryAcc[i] = 0.0f;
	}
	JointTrajSetStructSize = sizeof(JointTrajectorySetStruct);
	return true;
}

bool JointTrajectorySetStructRelease(JointTrajectorySetStruct* _msgDataPtr)
{
	free(_msgDataPtr);
	return true;
}

bool CartesianParameterSettingStructInit(CartesianParameterSettingStruct* _msgDataPtr)
{
	for (int i = 0; i < 3; i++) {
		_msgDataPtr->cartesianPositionPgain[i] = 0.0f;
		_msgDataPtr->cartesianPositionIgain[i] = 0.0f;
		_msgDataPtr->cartesianPositionDgain[i] = 0.0f;
	}
	CartesianParamSetStructSize = sizeof(CartesianParameterSettingStruct);
	return true;
}

bool CartesianParameterSettingStructRelease(CartesianParameterSettingStruct* _msgDataPtr)
{
	free(_msgDataPtr);
	return true;
}

bool CartesianTrajectorySetStructInit(CartesianTrajectorySetStruct* _msgDataPtr)
{
	for (int i = 0; i < 3; i++) {
		_msgDataPtr->cartesianTrajectoryTime[i] = 0.0f;
		_msgDataPtr->cartesianTrajectoryAcc[i] = 0.0f;
	}
	CartesianTrajSetStructSize = sizeof(CartesianTrajectorySetStruct);
	return true;
}

bool CartesianTrajectorySetStructRelease(CartesianTrajectorySetStruct* _msgDataPtr)
{
	free(_msgDataPtr);
	return true;
}

bool JointTargetStructInit(JointTargetStruct* _msgDataPtr)
{
	for (int i = 0; i < 4; i++) {
		_msgDataPtr->jointTarget[i] = 0.0f;
	}
	JointTargetStructSize = sizeof(JointTargetStruct);
	return true;
}

bool JointTargetStructRelease(JointTargetStruct* _msgDataPtr)
{
	free(_msgDataPtr);
	return true;
}

void CartesianTargetStructInit(CartesianTargetStruct* _msgDataPtr)
{
	_msgDataPtr->pX = 0.0f;
	_msgDataPtr->pY = 0.0f;
	_msgDataPtr->pZ = 0.0f;
	_msgDataPtr->rX = 0.0f;
	_msgDataPtr->rY = 0.0f;
	_msgDataPtr->rZ = 0.0f;
	CartesianTargetStructSize = sizeof(CartesianTargetStruct);
}

void CartesianTargetStructRelease(CartesianTargetStruct* _msgDataPtr)
{
	free(_msgDataPtr);
}
/* Initialize & Release Structure End */

/* All Variable Initialize */
bool EtherNetStructure_Init()
{
	/* Send EtherNetStructure Init */
	MsgStateSend = (MsgState*)malloc(sizeof(MsgState));
	jParamSet = (JointParameterSettingStruct*)malloc(sizeof(JointParameterSettingStruct));
	jTrajSet = (JointTrajectorySetStruct*)malloc(sizeof(JointTrajectorySetStruct));
	cParamSet = (CartesianParameterSettingStruct*)malloc(sizeof(CartesianParameterSettingStruct));
	cTrajSet = (CartesianTrajectorySetStruct*)malloc(sizeof(CartesianTrajectorySetStruct));
	jTargetSet = (JointTargetStruct*)malloc(sizeof(JointTargetStruct));
	cTargetSet = (CartesianTargetStruct*)malloc(sizeof(CartesianTargetStruct));

	MsgStateInit(MsgStateSend);
	if (!JointParameterSettingStructInit(jParamSet)) return false;
	if (!JointTrajectorySetStructInit(jTrajSet)) return false;
	if (!CartesianParameterSettingStructInit(cParamSet)) return false;
	if (!CartesianTrajectorySetStructInit(cTrajSet)) return false;
	if (!JointTargetStructInit(jTargetSet)) return false;
	CartesianTargetStructInit(cTargetSet);
	/* End Send EtherNetStructure Init */

	/* Recv EtherNetStructure Init */
	MsgStateRecv = (MsgState*)malloc(sizeof(MsgState));
	ServerData = (ServerSystemData*)malloc(sizeof(ServerSystemData));
	jParamGet = (JointParameterSettingStruct*)malloc(sizeof(JointParameterSettingStruct));
	jTrajGet = (JointTrajectorySetStruct*)malloc(sizeof(JointTrajectorySetStruct));
	cParamGet = (CartesianParameterSettingStruct*)malloc(sizeof(CartesianParameterSettingStruct));
	cTrajGet = (CartesianTrajectorySetStruct*)malloc(sizeof(CartesianTrajectorySetStruct));
	jTargetGet = (JointTargetStruct*)malloc(sizeof(JointTargetStruct));
	cTargetGet = (CartesianTargetStruct*)malloc(sizeof(CartesianTargetStruct));

	MsgStateInit(MsgStateRecv);
	if (!ServerSystemDataInit(ServerData)) return false;
	if (!JointParameterSettingStructInit(jParamGet)) return false;
	if (!JointTrajectorySetStructInit(jTrajGet)) return false;
	if (!CartesianParameterSettingStructInit(cParamGet)) return false;
	if (!CartesianTrajectorySetStructInit(cTrajGet)) return false;
	if (!JointTargetStructInit(jTargetGet)) return false;
	CartesianTargetStructInit(cTargetGet);
	/* End Recv EtherNetStructure Init */

	/* Size Get */
	totalSize = MsgStateSize + 
		JointParamSetStructSize + 
		JointTargetStructSize + 
		CartesianParamSetStructSize + 
		CartesianTrajSetStructSize + 
		JointTrajSetStructSize + 
		CartesianTargetStructSize +
		ServerSystemDataSize;
	/* End Size Get */
	return true;
}
/* End All Variable Initialize */

/* Convert to Buffer */
bool MsgStateToBuffer(MsgState* _msgStatePtr, int8_t* buff)
{
	if (buff == NULL)
	{
		buff = (int8_t*)calloc(MsgStateSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, MsgStateSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgStatePtr, MsgStateSize);
	return true;
}

bool ServerSystemDataToBuffer(ServerSystemData* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL)
	{
		buff = (int8_t*)calloc(ServerSystemDataSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, ServerSystemDataSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, ServerSystemDataSize);
	return true;
}

bool JointParameterSettingStructToBuffer(JointParameterSettingStruct* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL)
	{
		buff = (int8_t*)calloc(JointParamSetStructSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, JointParamSetStructSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, JointParamSetStructSize);
	return true;
}

bool JointTrajectorySetStructToBuffer(JointTrajectorySetStruct* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL)
	{
		buff = (int8_t*)calloc(JointTrajSetStructSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, JointTrajSetStructSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, JointTrajSetStructSize);
	return true;
}

bool CartesianParameterSettingStructToBuffer(CartesianParameterSettingStruct* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL)
	{
		buff = (int8_t*)calloc(CartesianParamSetStructSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, CartesianParamSetStructSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, CartesianParamSetStructSize);
	return true;
}

bool CartesianTrajectorySetStructToBuffer(CartesianTrajectorySetStruct* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL)
	{
		buff = (int8_t*)calloc(CartesianTrajSetStructSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, CartesianTrajSetStructSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, CartesianTrajSetStructSize);
	return true;
}

bool JointTargetStructToBuffer(JointTargetStruct* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL) {
		buff = (int8_t*)calloc(JointTargetStructSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, JointTargetStructSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, JointTargetStructSize);
	return true;
}

bool CartesianTargetStructToBuffer(CartesianTargetStruct* _msgDataPtr, int8_t* buff)
{
	if (buff == NULL) {
		buff = (int8_t*)calloc(CartesianTargetStructSize, sizeof(int8_t));
		if (buff == NULL) {
			return false;
		}
	}
	/*
	else {
		buff = (int8_t*)realloc(buff, CartesianTargetStructSize);
		if (buff == NULL) {
			return false;
		}
	}
	*/
	memcpy(buff, _msgDataPtr, CartesianTargetStructSize);
	return true;
}
/* End Convert To Buffer*/

/* Buffer Initialize */
bool BufferInit()
{
	if (msgRecvBuffer == NULL) {
		msgRecvBuffer = (int8_t*)calloc(MsgStateSize, sizeof(int8_t));
		if (msgRecvBuffer == NULL) {
			return false;
		}
	}
	else {
		msgRecvBuffer = (int8_t*)realloc(msgRecvBuffer, MsgStateSize);
		if (msgRecvBuffer == NULL) {
			return false;
		}
	}


	if (sendDataBuffer == NULL) {
		sendDataBuffer = (int8_t*)calloc(1, sizeof(int8_t));
		if (sendDataBuffer == NULL) {
			return false;
		}
	}
	else {
		sendDataBuffer = (int8_t*)realloc(sendDataBuffer, sizeof(int8_t));
		if (sendDataBuffer == NULL) {
			return false;
		}
	}

	if (recvDataBuffer == NULL) {
		recvDataBuffer = (int8_t*)calloc(MsgStateSize, sizeof(int8_t));
		if (recvDataBuffer == NULL) {
			return false;
		}
	}
	else {
		recvDataBuffer = (int8_t*)realloc(recvDataBuffer, MsgStateSize);
		if (recvDataBuffer == NULL) {
			return false;
		}
	}

	return true;
}
/* End Buffer Initialize */

/* Function for Data Gethering */
bool DataGethering2
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	return true;
}

bool DataGethering3
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2), buff3,  malloc_usable_size(buff3));

	return true;
}

bool DataGethering4
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2), buff3,  malloc_usable_size(buff3));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3), buff4,  malloc_usable_size(buff4));
	return true;
}

bool DataGethering5
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2), buff3,  malloc_usable_size(buff3));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3), buff4,  malloc_usable_size(buff4));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4), buff5,  malloc_usable_size(buff5));
	return true;
}

bool DataGethering6
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buff6,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2), buff3,  malloc_usable_size(buff3));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3), buff4,  malloc_usable_size(buff4));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4), buff5,  malloc_usable_size(buff5));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5), buff6,  malloc_usable_size(buff6));
	return true;
}

bool DataGethering7
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buff6,
	int8_t* buff7,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6) +  malloc_usable_size(buff7);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6) +  malloc_usable_size(buff7);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2), buff3,  malloc_usable_size(buff3));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3), buff4,  malloc_usable_size(buff4));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4), buff5,  malloc_usable_size(buff5));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5), buff6,  malloc_usable_size(buff6));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6), buff7,  malloc_usable_size(buff7));
	return true;
}

bool DataGethering8
(
	int8_t* buff1,
	int8_t* buff2,
	int8_t* buff3,
	int8_t* buff4,
	int8_t* buff5,
	int8_t* buff6,
	int8_t* buff7,
	int8_t* buff8,
	int8_t* buffOut
)
{
	if (buffOut == NULL) {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6) +  malloc_usable_size(buff7) +  malloc_usable_size(buff8);
		buffOut = (int8_t*)calloc(size, sizeof(int8_t));
		if (buffOut == NULL) {
			return false;
		}
	}
	/*
	else {
		int size =  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6) +  malloc_usable_size(buff7) +  malloc_usable_size(buff8);
		buffOut = (int8_t*)realloc(buffOut, size);
		if (buffOut == NULL) {
			return false;
		}
	}
	*/

	memcpy(buffOut, buff1,  malloc_usable_size(buff1));
	memcpy(buffOut +  malloc_usable_size(buff1), buff2,  malloc_usable_size(buff2));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2), buff3,  malloc_usable_size(buff3));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3), buff4,  malloc_usable_size(buff4));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4), buff5,  malloc_usable_size(buff5));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5), buff6,  malloc_usable_size(buff6));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6), buff7,  malloc_usable_size(buff7));
	memcpy(buffOut +  malloc_usable_size(buff1) +  malloc_usable_size(buff2) +  malloc_usable_size(buff3) +  malloc_usable_size(buff4) +  malloc_usable_size(buff5) +  malloc_usable_size(buff6) +  malloc_usable_size(buff7), buff8,  malloc_usable_size(buff8));
	return true;
}
/* End Function for Data Gethering */

/* Function for Data Divide */
bool DataDivideMsgState
(
	int8_t* inputBuff,
	MsgState* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}
	memcpy(outputStruct, inputBuff, MsgStateSize);
	return true;
}

bool DataDivideJointParamSettingStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	JointParameterSettingStruct* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}

	if (msg->packetType[0] & bitPattern[0]) {
		memcpy(outputStruct, inputBuff + MsgStateSize, JointParamSetStructSize);
	}
	else {
		return false;
	}
	return true;
}

bool DataDivideJointTrajectorySetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	JointTrajectorySetStruct* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}
	int pointer = 0;
	if (msg->packetType[0] & bitPattern[0]) {
		pointer += JointParamSetStructSize;
	}

	if (msg->packetType[0] & bitPattern[1]) {
		memcpy(outputStruct, inputBuff + MsgStateSize + pointer, JointTrajSetStructSize);
	}
	else {
		return false;
	}
	return true;
}

bool DataDivideCartesianParameterSettingStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	CartesianParameterSettingStruct* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}

	int pointer = 0;
	if (msg->packetType[0] & bitPattern[0]) {
		pointer += JointParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[1]) {
		pointer += JointTrajSetStructSize;
	}

	if (msg->packetType[0] & bitPattern[2]) {
		memcpy(outputStruct, inputBuff + MsgStateSize + pointer, CartesianParamSetStructSize);
	}
	else {
		return false;
	}
	return true;
}

bool DataDivideCartesianTrajectorySetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	CartesianTrajectorySetStruct* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}

	int pointer = 0;
	if (msg->packetType[0] & bitPattern[0]) {
		pointer += JointParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[1]) {
		pointer += JointTrajSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[2]) {
		pointer += CartesianParamSetStructSize;
	}

	if (msg->packetType[0] & bitPattern[3]) {
		memcpy(outputStruct, inputBuff + MsgStateSize + pointer, CartesianTrajSetStructSize);
	}
	else {
		return false;
	}
	return true;
}

bool DataDivideJointTaregetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	JointTargetStruct* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}

	int pointer = 0;

	if (msg->packetType[0] & bitPattern[0]) {
		pointer += JointParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[1]) {
		pointer += JointTrajSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[2]) {
		pointer += CartesianParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[3]) {
		pointer += CartesianTrajSetStructSize;
	}

	if (msg->packetType[0] & bitPattern[4]) {
		memcpy(outputStruct, inputBuff + MsgStateSize + pointer, JointTargetStructSize);
	}
	else {
		return false;
	}
	return true;
}

bool DataDivideCartesianTargetStruct
(
	int8_t* inputBuff,
	MsgState* msg,
	CartesianTargetStruct* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}

	int pointer = 0;

	if (msg->packetType[0] & bitPattern[0]) {
		pointer += JointParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[1]) {
		pointer += JointTrajSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[2]) {
		pointer += CartesianParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[3]) {
		pointer += CartesianTrajSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[4]) {
		pointer += JointTargetStructSize;
	}

	if (msg->packetType[0] & bitPattern[5]) {
		memcpy(outputStruct, inputBuff + MsgStateSize + pointer, CartesianTargetStructSize);
	}
	else {
		return false;
	}
	return true;
}

bool DataDivideServerSystemData
(
	int8_t* inputBuff,
	MsgState* msg,
	ServerSystemData* outputStruct
)
{
	if (inputBuff == NULL) {
		return false;
	}

	int pointer = 0;
	if (msg->packetType[0] & bitPattern[0]) {
		pointer += JointParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[1]) {
		pointer += JointTrajSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[2]) {
		pointer += CartesianParamSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[3]) {
		pointer += CartesianTrajSetStructSize;
	}
	if (msg->packetType[0] & bitPattern[4]) {
		pointer += JointTargetStructSize;
	}
	if (msg->packetType[0] & bitPattern[5]) {
		pointer += CartesianTargetStructSize;
	}

	if (msg->packetType[0] & bitPattern[6]) {
		memcpy(outputStruct, inputBuff + MsgStateSize + pointer, ServerSystemDataSize);
	}
	else {
		return false;
	}
	return true;
}
/* End Function for Data Divide */

#endif //ETHERNET_PARSER_H_