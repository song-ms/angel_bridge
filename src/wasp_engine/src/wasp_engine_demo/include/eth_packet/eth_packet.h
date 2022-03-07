#ifndef ETH_PACKET_H_
#define ETH_PACKET_H_

#include "ethernet_parser/ethernet_structure.h"

typedef enum ETH_MsgType{ // value must be msg type bit number
    JointParamSet      = 0,
    JointTrajSet       = 1,
    CartesianParamSet  = 2,
    CartesianTrajSet   = 3,
    JointTargetSet     = 4,
    CartesianTargetSet = 5,
    ServerSysData      = 6,
    MsgStateData = 0xFF, // default msg
}ETH_MsgType;


typedef struct ETH_PtrSizePair{
    char* ptr;
    int size;
}ETH_PtrSizePair;


typedef struct ETH_Packet
{
struct MsgState* msg_state;
struct ServerSystemData* sys_data;
struct JointParameterSettingStruct* j_param;
struct JointTrajectorySetStruct* j_traj;
struct CartesianParameterSettingStruct* c_param;
struct CartesianTrajectorySetStruct* c_traj;
struct JointTargetStruct* j_target;
struct CartesianTargetStruct* c_target;
int msg_state_size;
int sys_data_size;
int j_param_size;
int j_traj_size;
int c_param_size;
int c_traj_size;
int j_target_size;
int c_target_size;

char* buff;

int size;
int max_size;

uint16_t msg_types_flag;

}ETH_Packet;

/*
  ___      _    _ _    _ 
 | _ \_  _| |__| (_)__(_)
 |  _/ || | '_ \ | / _|_ 
 |_|  \_,_|_.__/_|_\__(_)

*/
int ETH_InitPacket(ETH_Packet* pkt);
int ETH_SetMsgType(ETH_Packet* pkt, ETH_MsgType msg_type);
int ETH_PackData(ETH_Packet* pkt);
int ETH_UnpackData(ETH_Packet* pkt);
int ETH_DeinitPacket(ETH_Packet* pkt);

/*
  ___     _          _       _ 
 | _ \_ _(_)_ ____ _| |_ ___(_)
 |  _/ '_| \ V / _` |  _/ -_)_ 
 |_| |_| |_|\_/\__,_|\__\___(_)
                               
*/

int ETH_DataToBuff(char* data_ptr, int data_size, int at);
int ETH_BuffToData(char* data_ptr, int data_size, int at);

// std::vector<MsgType> MsgTypeFlagToVec(const MsgTypeFlag& msg_flag, bool attach_default) const;
// int GetPayloadSize(const std::vector<MsgType>& msgs) const;

ETH_PtrSizePair ETH_MsgTypeToDataPair(ETH_MsgType msg);
int ETH_InitData(ETH_Packet* pkt);



#endif //ETH_PACKET_H_