#ifndef ETH_PACKET_H_
#define ETH_PACKET_H_

#include <iostream>
#include <vector>
#include <bitset>

#include "ethernet_parser/ethernet_structure.h"

namespace eth
{

typedef enum { // value must be msg type bit number
    JointParamSet      = 0,
    JointTrajSet       = 1,
    CartesianParamSet  = 2,
    CartesianTrajSet   = 3,
    JointTargetSet     = 4,
    CartesianTargetSet = 5,
    ServerSysData      = 6,
    MsgStateData = 0xFF, // default msg
}MsgType;


struct Data{
    MsgState* msg_state;
    ServerSystemData* sys_data;
    JointParameterSettingStruct* j_param;
    JointTrajectorySetStruct* j_traj;
    CartesianParameterSettingStruct* c_param;
    CartesianTrajectorySetStruct* c_traj;
    JointTargetStruct* j_target;
    CartesianTargetStruct* c_target;
};

struct DataSize{
    size_t msg_state;
    size_t sys_data;
    size_t j_param;
    size_t j_traj;
    size_t c_param;
    size_t c_traj;
    size_t j_target;
    size_t c_target;
};

struct PtrSizePair{
    char* ptr;
    size_t size;
};

typedef std::bitset<16> MsgTypeFlag;

class Packet
{
public:
    Packet();

    int SetMsgType(MsgType msg_type);
    int PackData();
    int UnpackData();
    
    Data& SetData();
    const Data& GetData() const;
    
    uint16_t GetCommState() const;
    void SetCommState(uint16_t commState);

    uint16_t GetControlMode() const;
    void SetControlMode(uint16_t controlMode);

    bool IsMsgRecieved(MsgType msg_type);

    char* ToChar();
    size_t Size() const;
    size_t MaxSize() const;

private:
    size_t size_;
    size_t max_size_;

    MsgTypeFlag msg_types_flag_;
    std::vector<char> buff_;

    int DataToBuff(char* data_ptr, size_t data_size, size_t at);
    int BuffToData(char* data_ptr, size_t data_size, size_t at);

    Data data_;
    DataSize data_size_;

    std::vector<MsgType> MsgTypeFlagToVec(const MsgTypeFlag& msg_flag, bool attach_default) const;
    size_t GetPayloadSize(const std::vector<MsgType>& msgs) const;

    PtrSizePair MsgTypeToDataPair(MsgType msg) const;

    int InitData();
};


} //namespace eth
#endif //ETH_PACKET_H_