#include "eth_packet/eth_packet.hpp"


using namespace eth;


Packet::Packet() : size_(0), max_size_(0)
{
    buff_.clear();
    
    InitData();

    max_size_ = 
        data_size_.msg_state +
        data_size_.sys_data +
        data_size_.j_param +
        data_size_.j_traj +
        data_size_.c_param +
        data_size_.c_traj +
        data_size_.j_target +
        data_size_.c_target;

    buff_.resize(max_size_);

    /* Set all buffer and  data to 0*/
    std::fill(buff_.begin(), buff_.end(), 0);

    std::vector<MsgType> msgs = {
        MsgStateData, 
        JointParamSet, 
        JointTrajSet,
        CartesianParamSet,
        CartesianTrajSet,
        JointTargetSet,
        CartesianTargetSet,
        ServerSysData
        };

    size_t pos = 0;
    for(auto msg : msgs){
        PtrSizePair data;
        data = MsgTypeToDataPair(msg);
        BuffToData(data.ptr, data.size, pos);
        pos += data.size;
    }
}


int Packet::SetMsgType(MsgType msg_type)
{
    msg_types_flag_.set(static_cast<size_t>(msg_type));
    return 0;
}


int Packet::PackData()
{
    auto msgs = MsgTypeFlagToVec(msg_types_flag_, true);
    size_ = GetPayloadSize(msgs);

    data_.msg_state->packetType[0] =  (msg_types_flag_.to_ulong()       & 0xFFFF);
    data_.msg_state->packetType[1] = ((msg_types_flag_.to_ulong() >> 8) & 0xFFFF);
    data_.msg_state->payloadSize = size_;

    size_t pos = 0;
    for(auto msg : msgs){
        PtrSizePair data;
        data = MsgTypeToDataPair(msg);
        if(DataToBuff(data.ptr, data.size, pos) < 0){
            return -1;
        }
        pos += data.size;
    }
    return 0;
}


int Packet::UnpackData()
{
    size_t pos = 0;
    if(BuffToData(reinterpret_cast<char*>(data_.msg_state), data_size_.msg_state, pos) < 0){
        return -1;
    }
    pos += data_size_.msg_state;

    int flag = data_.msg_state->packetType[0] 
             +(data_.msg_state->packetType[1] << 8);
    msg_types_flag_ = MsgTypeFlag(flag);

    size_ = data_.msg_state->payloadSize;

    auto msgs = MsgTypeFlagToVec(msg_types_flag_, false);
    for(auto msg : msgs){
        PtrSizePair data;
        data = MsgTypeToDataPair(msg);
        if(BuffToData(data.ptr, data.size, pos) < 0){
            return -1;
        }
        pos += data.size;
    }
    return 0;
}


int Packet::DataToBuff(char* data_ptr, size_t data_size, size_t at)
{
    if(data_ptr == NULL){
        return -1;
    }

    std::copy(data_ptr, data_ptr+data_size, buff_.data()+at);
    return 0;
}


int Packet::BuffToData(char* data_ptr, size_t data_size, size_t at)
{
    if(data_ptr == NULL){
        return -1;
    } 
    std::copy(buff_.data()+at, buff_.data()+at+data_size, data_ptr);
    return 0;

}


std::vector<MsgType> Packet::MsgTypeFlagToVec(const MsgTypeFlag& msg_flag, bool attach_default) const
{
    std::vector<MsgType> msgs;
    if(attach_default){
        msgs.push_back(MsgStateData); //Default msg
    }

    size_t i = 0, n_msgs = 0;
    size_t max_msgs = msg_flag.count();

    while(n_msgs < max_msgs && i < 16){
        if(msg_flag.test(i)){
            msgs.push_back(static_cast<MsgType>(i));
            ++n_msgs;
        }
        ++i;
    }

    return msgs;
}


size_t Packet::GetPayloadSize(const std::vector<MsgType>& msgs) const
{
    size_t payload = 0;
    for(auto msg : msgs){
        payload += MsgTypeToDataPair(msg).size;
    }
    return payload;
}


PtrSizePair Packet::MsgTypeToDataPair(MsgType msg) const
{
    PtrSizePair res;
    res.ptr = NULL;
    res.size = 0;

    switch(msg){
        case(MsgStateData):       res.ptr = reinterpret_cast<char*>(data_.msg_state); res.size = data_size_.msg_state; break;
        case(JointParamSet):      res.ptr = reinterpret_cast<char*>(data_.j_param);   res.size = data_size_.j_param;   break;
        case(JointTrajSet):       res.ptr = reinterpret_cast<char*>(data_.j_traj);    res.size = data_size_.j_traj;    break;
        case(CartesianParamSet):  res.ptr = reinterpret_cast<char*>(data_.c_param);   res.size = data_size_.c_param;   break;
        case(CartesianTrajSet):   res.ptr = reinterpret_cast<char*>(data_.c_traj);    res.size = data_size_.c_traj;    break;
        case(JointTargetSet):     res.ptr = reinterpret_cast<char*>(data_.j_target);  res.size = data_size_.j_target;  break;
        case(CartesianTargetSet): res.ptr = reinterpret_cast<char*>(data_.c_target);  res.size = data_size_.c_target;  break;
        case(ServerSysData):      res.ptr = reinterpret_cast<char*>(data_.sys_data);  res.size = data_size_.sys_data;  break;
        default: break;
    }

    return res;
}


int Packet::InitData()
{
    data_.msg_state = new MsgState;
    data_.sys_data  = new ServerSystemData;
    data_.j_param   = new JointParameterSettingStruct;
    data_.j_traj    = new JointTrajectorySetStruct;
    data_.c_param   = new CartesianParameterSettingStruct;
    data_.c_traj    = new CartesianTrajectorySetStruct;
    data_.j_target  = new JointTargetStruct;
    data_.c_target  = new CartesianTargetStruct;

    data_size_.msg_state = sizeof(*data_.msg_state);
    data_size_.sys_data = sizeof(*data_.sys_data);
    data_size_.j_param   = sizeof(*data_.j_param);
    data_size_.j_traj    = sizeof(*data_.j_traj);
    data_size_.c_param   = sizeof(*data_.c_param);
    data_size_.c_traj    = sizeof(*data_.c_traj);
    data_size_.j_target  = sizeof(*data_.j_target);
    data_size_.c_target  = sizeof(*data_.c_target);

    return 0;
}


Data& Packet::SetData()
{
    return data_;
}

const Data& Packet::GetData() const
{
    return data_;
}

uint16_t Packet::GetCommState() const
{
    return data_.msg_state->commState;
}

void Packet::SetCommState(uint16_t commState)
{
    data_.msg_state->commState = commState;
}

uint16_t Packet::GetControlMode() const
{
    return data_.msg_state->controlMode;
}
void Packet::SetControlMode(uint16_t controlMode)
{
    data_.msg_state->controlMode = controlMode;
}

bool Packet::IsMsgRecieved(MsgType msg_type)
{
    return msg_types_flag_ .test(static_cast<size_t>(msg_type));
}

char* Packet::ToChar()
{
    return reinterpret_cast<char*>(buff_.data());
}


size_t Packet::Size() const
{
    return this->size_;
}

size_t Packet::MaxSize() const
{
    return this->max_size_;
}