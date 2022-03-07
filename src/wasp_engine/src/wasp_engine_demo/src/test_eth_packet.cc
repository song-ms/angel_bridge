#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <sstream>
#include <cassert>

#include "eth_packet/eth_packet.hpp"

using namespace std;
using namespace eth;

int main()
{
    cout << "1. Init" << endl;
    Packet tx;
    Packet rx;

    cout << "2. Set Tx MsgTypes" << endl;
    tx.SetMsgType(JointParamSet);
    tx.SetMsgType(JointTargetSet);
    cout << endl;

    cout << "3. Set Tx Data" << endl;
    tx.SetData().j_param->jointPositionPgain[0] = 0.1;
    tx.SetData().j_param->jointPositionPgain[1] = 1.2;
    tx.SetData().j_param->jointPositionPgain[2] = 2.3;
    tx.SetData().j_param->jointPositionPgain[3] = 3.4;
    tx.SetData().j_param->jointConstantSpring[0] = 0.1;
    tx.SetData().j_param->jointConstantSpring[1] = 1.2;
    tx.SetData().j_param->jointConstantSpring[2] = 2.3;
    tx.SetData().j_param->jointConstantSpring[3] = 3.4;
    tx.SetData().j_target->jointTarget[0] = 1.23;
    tx.SetData().j_target->jointTarget[1] = 2.34;
    tx.SetData().j_target->jointTarget[2] = 3.45;
    tx.SetData().j_target->jointTarget[3] = 4.56;
    cout << endl;

    cout << "5. Pack Tx Data" << endl;
    assert(tx.PackData() == 0);
    cout << "Tx payload: " << tx.Size() << endl;
    assert(tx.Size() == 215);
    cout << endl;

    cout << "Tx : Rx Data Before Transport " << endl;
    cout << tx.GetData().j_param->jointPositionPgain[0]  << " : " << rx.GetData().j_param->jointPositionPgain[0]  << endl;
    cout << tx.GetData().j_param->jointPositionPgain[1]  << " : " << rx.GetData().j_param->jointPositionPgain[1]  << endl;
    cout << tx.GetData().j_param->jointPositionPgain[2]  << " : " << rx.GetData().j_param->jointPositionPgain[2]  << endl;
    cout << tx.GetData().j_param->jointPositionPgain[3]  << " : " << rx.GetData().j_param->jointPositionPgain[3]  << endl;
    cout << tx.GetData().j_param->jointConstantSpring[0] << " : " << rx.GetData().j_param->jointConstantSpring[0] << endl;
    cout << tx.GetData().j_param->jointConstantSpring[1] << " : " << rx.GetData().j_param->jointConstantSpring[1] << endl;
    cout << tx.GetData().j_param->jointConstantSpring[2] << " : " << rx.GetData().j_param->jointConstantSpring[2] << endl;
    cout << tx.GetData().j_param->jointConstantSpring[3] << " : " << rx.GetData().j_param->jointConstantSpring[3] << endl;
    cout << tx.GetData().j_target->jointTarget[0]        << " : " << rx.GetData().j_target->jointTarget[0]        << endl;
    cout << tx.GetData().j_target->jointTarget[1]        << " : " << rx.GetData().j_target->jointTarget[1]        << endl;
    cout << tx.GetData().j_target->jointTarget[2]        << " : " << rx.GetData().j_target->jointTarget[2]        << endl;
    cout << tx.GetData().j_target->jointTarget[3]        << " : " << rx.GetData().j_target->jointTarget[3]        << endl;
    assert(tx.GetData().j_param->jointPositionPgain[0]  != rx.GetData().j_param->jointPositionPgain[0]  );
    assert(tx.GetData().j_param->jointPositionPgain[1]  != rx.GetData().j_param->jointPositionPgain[1]  );
    assert(tx.GetData().j_param->jointPositionPgain[2]  != rx.GetData().j_param->jointPositionPgain[2]  );
    assert(tx.GetData().j_param->jointPositionPgain[3]  != rx.GetData().j_param->jointPositionPgain[3]  );
    assert(tx.GetData().j_param->jointConstantSpring[0] != rx.GetData().j_param->jointConstantSpring[0] );
    assert(tx.GetData().j_param->jointConstantSpring[1] != rx.GetData().j_param->jointConstantSpring[1] );
    assert(tx.GetData().j_param->jointConstantSpring[2] != rx.GetData().j_param->jointConstantSpring[2] );
    assert(tx.GetData().j_param->jointConstantSpring[3] != rx.GetData().j_param->jointConstantSpring[3] );
    assert(tx.GetData().j_target->jointTarget[0]        != rx.GetData().j_target->jointTarget[0]        );
    assert(tx.GetData().j_target->jointTarget[1]        != rx.GetData().j_target->jointTarget[1]        );
    assert(tx.GetData().j_target->jointTarget[2]        != rx.GetData().j_target->jointTarget[2]        );
    assert(tx.GetData().j_target->jointTarget[3]        != rx.GetData().j_target->jointTarget[3]        );
    cout << endl;

    cout << "6. Transport" << endl;
    copy(tx.ToChar(), tx.ToChar()+tx.Size(), rx.ToChar());
    cout << endl;

    cout << "7. Unack Data" << endl;
    assert(rx.UnpackData() == 0);
    cout << "Rx payload: " << rx.Size() << endl;
    assert(rx.Size() == 215);
    cout << endl;

    cout << "Tx : Rx Data After Transport " << tx.Size() << endl;
    cout << tx.GetData().j_param->jointPositionPgain[0]  << " : " << rx.GetData().j_param->jointPositionPgain[0]  << endl;
    cout << tx.GetData().j_param->jointPositionPgain[1]  << " : " << rx.GetData().j_param->jointPositionPgain[1]  << endl;
    cout << tx.GetData().j_param->jointPositionPgain[2]  << " : " << rx.GetData().j_param->jointPositionPgain[2]  << endl;
    cout << tx.GetData().j_param->jointPositionPgain[3]  << " : " << rx.GetData().j_param->jointPositionPgain[3]  << endl;
    cout << tx.GetData().j_param->jointConstantSpring[0] << " : " << rx.GetData().j_param->jointConstantSpring[0] << endl;
    cout << tx.GetData().j_param->jointConstantSpring[1] << " : " << rx.GetData().j_param->jointConstantSpring[1] << endl;
    cout << tx.GetData().j_param->jointConstantSpring[2] << " : " << rx.GetData().j_param->jointConstantSpring[2] << endl;
    cout << tx.GetData().j_param->jointConstantSpring[3] << " : " << rx.GetData().j_param->jointConstantSpring[3] << endl;
    cout << tx.GetData().j_target->jointTarget[0]        << " : " << rx.GetData().j_target->jointTarget[0]        << endl;
    cout << tx.GetData().j_target->jointTarget[1]        << " : " << rx.GetData().j_target->jointTarget[1]        << endl;
    cout << tx.GetData().j_target->jointTarget[2]        << " : " << rx.GetData().j_target->jointTarget[2]        << endl;
    cout << tx.GetData().j_target->jointTarget[3]        << " : " << rx.GetData().j_target->jointTarget[3]        << endl;
    assert(tx.GetData().j_param->jointPositionPgain[0]  == rx.GetData().j_param->jointPositionPgain[0]  );
    assert(tx.GetData().j_param->jointPositionPgain[1]  == rx.GetData().j_param->jointPositionPgain[1]  );
    assert(tx.GetData().j_param->jointPositionPgain[2]  == rx.GetData().j_param->jointPositionPgain[2]  );
    assert(tx.GetData().j_param->jointPositionPgain[3]  == rx.GetData().j_param->jointPositionPgain[3]  );
    assert(tx.GetData().j_param->jointConstantSpring[0] == rx.GetData().j_param->jointConstantSpring[0] );
    assert(tx.GetData().j_param->jointConstantSpring[1] == rx.GetData().j_param->jointConstantSpring[1] );
    assert(tx.GetData().j_param->jointConstantSpring[2] == rx.GetData().j_param->jointConstantSpring[2] );
    assert(tx.GetData().j_param->jointConstantSpring[3] == rx.GetData().j_param->jointConstantSpring[3] );
    assert(tx.GetData().j_target->jointTarget[0]        == rx.GetData().j_target->jointTarget[0]        );
    assert(tx.GetData().j_target->jointTarget[1]        == rx.GetData().j_target->jointTarget[1]        );
    assert(tx.GetData().j_target->jointTarget[2]        == rx.GetData().j_target->jointTarget[2]        );
    assert(tx.GetData().j_target->jointTarget[3]        == rx.GetData().j_target->jointTarget[3]        );
    cout << endl;

    // for(size_t i = 0; i < tx.Size(); i++){
    //     cout << +tx.ToChar()[i];
    //     if((i+1)%8 == 0){
    //         cout << endl;
    //     }else if((i+1)%4 ==0){
    //         cout << "\t|\t";
    //     }else{
    //         cout << "\t";
    //     }
    // }

    return 0;
}