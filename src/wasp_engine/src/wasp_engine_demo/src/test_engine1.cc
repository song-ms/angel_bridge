#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "std_srvs/srv/set_bool.hpp"



const std::string& packet_flag_param_name = "packet_flag";

const std::string& eth_connect_srv_name = "eht_connect_srv";
const std::string& eth_streaming_srv_name = "eht_streaming_srv";

typedef std_srvs::srv::SetBool SetBoolSrv;

using namespace std;
using namespace rclcpp;

void SrvTest(shared_ptr<Node> node, shared_ptr<Client<SetBoolSrv>> cli, bool flag)
{
    while (!cli->wait_for_service(1s)) {}
    auto req = make_shared<SetBoolSrv::Request>();
    req->data = flag;
    auto res = cli->async_send_request(req);
    bool success = spin_until_future_complete(node, res, 100ms) == FutureReturnCode::SUCCESS;
    RCLCPP_INFO(node->get_logger(), (success? "good" : "oh no"));
    sleep_for(100ms);
}

int main(int argc, char** argv)
{
    init(argc, argv);
    auto node = make_shared<Node>("test_engine1");
    
    auto conn_cli = node->create_client<SetBoolSrv>(eth_connect_srv_name);
    auto strm_cli = node->create_client<SetBoolSrv>(eth_streaming_srv_name);

    auto req = make_shared<SetBoolSrv::Request>();
    
    /* Connection Srv Test */
    SrvTest(node, conn_cli, true);
    SrvTest(node, conn_cli, false);

    /* Streaming Srv Test */
    SrvTest(node, strm_cli, true);
    SrvTest(node, strm_cli, false);

    shutdown();
}