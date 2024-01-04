/*
    Client node
    Task: Send two int numbers to server and receive the response 
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

// 3. write a node class
class AddIntsClient: public rclcpp::Node{
    public:
        AddIntsClient():Node("add_ints_client_node_cpp"){
            RCLCPP_INFO(this->get_logger(),"Client created!");
            // 3.1 create a client 
            // rclcpp::Client<ServiceT>::SharedPtr <typename ServiceT> 
            // create_client(const std::string &service_name, 
            //               const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default, 
            //               rclcpp::CallbackGroup::SharedPtr group = nullptr)
            client_ = this->create_client<AddInts>("add_ints");
        }
        // 3.2 waiting for connection to server
        bool connect_server(){
            while (!client_->wait_for_service(1s))
            {
                if(!rclcpp::ok()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Connecting cancled by user!");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Connecting to server ...");
            }
            return true;
        }
        // 3.3 send request
        rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2){
            auto request = std::make_shared<AddInts::Request>();
            request->num1 = num1;
            request->num2 = num2;
            return client_->async_send_request(request);
        }

    private:
        rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char* argv[]){
    // check inputs, must be 2 numbers
    if(argc != 3){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Please provide two int numbers");
        return 1;
    }
    rclcpp::init(argc, argv);
    // 3. 
    auto node = std::make_shared<AddIntsClient>();
    auto flag = node->connect_server();
    if(!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Connect to server failed, exit");
        return 1;
    }

    auto future = node->send_request(atoi(argv[1]),atoi(argv[2]));
    if(rclcpp::spin_until_future_complete(node,future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(node->get_logger(),"Connected to server! The sum is : %d", future.get()->sum);
    }else{
        RCLCPP_INFO(node->get_logger(),"Response failed!");
    }

    rclcpp::shutdown();
    return 0;
}