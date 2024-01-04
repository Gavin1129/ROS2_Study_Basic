/*
  Server node implement
  Task: receive 2 int numbers and add them together and then return to client
*/


// 1. include header files
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using base_interfaces_demo::srv::AddInts;

// 3. Define node
class AddIntsServer: public rclcpp::Node{
  public:
    AddIntsServer():Node("add_ints_server_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"Add ints server node is ready...");
      // rclcpp::Service<ServiceT>::SharedPtr <typename ServiceT, typename CallbackT> 
      // create_service(const std::string &service_name, 
      //                CallbackT &&callback, 
      //                const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default, 
      //                rclcpp::CallbackGroup::SharedPtr group = nullptr)
      server_ = this->create_service<AddInts>("add_ints",std::bind(&AddIntsServer::add, this, _1, _2));
    }
  private:
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res){
      res->sum = req->num1 + req->num2;
      RCLCPP_INFO(this->get_logger(), "Request number:(%d, %d),Result:(%d)",req->num1,req->num2,res->sum);
    }
    rclcpp::Service<AddInts>::SharedPtr server_;
};

int main(int argc, char* argv[]){
  // 2. initilization
  rclcpp::init(argc,argv);

  // 3. Create a node
  auto node = std::make_shared<AddIntsServer>();
  
  // 4. spin()
  rclcpp::spin(node);

  // 5. shutdown
  rclcpp::shutdown();

  return 0;
}
