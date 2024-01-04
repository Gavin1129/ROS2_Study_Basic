/*
    Task: 1. Create a client node
          2. Connect to server node
          3. Find paramters
          4. Change parameters
*/

#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

class ParamClient: public rclcpp::Node{
    public:
        ParamClient():Node("param_client_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true)){
            RCLCPP_INFO(this->get_logger(),"Param client created!");
            // explicit SyncParametersClient(
            //     std::shared_ptr<NodeT> node,
            //     const std::string & remote_node_name = "",
            //     const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters)
            param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");
            /*
                Q: Client connect to server by node name?
            */
        }
        
        // connect to server
        bool connect_server(){
            while(!param_client_->wait_for_service(1s)){
                
                if(!rclcpp::ok()){
                    return false;
                }
                RCLCPP_INFO(this->get_logger(),"Connecting ...");
            }
            return true;
        }
        // find parameters
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"----------Find--------");
            std::string car_name = param_client_->get_parameter<std::string>("car_name");
            double width = param_client_->get_parameter<double>("width");
            RCLCPP_INFO(this->get_logger(),"car_name = %s",car_name.c_str());
            RCLCPP_INFO(this->get_logger(),"Width = %f",width);
            auto params = param_client_->get_parameters({"car_name","width","wheels"});
            for(auto &&param:params){
                RCLCPP_INFO(this->get_logger(),"(%s = %s)",param.get_name().c_str(),param.value_to_string().c_str());
            }
            RCLCPP_INFO(this->get_logger(),"Has car_name? %d", this->has_parameter("car_name"));
        }

        //Edit parameters
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"----------Edit--------");
            param_client_->set_parameters({rclcpp::Parameter("car_name","Audi"),
                rclcpp::Parameter("width",3.0)});
        }
    private:
        rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ParamClient>();
    bool flag = node->connect_server();
    if(!flag){
        return 0;
    }
    node->get_param();
    node->update_param();
    node->get_param();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}