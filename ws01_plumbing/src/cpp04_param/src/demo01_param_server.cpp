/*
    Parameter server node
    Task: 1. Declare parameters
          2. Find parameters
          3. Edit paramters
          4. Delete parameters 
*/

#include "rclcpp/rclcpp.hpp"

class ParamServer: public rclcpp::Node{
    public:
        ParamServer():Node("param_server_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true)){
            RCLCPP_INFO(this->get_logger(),"Param server created!");
        }
        // add
        void declare_param(){
            RCLCPP_INFO(this->get_logger(),"------------------Add---------------");
            this->declare_parameter("car_name","BMW");
            this->declare_parameter("width",1.55);
            this->declare_parameter("wheels",5);

            this->set_parameter(rclcpp::Parameter("height",2.00));
        }
        // find
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"------------------Find---------------");
            // this->get_parameter();
            // this->get_parameters();
            // this->has_parameter();

            auto car = this->get_parameter("car_name");
            RCLCPP_INFO(this->get_logger(),"Key = %s, value = %s",car.get_name().c_str(),car.as_string().c_str());

            auto params = this->get_parameters({"car_name","width","wheels"});
            for(auto &&param:params){
                RCLCPP_INFO(this->get_logger(),"(%s = %s)",param.get_name().c_str(),param.value_to_string().c_str());
            }

            RCLCPP_INFO(this->get_logger(),"Has car_name? %d", this->has_parameter("car_name"));
        }
        // edit
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"------------------Edit---------------");
            this->set_parameter(rclcpp::Parameter("width",1.75));
            RCLCPP_INFO(this->get_logger(),"width = %.2f",this->get_parameter("width").as_double());

        }
        // delete
        void delete_param(){
            RCLCPP_INFO(this->get_logger(),"------------------Delete---------------");
            // this->undeclare_parameter("car_name"); // cannot delete declared parameters
            RCLCPP_INFO(this->get_logger(),"Has height? %d", this->has_parameter("height"));
            this->undeclare_parameter("height");
            RCLCPP_INFO(this->get_logger(),"Has height? %d", this->has_parameter("height"));
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->delete_param();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}