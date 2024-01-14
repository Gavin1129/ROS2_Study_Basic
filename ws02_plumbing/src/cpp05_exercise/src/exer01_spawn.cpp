/*
    Task: Generate a new turtle in the window
    Steps: 
        3.1 parameter service to declare the information of the new turtle
        3.2 Generate a client
        3.3 Connect to server
        3.4 Publish messages
*/


#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class Exer01Spawn:public rclcpp::Node{
    public:
        Exer01Spawn():Node("exer01_spawn_cpp"){
            //3.1 
            this->declare_parameter("x",3.0);
            this->declare_parameter("y",3.0);
            this->declare_parameter("theta",0.0);
            this->declare_parameter("turtle_name","turtle2");
            x = this->get_parameter("x").as_double();
            y = this->get_parameter("y").as_double();
            theta = this->get_parameter("theta").as_double();
            turtle_name = this->get_parameter("turtle_name").as_string();

            //3.2
            spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

        }

        // 3.3 Connect to server
        bool connect_server(){
            while(!spawn_client_->wait_for_service()){
                if(!rclcpp::ok()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Cancled by user!");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(),"Connecting to server, please wait...");
            }
            return true;
        }

        // 3.4 Publish messages
        rclcpp::Client<turtlesim::srv::Spawn>::FutureAndRequestId request(){
            /*
                rclcpp::Client<turtlesim::srv::Spawn>::FutureAndRequestId 
                    async_send_request(std::shared_ptr<turtlesim::srv::Spawn_Request> request)
            */
            auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
            req->x = x;
            req->y = y;
            req->theta = theta;
            req->name = turtle_name;
            return spawn_client_->async_send_request(req);
        }
    private:
        double x,y,theta;
        std::string turtle_name;
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto client = std::make_shared<Exer01Spawn>();
    bool flag = client->connect_server();
    if(!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Cannot connect to server!");
    }
    auto response = client->request();
    if(rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(),"Response succesed!");
        std::string name = response.get()->name;
        if(name.empty()){
            RCLCPP_INFO(client->get_logger(),"[Name Duplication]:The turtle name has been used. Try another one.");
        }else{
            RCLCPP_INFO(client->get_logger(),"The new turtle has been created!");
        }
    }else{
        RCLCPP_INFO(client->get_logger(),"Response failed!");
    }
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}