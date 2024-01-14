/*
    Task: Listener listen to the broadcasted data and get the position of turtle2 relevant to the turtle1
          generate command to control the motion of turtle2
*/


#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Exer03TFListener:public rclcpp::Node{
    public:
        Exer03TFListener():Node("exer01_spawn_cpp"){
            this->declare_parameter("father_frame","turtle2");
            this->declare_parameter("child_frame","turtle1");
            father_frame = this->get_parameter("father_frame").as_string();
            child_frame = this->get_parameter("child_frame").as_string();

            // 3.2 buffer
            buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

            //3.3 listener
            listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

            //3.4 speed publisher
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/"+father_frame+"/cmd_vel",10);

            //3.5 timer
            timer_ = this->create_wall_timer(1s,std::bind(&Exer03TFListener::on_timer,this));

        }
    private:
        void on_timer(){
            try
            {
                //3.5.1 Posiiton transfer
                auto ts = buffer_->lookupTransform(father_frame,child_frame,tf2::TimePointZero);

                //3.5.2 Publish speed command
                geometry_msgs::msg::Twist twist;
                twist.linear.x = 0.5*sqrt(
                    pow(ts.transform.translation.x,2) + 
                    pow(ts.transform.translation.y,2)); 
                twist.angular.z = 1.0*atan2(ts.transform.translation.y,ts.transform.translation.x);
                cmd_pub_->publish(twist);
            }
            catch(const tf2::LookupException& e)
            {
                RCLCPP_INFO(this->get_logger(),"Error: %s",e.what());
            }
            
            
        }
        std::string father_frame;
        std::string child_frame;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Exer03TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}