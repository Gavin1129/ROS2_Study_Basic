/*
    Task: Broadcast the turtle's position in world frame
*/


#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

class Exer02TFBroadcaster:public rclcpp::Node{
    public:
        Exer02TFBroadcaster():Node("exer01_spawn_cpp"){
            this->declare_parameter("turtle","turtle1");
            turtle = this->get_parameter("turtle").as_string();
            broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/"+turtle+"/pose",10,
                std::bind(&Exer02TFBroadcaster::do_pose,this,std::placeholders::_1));
        }
    private:
        void do_pose(const turtlesim::msg::Pose &pose){
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = turtle;

        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = 0.0;

        tf2::Quaternion qtn;
        qtn.setRPY(0,0,pose.theta);
        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        broadcaster_->sendTransform(ts);

        }
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
        std::string turtle;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Exer02TFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}