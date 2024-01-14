/*
    Task: 1. start turtlesim node
          2. Publish turtle position 
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "turtlesim/msg/pose.hpp"

class TFDynamicBroadcaster:public rclcpp::Node{
  public:
    TFDynamicBroadcaster():Node("tf_dynamic_broadcaster_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"TF Dynamic Broadcaster created!");
      broad_caster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TFDynamicBroadcaster::do_pose,this,std::placeholders::_1));
    }
  private:
    void do_pose(const turtlesim::msg::Pose &pose){
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "turtle1";

        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = 0.0;

        tf2::Quaternion qtn;
        qtn.setRPY(0,0,pose.theta);
        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        broad_caster_->sendTransform(ts);

    }
    std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<TFDynamicBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
