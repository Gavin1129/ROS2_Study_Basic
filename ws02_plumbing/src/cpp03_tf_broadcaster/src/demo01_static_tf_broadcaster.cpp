/*
  Task: Write a static broadcaster
        parameters input is needed:
        ros2 run [package_name] [node_name] x y z roll pitch yaw frame child_frame
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TFStaticBroadcaster:public rclcpp::Node{
  public:
    TFStaticBroadcaster(char* argv[]):Node("tf_static_broadcaster_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"TF Static Broadcaster created!");
      /*
        publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
                        node, "/tf_static", qos, options);
      */
      broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // publish message
      pub_static_tf(argv);
    }
  private:
    void pub_static_tf(char* argv[]){
      // messages
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now(); // time stamp
      transform.header.frame_id = argv[7]; // frame
      transform.child_frame_id = argv[8]; // child_frame

      // translation
      transform.transform.translation.x = atof(argv[1]);
      transform.transform.translation.y = atof(argv[2]);
      transform.transform.translation.z = atof(argv[3]);

      // rotation 
      tf2::Quaternion qtn;
      qtn.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
      transform.transform.rotation.x = qtn.x();
      transform.transform.rotation.y = qtn.y();
      transform.transform.rotation.z = qtn.z();
      transform.transform.rotation.w = qtn.w();

      // publish
      broadcaster_->sendTransform(transform);
    }
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char* argv[]){
  if(argc != 9){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Illegal input!");
    return 1;
  }
  rclcpp::init(argc,argv);
  auto node = std::make_shared<TFStaticBroadcaster>(argv);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
