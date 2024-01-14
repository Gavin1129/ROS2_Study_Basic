/*
  Task: write a listener node
        1. subscribe the relationship of different frames (laser ==> base_link, camera ==> base_link)
        2. Gnerate the coordinate transfer from laser to camera

  Steps:
      3. Define a node class
        3.1 create tf buffer pointer
        3.2 create tf listener, bind with buffer object. put all data into buffer
        3.3 create a timer, find frames and generate the transformation 
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TFListener:public rclcpp::Node{
  public:
    TFListener():Node("tf_listener_node_cpp"){
      // 3.1
      // TF2_ROS_PUBLIC Buffer(
      //  rclcpp::Clock::SharedPtr clock,
      //  tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME),
      //  rclcpp::Node::SharedPtr node = rclcpp::Node::SharedPtr());
      buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      // 3.2 write into buffer
      /*
        TransformListener(
          tf2::BufferCore & buffer,
          NodeT && node,
          bool spin_thread = true,
          const rclcpp::QoS & qos = DynamicListenerQoS(),
          const rclcpp::QoS & static_qos = StaticListenerQoS(),
          const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
          detail::get_default_transform_listener_sub_options<AllocatorT>(),
          const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options =
          detail::get_default_transform_listener_static_sub_options<AllocatorT>())
          : buffer_(buffer)
          {
            init(node, spin_thread, qos, static_qos, options, static_options);
          }
      */
      listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_,this);

      //3.3 
      timer_ = this->create_wall_timer(1s,std::bind(&TFListener::on_timer,this));
    }

  private:
    void on_timer(){
      // frame transfermation 
      
      try{
        /*
        geometry_msgs::msg::TransformStamped 
          lookupTransform(const std::string &target_frame, 
            const std::string &source_frame, 
            const tf2::TimePoint &time) const
      */
        auto ts = buffer_->lookupTransform("camera","laser",tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(),"-------Tranfered Result------");
        RCLCPP_INFO(this->get_logger(),
            "Parent frame: %s, Child frame: %s, Offset: (%.2f, %.2f, %.2f)",
            ts.header.frame_id.c_str(), // camera
            ts.child_frame_id.c_str(), //laser
            ts.transform.translation.x,
            ts.transform.translation.y,
            ts.transform.translation.z
          );
      }
      catch(const tf2::LookupException& e){
        RCLCPP_INFO(this->get_logger(),"Error: %s",e.what());
      }
      
    }
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<TFListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
