/*
    Task: Point tf listener node
          1. broadcast laser=>base_link , publish point=>laser
          2. find point=>base_link
    Steps:
        3.1 define a node class
        3.2 create a tf listener
        3.3 create a subscriber and subscribe a topic
        3.4 create a filter 
        3.5 write a callback function for filter to achieve the transformation
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/message_filter.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TFPointListener:public rclcpp::Node{
    public:
        TFPointListener():Node("tf_point_listener"){
            // 3.2 create a tf listener
            /*
                TF2_ROS_PUBLIC Buffer(
                    rclcpp::Clock::SharedPtr clock,
                    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME),
                    rclcpp::Node::SharedPtr node = rclcpp::Node::SharedPtr());
            */
            buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            /*
                CreateTimerROS(
                    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
                    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
                    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
            */
            timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
                                    this->get_node_base_interface(),
                                    this->get_node_timers_interface()
                                );
            /*
                inline void
                setCreateTimerInterface(CreateTimerInterface::SharedPtr create_timer_interface)
                {
                    timer_interface_ = create_timer_interface;
                }
            */
            buffer_->setCreateTimerInterface(timer_);
            /*
                TransformListener(tf2::BufferCore &buffer, bool spin_thread = true)
            */
            listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

            // 3.3 create a subscriber and subscribe a topic
            /*
                void subscribe(rclcpp::Node *node, const std::string &topic, const rmw_qos_profile_t qos)
            */
            point_sub.subscribe(this,"point");

            // 3.4 create a filter 
            /*
                MessageFilter(
                    F & f, BufferT & buffer, const std::string & target_frame, uint32_t queue_size,
                    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
                    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
                    std::chrono::duration<TimeRepT, TimeT> buffer_timeout =
                    std::chrono::duration<TimeRepT, TimeT>::max())
            */
           /*
                * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
                * \param buffer The buffer this filter should use
                * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
                * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
                * \param node_logging The logging interface to use for any log messages
                * \param node_clock The clock interface to use to get the node clock
                * \param buffer_timeout The timeout duration after requesting transforms from the buffer.
           */
            filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
                point_sub,
                *buffer_,
                "base_link",
                10,
                this->get_node_logging_interface(),
                this->get_node_clock_interface(),
                1s
            );

            // 3.5 write a callback function for filter to achieve the transformation
            filter_->registerCallback(&TFPointListener::transform_point,this);
        }
    private:
        void transform_point(const geometry_msgs::msg::PointStamped & ps){
            /*
                T & transform(
                    const T & in, T & out,
                    const std::string & target_frame, 
                    tf2::Duration timeout = tf2::durationFromSec(0.0))
            */
           /*
                T transform( const T & in,
                    const std::string & target_frame, 
                    tf2::Duration timeout = tf2::durationFromSec(0.0)) const
                    {
                        T out;
                        return this->transform(in, out, target_frame, timeout);
                    }
           */
            // include header: tf2_geometry_msgs
            auto out = buffer_->transform(ps,"base_link");
            RCLCPP_INFO(this->get_logger(),"Parent frame: %s, Location:(%.2f,%.2f,%.2f)",
                                    out.header.frame_id.c_str(),
                                    out.point.x,
                                    out.point.y,
                                    out.point.z);

        }
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
        std::shared_ptr<tf2_ros::CreateTimerROS> timer_;

        message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub;
        
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TFPointListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}