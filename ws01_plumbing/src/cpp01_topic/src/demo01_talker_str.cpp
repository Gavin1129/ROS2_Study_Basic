/* Publisher 
  Task: 1. Publish a message "Hello world!" with a certain frequency, add a number behind the message
        2. The number should be incremented by 1 each time of the message publication
  Steps:
        1. include files
        2. initilization 
        3. Define a publisher node
          3.1 create a publisher
          3.2 create a timer
          3.3 publish message
        4. rclcpp::spin()
        5. rclcpp::shutdowm()
*/

// 1.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 3.
class Talker:public rclcpp::Node{
  public:
    Talker():Node("talker_node_cpp"),count(0){
      RCLCPP_INFO(this->get_logger(),"Publisher created!");
      // 3.1
      // std::shared_ptr<PublisherT>   //return value 
      // create_publisher<MessageT, AllocatorT, PublisherT>(
      //                                  const std::string &topic_name, 
      //                                  const rclcpp::QoS &qos, 
      //                                  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options = rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);

      // 3.2
      // rclcpp::WallTimer<CallbackT, nullptr>::SharedPtr 
      // create_wall_timer<DurationRepT, DurationT, CallbackT>(
      //          std::chrono::duration<DurationRepT, DurationT> period, 
      //          CallbackT callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)

      // rclcpp::timer::WallTimer<CallbackT> is a specific implementation of a timer that uses wall-clock time.
      // It's a subclass of rclcpp::TimerBase
      timer_ = this->create_wall_timer(1s,std::bind(&Talker::on_timer,this));
    }
  private:
    // 3.3
    void on_timer(){
      auto message = std_msgs::msg::String();
      message.data = "Hello world!" + std::to_string(count++);
      RCLCPP_INFO(this->get_logger(),"Published messages: %s",message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count;


};

int main(int argc, char* argv[]){
   // 2. 
   rclcpp::init(argc,argv);
   // 3.
   auto node = std::make_shared<Talker>();
   // 4. 
   rclcpp::spin(node);
   // 5.
   rclcpp::shutdown();
  return 0;
}
