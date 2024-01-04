/* Write a subscriber
    Task: subscribe the published messages and output to the terminal 
    Steps:
        1. include header files
        2. initilization 
        3. Define a node
            3.1 create a subscriber
            3.2 receive message and output
        4. spin()
        5. shutdowm()
*/

// 1.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 3. 
class Listener: public rclcpp::Node{
    public:
        Listener():Node("Listener_node_cpp"){
            RCLCPP_INFO(this->get_logger(),"Subscription created!");
            // 3.1
            // std::shared_ptr<SubscriptionT> 
            // create_subscription<MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
            //        const std::string &topic_name, 
            //        const rclcpp::QoS &qos, 
            //        CallbackT &&callback, 
            //        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
            //        MessageMemoryStrategyT::SharedPtr msg_mem_strat = MessageMemoryStrategyT::create_default())

            sub_ = this->create_subscription<std_msgs::msg::String>("chatter",10,
                                std::bind(&Listener::do_cb,this,std::placeholders::_1));
        }
    private:
        // 3.2
        void do_cb(const std_msgs::msg::String &msg){
            RCLCPP_INFO(this->get_logger(),"Received messages: %s", msg.data.c_str());
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[]){
    // 2.
    rclcpp::init(argc,argv);

    // 3.
    auto node = std::make_shared<Listener>();

    // 4.
    rclcpp::spin(node);

    // 5.
    rclcpp::shutdown();

    return 0;
}