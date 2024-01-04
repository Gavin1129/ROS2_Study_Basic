#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char* argv[]){
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("listener_node_cpp");

    // Create a Subscription
    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "chatter", 
        10, 
        [node](const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(node->get_logger(), "Received message: %s", msg->data.c_str());
        }
    );

    // Spin to handle callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
