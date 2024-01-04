#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Global callback function for the subscription
void do_cb(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << "Received message: " << msg->data.c_str() << std::endl;
}

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("listener_node_cpp");

    // Create a Subscription
    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "chatter", 
        10, 
        do_cb
    );

    // Spin to handle callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
