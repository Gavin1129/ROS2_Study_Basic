#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// Callback function for the subscription
void callback(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << "Received message: " << msg->data << std::endl;
}

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("listener_node_cpp");

    // Create a Subscription
    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, callback);

    // Manual loop for checking ROS 2 shutdown and polling for messages
    while (rclcpp::ok()) {
        // Manually calling spin_some to handle incoming messages
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(100ms); // Sleep for a short duration
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
