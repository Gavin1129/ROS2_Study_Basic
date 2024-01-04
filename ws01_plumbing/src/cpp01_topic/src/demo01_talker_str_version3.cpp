#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Global variables for publisher and message count
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher;
size_t count = 0;

// Timer callback function
void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello world! " + std::to_string(count++);
    RCLCPP_INFO(rclcpp::get_logger("publisher"), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
}

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("talker_node_cpp");

    // Create a Publisher
    publisher = node->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Create a Wall Timer
    auto timer = node->create_wall_timer(
        1s, 
        timer_callback
    );

    // Spin to handle callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
