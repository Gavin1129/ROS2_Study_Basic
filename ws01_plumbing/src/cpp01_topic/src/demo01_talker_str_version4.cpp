#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

void publish_message(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher) {
    size_t count = 0;
    while (rclcpp::ok()) {
        auto message = std_msgs::msg::String();
        message.data = "Hello world! " + std::to_string(count++);
        RCLCPP_INFO(rclcpp::get_logger("publisher"), "Publishing: '%s'", message.data.c_str());
        publisher->publish(message);
        std::this_thread::sleep_for(1s); // Sleep for 1 second
    }
}

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("talker_node_cpp");

    // Create a Publisher
    auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Create a thread for publishing messages
    std::thread publish_thread(publish_message, publisher);

    // Manual loop for checking ROS 2 shutdown
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(1s); // Sleep for 1 second or a suitable duration
    }

    // Wait for the publishing thread to finish
    if (publish_thread.joinable()) {
        publish_thread.join();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
