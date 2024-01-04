#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]){
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("talker_node_cpp");

    // Create a Publisher
    auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", 10);

    size_t count = 0;

    // Create a Wall Timer
    auto timer = node->create_wall_timer(
        1s, 
        [&](){
            auto message = std_msgs::msg::String();
            message.data = "Hello world! " + std::to_string(count++);
            RCLCPP_INFO(node->get_logger(), "Published message: %s", message.data.c_str());
            publisher->publish(message);
        }
    );

    // Spin to handle callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
