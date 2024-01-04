#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    // Check inputs, must be 2 numbers
    if (argc != 3) {
        std::cout << "Please provide two integer numbers" << std::endl;
        return 1;
    }

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a Node
    auto node = std::make_shared<rclcpp::Node>("add_ints_client_node_cpp");

    // Create a client for the add_ints service
    auto client = node->create_client<AddInts>("add_ints");

    // Wait for connection to server
    while (!client->wait_for_service(1s)) {
        // Check Ctrl+c
        if (!rclcpp::ok()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connecting canceled by user!");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connecting to server ...");
    }

    // Send request
    auto request = std::make_shared<AddInts::Request>();
    request->num1 = std::atoi(argv[1]);
    request->num2 = std::atoi(argv[2]);

    auto future = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Connected to server! The sum is: %d", future.get()->sum);
    } else {
        RCLCPP_INFO(node->get_logger(), "Response failed!");
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
