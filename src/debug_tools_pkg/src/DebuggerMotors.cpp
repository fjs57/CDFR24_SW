#include "rclcpp/rclcpp.hpp"
#include "hw_support_interfaces_pkg/srv/motors_settings.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("debugger_motors");
    rclcpp::Client<hw_support_interfaces_pkg::srv::MotorsSettings>::SharedPtr client =
        node->create_client<hw_support_interfaces_pkg::srv::MotorsSettings>("motors_settings");

    auto request = std::make_shared<hw_support_interfaces_pkg::srv::MotorsSettings::Request>();
    request->left_enable        = atol(argv[1]);
    request->left_set_speed     = atol(argv[2]);
    request->left_set_acc       = atol(argv[3]);
    request->right_enable       = atol(argv[4]);
    request->right_set_speed    = atol(argv[5]);
    request->right_set_acc      = atol(argv[6]);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "left_enable        = %d",
        request->left_enable
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "left_set_speed     = %d",
        request->left_set_speed
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "left_set_acc       = %d",
        request->left_set_acc
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "right_enable       = %d",
        request->right_enable
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "right_set_speed    = %d",
        request->right_set_speed
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "right_set_acc      = %d",
        request->right_set_acc
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "starting service client"
    );

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result: %d", result.get()->status);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "ending service client"
    );

    rclcpp::shutdown();
    return 0;
}