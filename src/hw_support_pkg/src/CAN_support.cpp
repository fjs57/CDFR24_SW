#include "CAN_support.hpp"

/**
 * 
 * default constrictor
 * 
 * Initialize the interfaces and the 
 */
CanSupportNode::CanSupportNode() : Node("CAN_support")
{
    RCLCPP_INFO(this->get_logger(), "CAN Support Node starting");

    // parameters
    this->declare_parameter("interface_file",   rclcpp::PARAMETER_STRING    );
    this->declare_parameter("baudrate",         rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("filter_addr",      rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("filter_mask",      rclcpp::PARAMETER_INTEGER   );

    parameter_interface_file    =               this->get_parameter("interface_file")   .as_string()     ;
    parameter_baudrate          = (uint32_t)(   this->get_parameter("baudrate")         .as_int()       );
    parameter_filter_addr       = (uint32_t)(   this->get_parameter("filter_addr")      .as_int()       );
    parameter_filter_mask       = (uint32_t)(   this->get_parameter("filter_mask")      .as_int()       );

    RCLCPP_INFO(this->get_logger(), "interface_file : %s",      parameter_interface_file.c_str());
    RCLCPP_INFO(this->get_logger(), "baudrate       : %d",      parameter_baudrate      );
    RCLCPP_INFO(this->get_logger(), "filter_addr    : 0x%08x",  parameter_filter_addr   );
    RCLCPP_INFO(this->get_logger(), "filter_mask    : 0x%08x",  parameter_filter_mask   );

    // create the topic publisher for received can frames
    can_frame_receiver_publisher_ = this->create_publisher<hw_support_interfaces_pkg::msg::CanFrame>("received_frame", 10);

    // create the service server for frames to send
    can_frame_sender_service_ = this->create_service<hw_support_interfaces_pkg::srv::CanFrame>(
        "send_frame",
        std::bind(
            &CanSupportNode::callback_can_frame_sender_server,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    RCLCPP_INFO(this->get_logger(), "CAN Support Node started");
}

CanSupportNode::~CanSupportNode()
{
    RCLCPP_INFO(this->get_logger(), "CAN Support Node cleanly destroyed");
}

void CanSupportNode::callback_can_frame_sender_server(
    const hw_support_interfaces_pkg::srv::CanFrame::Request::SharedPtr request,
    const hw_support_interfaces_pkg::srv::CanFrame::Response::SharedPtr response
)
{
    
    response->status = (request->can_frame.length != 0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CanSupportNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}