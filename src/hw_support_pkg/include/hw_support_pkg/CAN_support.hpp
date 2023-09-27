#ifndef CAN_SUPPORT_HPP__
#define CAN_SUPPORT_HPP__

#include "rclcpp/rclcpp.hpp"
#include "hw_support_interfaces_pkg/msg/can_frame.hpp"
#include "hw_support_interfaces_pkg/srv/can_frame.hpp"

class CanSupportNode : public rclcpp::Node
{
public :
    CanSupportNode();
    ~CanSupportNode();

private :

    std::string parameter_interface_file;
    uint32_t parameter_baudrate;
    uint32_t parameter_filter_addr;
    uint32_t parameter_filter_mask;

    rclcpp::Publisher<hw_support_interfaces_pkg::msg::CanFrame>::SharedPtr can_frame_receiver_publisher_;
    
    rclcpp::Service<hw_support_interfaces_pkg::srv::CanFrame>::SharedPtr can_frame_sender_service_;

    void callback_can_frame_sender_server(
        const hw_support_interfaces_pkg::srv::CanFrame::Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::CanFrame::Response::SharedPtr response
    );
};

#endif // CAN_SUPPORT_HPP__