#ifndef GENERIC_BOARD_HPP__
#define GENERIC_BOARD_HPP__

#include "rclcpp/rclcpp.hpp"
#include "hw_support_interfaces_pkg/msg/can_frame.hpp"
#include "hw_support_interfaces_pkg/srv/can_frame.hpp"

class GenericBoardNode : public rclcpp::Node
{
public :
    GenericBoardNode();
    ~GenericBoardNode();

private :

    rclcpp::Subscription<hw_support_interfaces_pkg::msg::CanFrame>::SharedPtr can_frame_receiver_subscriber_;
    
    rclcpp::Client<hw_support_interfaces_pkg::srv::CanFrame>::SharedPtr can_frame_sender_client_;

    void callback_can_frame_receiver_subscriber(
        const hw_support_interfaces_pkg::srv::CanFrame::Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::CanFrame::Response::SharedPtr response
    );
};

#endif GENERIC_BOARD_HPP__