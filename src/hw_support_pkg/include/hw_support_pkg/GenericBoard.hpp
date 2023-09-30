#ifndef GENERIC_BOARD_HPP__
#define GENERIC_BOARD_HPP__

#include "rclcpp/rclcpp.hpp"
#include "hw_support_interfaces_pkg/msg/can_frame.hpp"
#include "hw_support_interfaces_pkg/msg/board_status.hpp"
#include "hw_support_interfaces_pkg/srv/can_frame.hpp"
#include "hw_support_interfaces_pkg/srv/reset_board.hpp"

class GenericBoardNode : public rclcpp::Node
{
public :
    GenericBoardNode();
    ~GenericBoardNode();

private :

    // parameters
    uint32_t board_bus_id;
    uint32_t service_length;
    uint32_t watchdog_timeout;

    uint32_t address_filter;
    uint32_t address_mask;

    rclcpp::Subscription<hw_support_interfaces_pkg::msg::CanFrame>::SharedPtr frame_received_subscriber_;
    rclcpp::Publisher<hw_support_interfaces_pkg::msg::BoardStatus>::SharedPtr board_status_publisher_;
    rclcpp::Service<hw_support_interfaces_pkg::srv::ResetBoard>::SharedPtr reset_board_service_;

    void callback_frame_received_subscriber(
        const hw_support_interfaces_pkg::msg::CanFrame::SharedPtr message
    );

    void callback_reset_board_service(
        const hw_support_interfaces_pkg::srv::ResetBoard::Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::ResetBoard::Response::SharedPtr response
    );

    void init_filter(void);
    bool filter_frame(const hw_support_interfaces_pkg::msg::CanFrame::SharedPtr frame);
    uint32_t get_service_id(const hw_support_interfaces_pkg::msg::CanFrame::SharedPtr frame);

    virtual void on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data);
};

#endif // GENERIC_BOARD_HPP__