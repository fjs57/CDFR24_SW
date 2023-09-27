#ifndef CAN_SUPPORT_HPP__
#define CAN_SUPPORT_HPP__

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"
#include "hw_support_interfaces_pkg/msg/can_frame.hpp"
#include "hw_support_interfaces_pkg/srv/can_frame.hpp"

class CanSupportNode : public rclcpp::Node
{
public :
    CanSupportNode();
    ~CanSupportNode();

private :

    /**
     * ROS Section
     */

    std::string parameter_interface_file;
    uint32_t parameter_baudrate;
    uint32_t parameter_filter_addr;
    uint32_t parameter_filter_mask;
    uint32_t parameter_timeout_seconds;
    uint32_t parameter_timeout_microseconds;
    bool parameter_log_raw_frames;

    rclcpp::Publisher<hw_support_interfaces_pkg::msg::CanFrame>::SharedPtr can_frame_receiver_publisher_;
    
    rclcpp::Service<hw_support_interfaces_pkg::srv::CanFrame>::SharedPtr can_frame_sender_service_;

    void callback_can_frame_sender_server(
        const hw_support_interfaces_pkg::srv::CanFrame::Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::CanFrame::Response::SharedPtr response
    );

    /**
     * Functional section
     */

    std::thread receiver_thread;
    std::mutex send_mutex;

    bool continue_receiving;
    int can_socket;

    bool init_can(void);
    void deinit_can(void);

    void start_receiving_thread(void);
    void receiving_thread_function(void);
    void stop_receiving_thread(void);

    void on_receive(struct can_frame*);

    bool send(struct can_frame frame);

};

#endif // CAN_SUPPORT_HPP__