#include "GenericBoard.hpp"

/**
 * 
 * default constrictor
 * 
 * Initialize the interfaces and the 
 */
GenericBoardNode::GenericBoardNode() : Node("Generic_Board")
{
    RCLCPP_INFO(this->get_logger(), "Node starting");

    // parameters
    this->declare_parameter("board_bus_id",            rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("service_length",          rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("watchdog_timeout",        rclcpp::PARAMETER_INTEGER   );

    board_bus_id                    = (uint32_t)(   this->get_parameter("board_bus_id")         .as_int()       );
    service_length                  = (uint32_t)(   this->get_parameter("service_length")       .as_int()       );
    watchdog_timeout                = (uint32_t)(   this->get_parameter("watchdog_timeout")     .as_int()       );

    RCLCPP_INFO(this->get_logger(), "board_bus_id           : %d",      board_bus_id                    );
    RCLCPP_INFO(this->get_logger(), "service_length         : %d",      service_length                  );
    RCLCPP_INFO(this->get_logger(), "watchdog_timeout       : %d ms",   watchdog_timeout                );

    // initialize the CAN message filters from the parameters
    init_filter();

    // create the subscription for received frames
    frame_received_subscriber_ = this->create_subscription<hw_support_interfaces_pkg::msg::CanFrame>(
        "received_frame",
        10,
        std::bind(
            &GenericBoardNode::callback_frame_received_subscriber,
            this,
            std::placeholders::_1
        )
    );

    // create the topic publisher for the board status
    board_status_publisher_ = this->create_publisher<hw_support_interfaces_pkg::msg::BoardStatus>("board_status", 10);

    // create the service server for frames to send
    reset_board_service_ = this->create_service<hw_support_interfaces_pkg::srv::ResetBoard>(
        "reset_board",
        std::bind(
            &GenericBoardNode::callback_reset_board_service,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    RCLCPP_INFO(this->get_logger(), "Node started");
}

GenericBoardNode::~GenericBoardNode()
{
    RCLCPP_INFO(this->get_logger(), "Node cleanly destroyed");
}

void GenericBoardNode::callback_frame_received_subscriber(
    const hw_support_interfaces_pkg::msg::CanFrame::SharedPtr message
)
{
    uint32_t service_id;
    
    if(!filter_frame(message))
    {
        return;
    }

    service_id = get_service_id(message);

    on_frame_received(service_id, message->length, message->data);

}

void GenericBoardNode::callback_reset_board_service(
    const hw_support_interfaces_pkg::srv::ResetBoard::Request::SharedPtr request,
    const hw_support_interfaces_pkg::srv::ResetBoard::Response::SharedPtr response
)
{
    (void)request; // unused request

    // unimplemented in the FW of the boards for now

    response->status = true;
}

void GenericBoardNode::init_filter(void)
{
    address_filter = board_bus_id << service_length;
    address_mask = 0x07FF & (0xFFFF << (service_length-1));
}


bool GenericBoardNode::filter_frame(const hw_support_interfaces_pkg::msg::CanFrame::SharedPtr frame)
{
    uint32_t masked_id;
    masked_id = frame->id & address_mask;
    return masked_id == address_filter;
}

uint32_t GenericBoardNode::get_service_id(const hw_support_interfaces_pkg::msg::CanFrame::SharedPtr frame)
{
    return frame->id & ~address_mask;
}

void GenericBoardNode::on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data)
{
    (void)length; // UNUSED
    (void)data; // UNUSED

    RCLCPP_DEBUG(this->get_logger(), "Message received, service=%02d", service);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GenericBoardNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
