#include "GenericBoard.hpp"

#define GENERIC_BOARD_STATUS_PUBLISH_INTERVAL 500

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

    status_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(GENERIC_BOARD_STATUS_PUBLISH_INTERVAL),
        std::bind(&GenericBoardNode::callback_publish_status, this)
    );

    watchdog_init();

    RCLCPP_INFO(this->get_logger(), "Node started");
}

GenericBoardNode::~GenericBoardNode()
{
    watchdog_timer_cancel();
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

    watchdog_update();

    last_receive_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    service_id = get_service_id(message);

    RCLCPP_DEBUG(this->get_logger(), "frame received, srv:%d timestamp:%ld", service_id, last_receive_timestamp);

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

void GenericBoardNode::watchdog_init(void)
{
    watchdog_timer_set();
    watchdog_state = 0;
    RCLCPP_DEBUG(this->get_logger(), "watchdog initialized");
}

void GenericBoardNode::watchdog_update(void)
{
    watchdog_timer_cancel();
    watchdog_timer_set();
    watchdog_state = 0;
    RCLCPP_INFO(this->get_logger(), "watchdog updated");
}

void GenericBoardNode::watchdog_timer_cancel(void)
{
    if(watchdog_timer_ == nullptr) return;
    watchdog_timer_->cancel();
}

void GenericBoardNode::watchdog_timer_set(void)
{
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(watchdog_timeout),
        std::bind(&GenericBoardNode::callback_watchdog_timeout_trigger, this)
    );
}


void GenericBoardNode::callback_watchdog_timeout_trigger(void)
{
    RCLCPP_INFO(this->get_logger(), "watchdog triggered");
    watchdog_state = 1;
}

void GenericBoardNode::callback_publish_status(void)
{
    auto msg = hw_support_interfaces_pkg::msg::BoardStatus();

    msg.status_vector = status_vector;
    msg.status_string = status_string;
    msg.watchdog_status = watchdog_state;
    msg.last_message_timestamp = last_receive_timestamp;

    board_status_publisher_->publish(msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GenericBoardNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
