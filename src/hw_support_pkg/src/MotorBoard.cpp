#include "MotorBoard.hpp"
#include "MotorBoardProtocol.hpp"

#define MOTOR_BOARD_SERVO_COUNT 16
#define MOTOR_BOARD_MOTOR_COUNT 2
#define MOTOR_BOARD_GPIO_COUNT 4
#define MOTOR_BOARD_ADC_COUNT 3
#define MOTOR_BOARD_ADC_MAX_VALUE 0xFFFF
#define MOTOR_BOARD_ADC_SERVO_SCALE 36.3f
#define MOTOR_BOARD_ADC_IN_SCALE 3.3f

MotorBoardNode::MotorBoardNode() : GenericBoardNode("Motor_Board")
{
    motors_state_publisher_ = this->create_publisher<hw_support_interfaces_pkg::msg::MotorsState>(
        "motors_state",
        10
    );

    motors_settings_service_ = this->create_service<hw_support_interfaces_pkg::srv::MotorsSettings>(
        "motors_settings",
        std::bind(
            &MotorBoardNode::callback_motors_settings_service,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    motors_enable_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "motors_enable",
        2,
        std::bind(
            &MotorBoardNode::callback_on_enable_received,
            this,
            std::placeholders::_1
        )
    );

    motors_set_speeds_subsciber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "motors_set_speeds",
        2,
        std::bind(
            &MotorBoardNode::callback_on_set_speed_received,
            this,
            std::placeholders::_1
        )
    );
}

MotorBoardNode::~MotorBoardNode()
{

}

void MotorBoardNode::on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data)
{
    RCLCPP_DEBUG(this->get_logger(), "Message received, service=%02d", service);

    if ( service >= motor_board::services_t::MAX )
    {
        RCLCPP_WARN(this->get_logger(), "unknown service, service=%02d", service);
        return;
    }

    switch ( (motor_board::services_t)service )
    {
    case motor_board::services_t::ID :
        RCLCPP_DEBUG(this->get_logger(), "Heartbeat received");       
        break;

    case motor_board::services_t::STATUS :
        if ( length != MOTOR_BOARD_PROTOCOL_RX_SERVICE_STATUS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_status(data);
        break;

    case motor_board::services_t::CURRENT_SPEEDS :
        if ( length != MOTOR_BOARD_PROTOCOL_RX_SERVICE_CURRENT_SPEEDS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_current_speeds(data);
        break;

    case motor_board::services_t::CURRENT_POSITIONS :
        if ( length != MOTOR_BOARD_PROTOCOL_RX_SERVICE_CURRENT_POSITIONS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_current_positions(data);
        publish_motors_state(); // publish motors states when the positions are updated
        break;

    case motor_board::services_t::ENABLE :
        if ( length != MOTOR_BOARD_PROTOCOL_RX_SERVICE_ENABLE_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_enables(data);
        break;

    case motor_board::services_t::SET_SPEEDS :
        if ( length != MOTOR_BOARD_PROTOCOL_RX_SERVICE_SET_SPEEDS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_set_speeds(data);
        break;

    case motor_board::services_t::SET_ACCELERATIONS :
        if ( length != MOTOR_BOARD_PROTOCOL_RX_SERVICE_SET_ACCELERATIONS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_set_accelerations(data);
        break;


    default:
        RCLCPP_WARN(this->get_logger(), "unhandled service, service=%02d", service);
        break;
    }
}

void MotorBoardNode::decode_status(std::vector<uint8_t> data)
{
    set_status_vector(data[0]);
    // TODO : complete the status decoding to extract data from the vector
}

void MotorBoardNode::decode_current_speeds(std::vector<uint8_t> data)
{
    uint32_t speed_l, speed_r;

    speed_l  = ( (uint32_t)(data[0]) <<  0 ) & 0x000000FF;
    speed_l |= ( (uint32_t)(data[1]) <<  8 ) & 0x0000FF00;
    speed_l |= ( (uint32_t)(data[2]) << 16 ) & 0x00FF0000;
    speed_l |= ( (uint32_t)(data[3]) << 24 ) & 0xFF000000;

    speed_r  = ( (uint32_t)(data[4]) <<  0 ) & 0x000000FF;
    speed_r |= ( (uint32_t)(data[5]) <<  8 ) & 0x0000FF00;
    speed_r |= ( (uint32_t)(data[6]) << 16 ) & 0x00FF0000;
    speed_r |= ( (uint32_t)(data[7]) << 24 ) & 0xFF000000;

    left_motor.speed = (int32_t)speed_l;
    right_motor.speed = (int32_t)speed_r;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor speeds : [%d, %d] Hz",
        left_motor.speed, right_motor.speed
    );
}

void MotorBoardNode::decode_current_positions(std::vector<uint8_t> data)
{

    uint32_t pos_l, pos_r;

    pos_l  = ( (uint32_t)(data[0]) <<  0 ) & 0x000000FF;
    pos_l |= ( (uint32_t)(data[1]) <<  8 ) & 0x0000FF00;
    pos_l |= ( (uint32_t)(data[2]) << 16 ) & 0x00FF0000;
    pos_l |= ( (uint32_t)(data[3]) << 24 ) & 0xFF000000;

    pos_r  = ( (uint32_t)(data[4]) <<  0 ) & 0x000000FF;
    pos_r |= ( (uint32_t)(data[5]) <<  8 ) & 0x0000FF00;
    pos_r |= ( (uint32_t)(data[6]) << 16 ) & 0x00FF0000;
    pos_r |= ( (uint32_t)(data[7]) << 24 ) & 0xFF000000;

    left_motor.position = (int32_t)pos_l;
    right_motor.position = (int32_t)pos_r;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor positions : [%d, %d] steps",
        left_motor.position, right_motor.position
    );
}

void MotorBoardNode::decode_enables(std::vector<uint8_t> data)
{
    left_motor.enable = data[0] & 0x01;
    right_motor.enable = data[1] & 0x01;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor enables : [%d, %d]",
        left_motor.enable, right_motor.enable
    );
}

void MotorBoardNode::decode_set_speeds(std::vector<uint8_t> data)
{
    uint32_t speed_l, speed_r;

    speed_l  = ( (uint32_t)(data[0]) <<  0 ) & 0x000000FF;
    speed_l |= ( (uint32_t)(data[1]) <<  8 ) & 0x0000FF00;
    speed_l |= ( (uint32_t)(data[2]) << 16 ) & 0x00FF0000;
    speed_l |= ( (uint32_t)(data[3]) << 24 ) & 0xFF000000;

    speed_r  = ( (uint32_t)(data[4]) <<  0 ) & 0x000000FF;
    speed_r |= ( (uint32_t)(data[5]) <<  8 ) & 0x0000FF00;
    speed_r |= ( (uint32_t)(data[6]) << 16 ) & 0x00FF0000;
    speed_r |= ( (uint32_t)(data[7]) << 24 ) & 0xFF000000;

    left_motor.set_speed = (int32_t)speed_l;
    right_motor.set_speed = (int32_t)speed_r;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor speed targets : [%d, %d] Hz",
        left_motor.set_speed, right_motor.set_speed
    );
}

void MotorBoardNode::decode_set_accelerations(std::vector<uint8_t> data)
{
    uint32_t acc_l, acc_r;

    acc_l   = ( (uint16_t)(data[0]) << 0 ) & 0x00FF;
    acc_l  |= ( (uint16_t)(data[1]) << 8 ) & 0xFF00;

    acc_r   = ( (uint16_t)(data[2]) << 0 ) & 0x00FF;
    acc_r  |= ( (uint16_t)(data[3]) << 8 ) & 0xFF00;

    left_motor.set_acc = acc_l;
    right_motor.set_acc = acc_r;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor acc setting : [%d, %d]",
        left_motor.set_acc, right_motor.set_acc
    );
}

void MotorBoardNode::publish_motors_state(void)
{
    auto to_publish = hw_support_interfaces_pkg::msg::MotorsState();

    to_publish.left_position    = left_motor.position;
    to_publish.left_speed       = left_motor.speed;
    to_publish.left_enable      = left_motor.enable;
    to_publish.left_set_speed   = left_motor.set_speed;
    to_publish.left_set_acc     = left_motor.set_acc;

    to_publish.right_position    = right_motor.position;
    to_publish.right_speed       = right_motor.speed;
    to_publish.right_enable      = right_motor.enable;
    to_publish.right_set_speed   = right_motor.set_speed;
    to_publish.right_set_acc     = right_motor.set_acc;

    motors_state_publisher_->publish(to_publish);
}

void MotorBoardNode::callback_motors_settings_service(
    const hw_support_interfaces_pkg::srv::MotorsSettings::Request::SharedPtr request,
    const hw_support_interfaces_pkg::srv::MotorsSettings::Response::SharedPtr response
)
{
    RCLCPP_DEBUG(this->get_logger(), "sending enables");

    if ( !set_enables(request->left_enable, request->right_enable) )
    {
        response->status = false;
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "sending speeds");

    if ( !set_speeds(request->left_set_speed, request->right_set_speed) )
    {
        response->status = false;
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "sending accelerations");

    if ( !set_accelerations(request->left_set_acc, request->right_set_acc) )
    {
        response->status = false;
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "sending sucessful");

    response->status = true;
}

void MotorBoardNode::callback_on_set_speed_received(std_msgs::msg::Int32MultiArray::SharedPtr msg)
{

    if (msg->data.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "set speeds command received but empty");
        return;
    }

    if (msg->data.size() != 2)
    {
        RCLCPP_ERROR(this->get_logger(), "set speeds command received but size does not match (%lu != 2)", msg->data.size());
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "set speeds : %d|%d", msg->data[0], msg->data[1]);
}

void MotorBoardNode::callback_on_enable_received(std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "set enables : %d", msg->data);
    set_enables(msg->data, msg->data);
}

bool MotorBoardNode::set_enables(bool left, bool right)
{
    std::vector<uint8_t> data;
    data.push_back(left);
    data.push_back(right);
    return send_frame(motor_board::services_t::ENABLE, data);
}

bool MotorBoardNode::set_accelerations(uint16_t left, uint16_t right)
{
    std::vector<uint8_t> data;

    data.push_back(
        (uint8_t)(left & 0x00FF)
    );
    data.push_back(
        (uint8_t)((left >> 8) & 0x00FF)
    );
    data.push_back(
        (uint8_t)(right & 0x00FF)
    );
    data.push_back(
        (uint8_t)((right >> 8) & 0x00FF)
    );

    return send_frame(motor_board::services_t::SET_ACCELERATIONS, data);
}

bool MotorBoardNode::set_speeds(int32_t left, int32_t right)
{
    std::vector<uint8_t> data;

    data.push_back(
        (uint8_t)(left & 0x00000000FF)
    );
    data.push_back(
        (uint8_t)((left >> 8) & 0x00000000FF)
    );
    data.push_back(
        (uint8_t)((left >> 16) & 0x00000000FF)
    );
    data.push_back(
        (uint8_t)((left >> 24) & 0x00000000FF)
    );


    data.push_back(
        (uint8_t)(right & 0x00000000FF)
    );
    data.push_back(
        (uint8_t)((right >> 8) & 0x00000000FF)
    );
    data.push_back(
        (uint8_t)((right >> 16) & 0x00000000FF)
    );
    data.push_back(
        (uint8_t)((right >> 24) & 0x00000000FF)
    );

    return send_frame(motor_board::services_t::SET_SPEEDS, data);
}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorBoardNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}