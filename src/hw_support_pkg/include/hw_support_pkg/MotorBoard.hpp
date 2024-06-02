#ifndef MOTOR_BOARD_HPP__
#define MOTOR_BOARD_HPP__

#include "GenericBoard.hpp"

#include "hw_support_interfaces_pkg/msg/motors_state.hpp"
#include "hw_support_interfaces_pkg/srv/motors_settings.hpp"

#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

namespace motor_board
{
    class StepperMotor
    {
    public:

        int32_t position;
        int32_t speed;
        bool enable;

        int32_t set_speed;
        uint16_t set_acc;

    };

};

class MotorBoardNode : public GenericBoardNode
{
public :
    MotorBoardNode();
    ~MotorBoardNode();

private :

    motor_board::StepperMotor left_motor, right_motor;
    rclcpp::Service<hw_support_interfaces_pkg::srv::MotorsSettings>::SharedPtr motors_settings_service_;
    rclcpp::Publisher<hw_support_interfaces_pkg::msg::MotorsState>::SharedPtr motors_state_publisher_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr motors_set_speeds_subsciber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motors_enable_subscriber_;

    void publish_motors_state(void);

    void callback_motors_settings_service(
        const hw_support_interfaces_pkg::srv::MotorsSettings::Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::MotorsSettings::Response::SharedPtr response
    );

    void callback_on_set_speed_received(std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void callback_on_enable_received(std_msgs::msg::Bool::SharedPtr msg);
    
    void on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data);

    void decode_status(std::vector<uint8_t> data);
    void decode_current_speeds(std::vector<uint8_t> data);
    void decode_current_positions(std::vector<uint8_t> data);
    void decode_enables(std::vector<uint8_t> data);
    void decode_set_speeds(std::vector<uint8_t> data);
    void decode_set_accelerations(std::vector<uint8_t> data);

    bool set_enables(bool left, bool right);
    bool set_accelerations(uint16_t left, uint16_t right);
    bool set_speeds(int32_t left, int32_t right);





};

#endif // MOTOR_BOARD_HPP__