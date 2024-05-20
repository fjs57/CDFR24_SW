#ifndef STEERING_HARDWARE_INTERFACE
#define STEERING_HARDWARE_INTERFACE

#include <cstring>
#include "rclcpp/rclcpp.hpp"

// #include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include "hw_support_interfaces_pkg/msg/motors_state.hpp"
#include "hw_support_interfaces_pkg/msg/encoders_position.hpp"

#include "hw_support_interfaces_pkg/srv/motors_settings.hpp"

namespace steering_hardware_interface
{

class SteeringHardwareInterface : 
    public hardware_interface::SystemInterface
{
public:

    SteeringHardwareInterface();

    CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces(void) override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces(void) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Logger m_logger;

    rclcpp::Subscription<hw_support_interfaces_pkg::msg::MotorsState>::SharedPtr m_motors_state_subscriber;
    rclcpp::Subscription<hw_support_interfaces_pkg::msg::EncodersPosition>::SharedPtr m_encoders_state_subscriber;

    std::string m_param_left_wheel_name, m_param_right_wheel_name;
    uint32_t m_param_motor_step_per_rev;
    uint32_t m_param_encoder_step_per_rev;
    bool m_param_invert_motor_left, m_param_invert_motor_right;
    bool m_param_invert_encoder_left, m_param_invert_encoder_right;
    double m_param_motor_acceleration;
    double m_param_encoder_read_frequency;

    bool m_set_enabled, m_real_enabled;
    double m_set_speed_left, m_set_speed_right;
    double m_motor_speed_left, m_motor_speed_right;
    double m_motor_position_left, m_motor_position_right;
    double m_encoder_speed_left, m_encoder_speed_right;
    double m_encoder_position_left, m_encoder_position_right;


    std::string get_parameter_in_hardware_info_select(const hardware_interface::HardwareInfo & hardware_info, const std::string name);
    CallbackReturn get_parameter_in_hardware_info(const hardware_interface::HardwareInfo & hardware_info);

    void callback_on_receive_motors_state(hw_support_interfaces_pkg::msg::MotorsState motors_state);
    void callback_on_receive_encoders_state(hw_support_interfaces_pkg::msg::EncodersPosition encoders_state);

    double motor_compute_position(int32_t raw);
    double motor_compute_speed(int32_t raw);

    int32_t motor_encode_speed(double speed);
    uint32_t motor_encode_acceleration(double acc);

    double encoder_compute_position(int32_t raw);
    double encoder_compute_speed(int32_t raw);

    bool send_motor_parameters(void);
    


}; 

}

#endif // STEERING_HARDWARE_INTERFACE
