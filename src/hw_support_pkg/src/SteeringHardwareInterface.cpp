#include "SteeringHardwareInterface.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

SteeringHardwareInterface::SteeringHardwareInterface():
    m_logger(rclcpp::get_logger("SteeringHardwareInterface"))
{
    m_node = rclcpp::Node::make_shared("SteeringHardwareInterfaceNode");
}

SteeringHardwareInterface::CallbackReturn SteeringHardwareInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    SteeringHardwareInterface::CallbackReturn return_value;


    RCLCPP_INFO(m_logger, "Start initilisation");

    /**
     * Set the parameters
    */
    return_value = get_parameter_in_hardware_info(hardware_info);
    if (return_value != SteeringHardwareInterface::CallbackReturn::SUCCESS)
    {
        return return_value;
    }


    // create the subscription for motors state
    m_motors_state_subscriber = m_node->create_subscription<hw_support_interfaces_pkg::msg::MotorsState>(
        "motors_state",
        10,
        std::bind(
            &SteeringHardwareInterface::callback_on_receive_motors_state,
            this,
            std::placeholders::_1
        )
    );

    // create the subscription for encoders state
    m_encoders_state_subscriber = m_node->create_subscription<hw_support_interfaces_pkg::msg::EncodersPosition>(
        "encoders_position",
        10,
        std::bind(
            &SteeringHardwareInterface::callback_on_receive_encoders_state,
            this,
            std::placeholders::_1
        )
    );



}

SteeringHardwareInterface::CallbackReturn SteeringHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(m_logger, "Start activation");
    m_set_enabled = true;
    m_set_speed_left = 0;
    m_set_speed_right = 0;
    send_motor_parameters();
}

SteeringHardwareInterface::CallbackReturn SteeringHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(m_logger, "Start deactivation");
    RCLCPP_INFO(m_logger, "Start activation");
    m_set_enabled = false;
    m_set_speed_left = 0;
    m_set_speed_right = 0;
    send_motor_parameters();
}

std::vector<hardware_interface::StateInterface> SteeringHardwareInterface::export_state_interfaces(void)
{
    RCLCPP_INFO(m_logger, "Start export state interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_left_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_encoder_speed_left));
    state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_left_wheel_name, hardware_interface::HW_IF_POSITION, &m_encoder_position_left));
    state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_right_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_encoder_speed_right));
    state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_right_wheel_name, hardware_interface::HW_IF_POSITION, &m_encoder_position_right));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SteeringHardwareInterface::export_command_interfaces(void)
{
    RCLCPP_INFO(m_logger, "Start export command interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(m_param_left_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_set_speed_left));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(m_param_right_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_set_speed_right));

    return command_interfaces;
}

hardware_interface::return_type SteeringHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // NOP as the encoder values are published at fixed frequency
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SteeringHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if( send_motor_parameters() )
    {
        return hardware_interface::return_type::OK;
    }
    return hardware_interface::return_type::ERROR;
}


std::string SteeringHardwareInterface::get_parameter_in_hardware_info_select(const hardware_interface::HardwareInfo & hardware_info, const std::string name)
{
    std::unordered_map<std::string, std::string> parameters;
    std::unordered_map<std::string, std::string>::iterator position;

    parameters = hardware_info.hardware_parameters;

    position = parameters.find(name);

    if (position == parameters.end())
    {
        return "invalid";
    }

    return position->second;
}

SteeringHardwareInterface::CallbackReturn SteeringHardwareInterface::get_parameter_in_hardware_info(const hardware_interface::HardwareInfo & hardware_info)
{
    std::string value;

    value = get_parameter_in_hardware_info_select(hardware_info, "left_wheel_name");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter left_wheel_name undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_left_wheel_name = value;

    value = get_parameter_in_hardware_info_select(hardware_info, "right_wheel_name");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter right_wheel_name undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_right_wheel_name = value;

    value = get_parameter_in_hardware_info_select(hardware_info, "motor_step_per_revolution");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter motor_step_per_revolution undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_motor_step_per_rev = std::stoi(value);

    value = get_parameter_in_hardware_info_select(hardware_info, "encoder_step_per_rev");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter encoder_step_per_rev undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_encoder_step_per_rev = std::stoi(value);

    value = get_parameter_in_hardware_info_select(hardware_info, "invert_motor_left");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter invert_motor_left undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_invert_motor_left = (value == "true");

    value = get_parameter_in_hardware_info_select(hardware_info, "invert_motor_right");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter invert_motor_right undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_invert_motor_right = (value == "true");

    value = get_parameter_in_hardware_info_select(hardware_info, "invert_encoder_left");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter invert_encoder_left undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_invert_encoder_left = (value == "true");

    value = get_parameter_in_hardware_info_select(hardware_info, "invert_encoder_right");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter invert_encoder_right undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_invert_encoder_right = (value == "true");

    value = get_parameter_in_hardware_info_select(hardware_info, "motors_acceleration");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter motors_acceleration undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_motor_acceleration = std::stod(value);

    value = get_parameter_in_hardware_info_select(hardware_info, "encoders_read_frequency");
    if (value == "invalid")
    {
        RCLCPP_FATAL(m_logger, "parameter encoders_read_frequency undefined");
        return SteeringHardwareInterface::CallbackReturn::ERROR;
    }
    m_param_encoder_read_frequency = std::stod(value);

    

    return SteeringHardwareInterface::CallbackReturn::SUCCESS;
}

void SteeringHardwareInterface::callback_on_receive_motors_state(
    hw_support_interfaces_pkg::msg::MotorsState motors_state
)
{
    m_real_enabled = motors_state.left_enable && motors_state.right_enable;
    m_motor_position_left = motor_compute_position(motors_state.left_position);
    m_motor_position_right = motor_compute_position(motors_state.right_position);
    m_motor_speed_left = motor_compute_position(motors_state.left_speed);
    m_motor_speed_right = motor_compute_position(motors_state.right_speed);
}

void SteeringHardwareInterface::callback_on_receive_encoders_state(
    hw_support_interfaces_pkg::msg::EncodersPosition encoders_state
)
{
    m_encoder_position_left = encoder_compute_position(encoders_state.left_raw);
    m_encoder_speed_left = encoder_compute_speed(encoders_state.left_delta);
    m_encoder_position_right = encoder_compute_position(encoders_state.right_raw);
    m_encoder_speed_right = encoder_compute_speed(encoders_state.right_delta);
}


double SteeringHardwareInterface::motor_compute_position(int32_t raw)
{
    return (double)raw * M_2_PIf64 / (double)m_param_motor_step_per_rev;
}

double SteeringHardwareInterface::motor_compute_speed(int32_t raw)
{
    return (double)raw * M_2_PIf64 / (double)m_param_motor_step_per_rev;
}

int32_t SteeringHardwareInterface::motor_encode_speed(double speed)
{
    return (int32_t)(speed * (double)m_param_motor_step_per_rev / M_2_PIf64);
}

uint32_t SteeringHardwareInterface::motor_encode_acceleration(double acc)
{
    return (int32_t)(acc * (double)m_param_motor_step_per_rev / M_2_PIf64);
}

double SteeringHardwareInterface::encoder_compute_position(int32_t raw)
{
    return (double)raw * M_2_PIf64 / (double)m_param_encoder_step_per_rev;
}

double SteeringHardwareInterface::encoder_compute_speed(int32_t raw)
{
    return (double)raw * M_2_PIf64 / (double)m_param_encoder_step_per_rev * m_param_encoder_read_frequency;
}

bool SteeringHardwareInterface::send_motor_parameters(void)
{
    RCLCPP_DEBUG(m_logger, "enter sending motors parameter");

    auto client = m_node->create_client<hw_support_interfaces_pkg::srv::MotorsSettings>("send_frame");

    RCLCPP_DEBUG(m_logger, "client created");

    while(!client->wait_for_service(std::chrono::seconds(1))) // waits for the service to be up
    {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_logger, "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_DEBUG(m_logger, "waiting for sender service to be up");
    }

    RCLCPP_DEBUG(m_logger, "service up");

    auto request = std::make_shared<hw_support_interfaces_pkg::srv::MotorsSettings::Request>();

    request->left_enable = m_set_enabled;
    request->right_enable = m_set_enabled;
    request->left_set_acc = motor_encode_acceleration(m_param_motor_acceleration);
    request->right_set_acc = motor_encode_acceleration(m_param_motor_acceleration);
    request->left_set_speed = motor_encode_speed(m_motor_speed_left);
    request->right_set_speed = motor_encode_speed(m_motor_speed_right);

    RCLCPP_DEBUG(m_logger, "request creation");

    auto future = client->async_send_request(request);

    return true;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  SteeringHardwareInterface,
  hardware_interface::SystemInterface
)