// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_pkg
{

	DiffBotSystemHardware::DiffBotSystemHardware():
		m_logger(rclcpp::get_logger("DiffBotSystemHardwareNode"))
	{
        RCLCPP_ERROR(m_logger, "Start constructor");
        RCLCPP_ERROR(m_logger, "End constructor");
	}

	hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
  	{
		hardware_interface::CallbackReturn return_value;

		RCLCPP_INFO(m_logger, "Start initilization");

		if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
		{
			RCLCPP_ERROR(m_logger, "initialization failed, failed to initialize base class");
			return hardware_interface::CallbackReturn::ERROR;
		}

		return_value = get_parameter_in_hardware_info(info);
        if (return_value != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(m_logger, "initialization failed, failed to initialize parameters");
            return return_value;
        }


		// for (const hardware_interface::ComponentInfo & joint : info_.joints)
		// {
		// 	// DiffBotSystem has exactly two states and one command interface on each joint
		// 	if (joint.command_interfaces.size() != 1)
		// 	{
		// 		RCLCPP_FATAL(
		// 		m_logger,
		// 		"Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
		// 		joint.command_interfaces.size());
		// 		return hardware_interface::CallbackReturn::ERROR;
		// 	}

		// 	if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
		// 	{
		// 		RCLCPP_FATAL(
		// 		m_logger,
		// 		"Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
		// 		joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
		// 		return hardware_interface::CallbackReturn::ERROR;
		// 	}

		// 	if (joint.state_interfaces.size() != 2)
		// 	{
		// 		RCLCPP_FATAL(
		// 		m_logger,
		// 		"Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
		// 		joint.state_interfaces.size());
		// 		return hardware_interface::CallbackReturn::ERROR;
		// 	}

		// 	if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		// 	{
		// 		RCLCPP_FATAL(
		// 		m_logger,
		// 		"Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
		// 		joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
		// 		return hardware_interface::CallbackReturn::ERROR;
		// 	}

		// 	if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
		// 	{
		// 		RCLCPP_FATAL(
		// 		m_logger,
		// 		"Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
		// 		joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
		// 		return hardware_interface::CallbackReturn::ERROR;
		// 	}
		// }


        // // create the subscription for motors state
        // m_motors_state_subscriber = m_node->create_subscription<hw_support_interfaces_pkg::msg::MotorsState>(
        //     "motors_state",
        //     10,
        //     std::bind(
        //         &DiffBotSystemHardware::callback_on_receive_motors_state,
        //         this,
        //         std::placeholders::_1
        //     )
        // );

        // create the subscription for encoders state
        // m_encoders_state_subscriber = m_node->create_subscription<hw_support_interfaces_pkg::msg::EncodersPosition>(
        //     "encoders_position",
        //     10,
        //     std::bind(
        //         &DiffBotSystemHardware::callback_on_receive_encoders_state,
        //         this,
        //         std::placeholders::_1
        //     )
        // );

        m_node = rclcpp::Node::make_shared("DiffBotSystemHardwareNode");

        // create the subscription for motors state
        m_motors_state_subscriber = m_node->create_subscription<hw_support_interfaces_pkg::msg::MotorsState>(
            "motors_state",
            10,
            std::bind(
                &DiffBotSystemHardware::callback_on_receive_motors_state,
                this,
                std::placeholders::_1
            )
        );

        // create the subscription for encoders state
        m_encoders_state_subscriber = m_node->create_subscription<hw_support_interfaces_pkg::msg::EncodersPosition>(
            "encoders_position",
            10,
            std::bind(
                &DiffBotSystemHardware::callback_on_receive_encoders_state,
                this,
                std::placeholders::_1
            )
        );

        m_executor.add_node(m_node);
        std::thread([this](){m_executor.spin();}).detach();

        RCLCPP_INFO(m_logger, "End initialization");

    	return hardware_interface::CallbackReturn::SUCCESS;
  	}

	std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
	{
		RCLCPP_INFO(m_logger, "Start export state interfaces");

        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_left_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_encoder_speed_left));
        state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_left_wheel_name, hardware_interface::HW_IF_POSITION, &m_encoder_position_left));
        state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_right_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_encoder_speed_right));
        state_interfaces.emplace_back(hardware_interface::StateInterface(m_param_right_wheel_name, hardware_interface::HW_IF_POSITION, &m_encoder_position_right));

		RCLCPP_INFO(m_logger, "End export state interfaces");

        return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
	{
        RCLCPP_INFO(m_logger, "Start export command interfaces");

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(m_param_left_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_cmd_motor_speed_left));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(m_param_right_wheel_name, hardware_interface::HW_IF_VELOCITY, &m_cmd_motor_speed_right));

		RCLCPP_INFO(m_logger, "End export command interfaces");

        return command_interfaces;
	}

	hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
		RCLCPP_INFO(m_logger, "Start activation");

		m_set_enabled = true;
		m_set_speed_left = 0;
		m_set_speed_right = 0;
		
		send_motor_parameters();

        RCLCPP_INFO(m_logger, "End activation");

		return hardware_interface::CallbackReturn::SUCCESS;
	}

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
		RCLCPP_INFO(m_logger, "Start deactivation");

        m_set_enabled = true;
		m_set_speed_left = 0;
		m_set_speed_right = 0;
		
		send_motor_parameters();

        RCLCPP_INFO(m_logger, "End deactivation");
		
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
	{
        //return hardware_interface::return_type::OK;

		// TODO : compute speed based on the period given in argument
        RCLCPP_DEBUG(m_logger, "Read");

        return hardware_interface::return_type::OK;
        /*

        auto client = m_node->create_client<hw_support_interfaces_pkg::srv::GetEncoderPosition>("get_encoder_position");

        RCLCPP_INFO(m_logger, "client created");

        while(!client->wait_for_service(std::chrono::seconds(1))) // waits for the service to be up
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(m_logger, "Interrupted while waiting for the service. Exiting.");
                return hardware_interface::return_type::ERROR;
            }
            RCLCPP_INFO(m_logger, "waiting for sender service to be up");
        }

        RCLCPP_INFO(m_logger, "service up");

        auto request = std::make_shared<hw_support_interfaces_pkg::srv::GetEncoderPosition::Request>();

        RCLCPP_INFO(m_logger, "request creation");

        auto future = client->async_send_request(request);

        //return hardware_interface::return_type::OK;

        try
        {
            
            if(future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready)
            {
                auto response = future.get();
                RCLCPP_INFO(m_logger, "encoders read %d\t%d", response->left, response->right);
                return hardware_interface::return_type::OK;
            }
            RCLCPP_WARN(m_logger, "Service call timed out");
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(m_logger, "Service call failed");
        }

		return hardware_interface::return_type::OK;
        */
	}

	hardware_interface::return_type robot_pkg::DiffBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
	{
        double linear_speed, angular_speed; //m/s
        double encoder_wheel_radius, motor_wheel_radius;

        RCLCPP_DEBUG(m_logger, "Write");

        motor_wheel_radius = m_param_motor_wheel_diameter/2;
        encoder_wheel_radius = m_param_encoder_wheel_diameter/2;


        // commanded speed (on the encoders wheel)

        RCLCPP_DEBUG(m_logger, "Write > received encoder speeds [%.02f, %.02f]", m_cmd_motor_speed_left, m_cmd_motor_speed_right);

        linear_speed = (m_cmd_motor_speed_left + m_cmd_motor_speed_right) * encoder_wheel_radius / 2;
        angular_speed = (m_cmd_motor_speed_left - m_cmd_motor_speed_right) * encoder_wheel_radius / (2 * m_param_encoder_wheel_separation);

        RCLCPP_DEBUG(m_logger, "Write > computed robot speeds [l=%.02f, a=%.02f]", linear_speed, angular_speed);

        // resultant speed after dapatation for motors placement

        m_motor_speed_left = (linear_speed + angular_speed * m_param_motor_wheel_separation) / motor_wheel_radius;
        m_motor_speed_right = (linear_speed - angular_speed * m_param_motor_wheel_separation) / motor_wheel_radius;

        RCLCPP_DEBUG(m_logger, "Write > computed motor speeds [%.02f, %.02f]", m_motor_speed_left, m_motor_speed_right);

        if (m_param_invert_motor_left) m_motor_speed_left *= -1;
        if (m_param_invert_motor_right) m_motor_speed_right *= -1;

        
        if( send_motor_parameters() )
        {
            RCLCPP_DEBUG(m_logger, "Written sucessfully");
            return hardware_interface::return_type::OK;
        }
        RCLCPP_WARN(m_logger, "Write error");
        return hardware_interface::return_type::ERROR;
	}

	std::string DiffBotSystemHardware::get_parameter_in_hardware_info_select(const hardware_interface::HardwareInfo & hardware_info, const std::string name)
    {
        std::unordered_map<std::string, std::string> parameters;
        std::unordered_map<std::string, std::string>::iterator position;

        parameters = hardware_info.hardware_parameters;

        position = parameters.find(name);

        if (position == parameters.end())
        {
            return "invalid";
        }

        RCLCPP_INFO(m_logger, "Parameter %s \t = %s", name.c_str(), position->second.c_str());

        return position->second;
    }

    DiffBotSystemHardware::CallbackReturn DiffBotSystemHardware::get_parameter_in_hardware_info(const hardware_interface::HardwareInfo & hardware_info)
    {
        std::string value;

        value = get_parameter_in_hardware_info_select(hardware_info, "left_wheel_name");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter left_wheel_name undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_left_wheel_name = value;

        value = get_parameter_in_hardware_info_select(hardware_info, "right_wheel_name");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter right_wheel_name undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_right_wheel_name = value;

        value = get_parameter_in_hardware_info_select(hardware_info, "motor_step_per_revolution");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter motor_step_per_revolution undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_motor_step_per_rev = std::stoi(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "encoder_step_per_rev");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter encoder_step_per_rev undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_encoder_step_per_rev = std::stoi(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "invert_motor_left");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter invert_motor_left undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_invert_motor_left = (value == "true");

        value = get_parameter_in_hardware_info_select(hardware_info, "invert_motor_right");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter invert_motor_right undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_invert_motor_right = (value == "true");

        value = get_parameter_in_hardware_info_select(hardware_info, "invert_encoder_left");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter invert_encoder_left undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_invert_encoder_left = (value == "true");

        value = get_parameter_in_hardware_info_select(hardware_info, "invert_encoder_right");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter invert_encoder_right undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_invert_encoder_right = (value == "true");

        value = get_parameter_in_hardware_info_select(hardware_info, "swap_motors");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter swap_motors undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_swap_motors = (value == "true");

        value = get_parameter_in_hardware_info_select(hardware_info, "swap_encoders");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter swap_encoders undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_swap_encoders = (value == "true");

        value = get_parameter_in_hardware_info_select(hardware_info, "motors_acceleration");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter motors_acceleration undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_motor_acceleration = std::stod(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "encoders_read_frequency");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter encoders_read_frequency undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_encoder_read_frequency = std::stod(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "encoder_wheel_diameter");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter encoder_wheel_diameter undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_encoder_wheel_diameter = std::stod(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "encoder_wheel_separation");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter encoder_wheel_separation undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_encoder_wheel_separation = std::stod(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "motor_wheel_diameter");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter motor_wheel_diameter undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_motor_wheel_diameter = std::stod(value);

        value = get_parameter_in_hardware_info_select(hardware_info, "motor_wheel_separation");
        if (value == "invalid")
        {
            RCLCPP_FATAL(m_logger, "parameter motor_wheel_separation undefined");
            return DiffBotSystemHardware::CallbackReturn::ERROR;
        }
        m_param_motor_wheel_separation = std::stod(value);

        

        return DiffBotSystemHardware::CallbackReturn::SUCCESS;
    }

	void DiffBotSystemHardware::callback_on_receive_motors_state(
        hw_support_interfaces_pkg::msg::MotorsState motors_state
    )
    {
        RCLCPP_DEBUG(m_logger, "receive motors state");
        m_real_enabled = motors_state.left_enable && motors_state.right_enable;
        m_motor_position_left = motor_compute_position(motors_state.left_position);
        m_motor_position_right = motor_compute_position(motors_state.right_position);
        m_motor_speed_left = motor_compute_position(motors_state.left_speed);
        m_motor_speed_right = motor_compute_position(motors_state.right_speed);
    }

    void DiffBotSystemHardware::callback_on_receive_encoders_state(
        hw_support_interfaces_pkg::msg::EncodersPosition encoders_state
    )
    {
        double invert_left, invert_right;


        invert_left = 1;
        if(m_param_invert_encoder_left) invert_left = -1;

        invert_right = 1;
        if(m_param_invert_encoder_right) invert_right = -1;


        if(m_param_swap_encoders)
        {
            m_encoder_position_left = encoder_compute_position(encoders_state.left_continuous) * invert_left;
            m_encoder_speed_left = encoder_compute_speed(encoders_state.left_delta) * invert_left;
            m_encoder_position_right = encoder_compute_position(encoders_state.right_continuous) * invert_right;
            m_encoder_speed_right = encoder_compute_speed(encoders_state.right_delta) * invert_right;
        }
        else
        {
            m_encoder_position_right = encoder_compute_position(encoders_state.left_continuous) * invert_right;
            m_encoder_speed_right = encoder_compute_speed(encoders_state.left_delta) * invert_right;
            m_encoder_position_left = encoder_compute_position(encoders_state.right_continuous) * invert_left;
            m_encoder_speed_left = encoder_compute_speed(encoders_state.right_delta) * invert_left;
        }


        RCLCPP_DEBUG(m_logger, "encoder positions received %.03f %.03f %d %d %f", m_encoder_position_left, m_encoder_position_right, encoders_state.left_raw, encoders_state.right_raw, ((double)(encoders_state.left_raw))*6.28318530718/4096.0);
    }


    double DiffBotSystemHardware::motor_compute_position(int32_t raw)
    {
        return ((double)raw) * 2 * M_PIf64 / ((double)m_param_motor_step_per_rev);
    }

    double DiffBotSystemHardware::motor_compute_speed(int32_t raw)
    {
        return ((double)raw) * 2 * M_PIf64 / ((double)m_param_motor_step_per_rev);
    }

    int32_t DiffBotSystemHardware::motor_encode_speed(double speed)
    {
        if (speed > 2 * M_PIf64) speed = 2 * M_PIf64;
        if (speed < -2 * M_PIf64) speed = -2 * M_PIf64;

        return (int32_t)(speed * ((double)m_param_motor_step_per_rev) / 2 * M_PIf64);
    }

    uint32_t DiffBotSystemHardware::motor_encode_acceleration(double acc)
    {
        return (int32_t)(acc);// * (double)m_param_motor_step_per_rev / 2 * M_PIf64);
    }

    double DiffBotSystemHardware::encoder_compute_position(int32_t raw)
    {
        return ((((double)(raw)) * 2 / ((double)m_param_encoder_step_per_rev)) - 1) * M_PIf64; 
    }

    double DiffBotSystemHardware::encoder_compute_speed(int32_t raw)
    {
        return (((double)(raw)) * 2 / ((double)m_param_encoder_step_per_rev) * m_param_encoder_read_frequency - 1) * M_PIf64;
    }

	bool DiffBotSystemHardware::send_motor_parameters(void)
    {
        RCLCPP_DEBUG(m_logger, "enter sending motors parameter");

        auto client = m_node->create_client<hw_support_interfaces_pkg::srv::MotorsSettings>("motors_settings");

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

        if(m_param_swap_motors)
        {
            request->left_set_speed = motor_encode_speed(m_motor_speed_right);
            request->right_set_speed = motor_encode_speed(m_motor_speed_left);
        }
        else
        {
            request->left_set_speed = motor_encode_speed(m_motor_speed_left);
            request->right_set_speed = motor_encode_speed(m_motor_speed_right);
        }

        RCLCPP_DEBUG(m_logger, "request creation");

        auto future = client->async_send_request(request);

        return true;
    }


}  // namespace robot_pkg

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_pkg::DiffBotSystemHardware, hardware_interface::SystemInterface)
