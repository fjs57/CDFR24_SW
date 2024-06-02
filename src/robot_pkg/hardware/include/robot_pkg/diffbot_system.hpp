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

#ifndef ROBOT_PKG__DIFFBOT_SYSTEM_HPP_
#define ROBOT_PKG__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

// #include <single_threaded_executor.hpp>

#include "visibility_control.h"

#include "hw_support_interfaces_pkg/msg/motors_state.hpp"
#include "hw_support_interfaces_pkg/msg/encoders_position.hpp"

#include "hw_support_interfaces_pkg/srv/motors_settings.hpp"
#include "hw_support_interfaces_pkg/srv/get_encoder_position.hpp"

namespace robot_pkg
{
	class DiffBotSystemHardware : public hardware_interface::SystemInterface
	{
		public:

			DiffBotSystemHardware();

			RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)//; (macro doesn't need ; to end block)

			ROBOT_PKG_PUBLIC
			hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

			ROBOT_PKG_PUBLIC
			std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

			ROBOT_PKG_PUBLIC
			std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

			ROBOT_PKG_PUBLIC
			hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

			ROBOT_PKG_PUBLIC
			hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

			ROBOT_PKG_PUBLIC
			hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

			ROBOT_PKG_PUBLIC
			hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

		private:
			rclcpp::executors::SingleThreadedExecutor m_executor;
			rclcpp::Node::SharedPtr m_node;
			rclcpp::Logger m_logger;

			rclcpp::Subscription<hw_support_interfaces_pkg::msg::MotorsState>::SharedPtr m_motors_state_subscriber;
    		rclcpp::Subscription<hw_support_interfaces_pkg::msg::EncodersPosition>::SharedPtr m_encoders_state_subscriber;

			std::string m_param_left_wheel_name, m_param_right_wheel_name;
			uint32_t m_param_motor_step_per_rev;
			uint32_t m_param_encoder_step_per_rev;
			bool m_param_invert_motor_left, m_param_invert_motor_right;
			bool m_param_invert_encoder_left, m_param_invert_encoder_right;
			bool m_param_swap_motors, m_param_swap_encoders;
			double m_param_motor_acceleration;
			double m_param_encoder_read_frequency;

			double m_param_encoder_wheel_diameter;
			double m_param_encoder_wheel_separation;
			double m_param_motor_wheel_diameter;
			double m_param_motor_wheel_separation;

			bool m_set_enabled, m_real_enabled;
			double m_set_speed_left, m_set_speed_right;
			double m_motor_speed_left, m_motor_speed_right;
			double m_cmd_motor_speed_left, m_cmd_motor_speed_right;
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

}  // namespace robot_pkg

#endif  // ROBOT_PKG__DIFFBOT_SYSTEM_HPP_
