#ifndef DISTRIBUTION_BOARD_HPP__
#define DISTRIBUTION_BOARD_HPP__

#include "GenericBoard.hpp"
#include "DistributionBoardProtocol.hpp"

#include "hw_support_interfaces_pkg/msg/distribution_telemetry.hpp"
#include "hw_support_interfaces_pkg/msg/encoders_position.hpp"
#include "hw_support_interfaces_pkg/srv/relay_change_state.hpp"
#include "hw_support_interfaces_pkg/srv/get_encoder_position.hpp"

class DistributionBoardNode : public GenericBoardNode
{
public :
    DistributionBoardNode();
    ~DistributionBoardNode();

private :
    distribution_board::Safety_StatusVector_t safety_union;
    std::vector<uint32_t> raw_cells_voltages;
    std::vector<uint32_t> raw_voltages;
    std::vector<uint32_t> raw_currents;
    std::vector<double> cells_voltages;
    std::vector<double> voltages;
    std::vector<double> currents;
    uint32_t left_encoder, right_encoder, last_left, last_right;
    int64_t left_continuous, right_continuous;
    int32_t left_delta, right_delta;

    std::vector<double> cells_scales;
    std::vector<double> voltages_scales;
    std::vector<double> current_scales;
    std::vector<double> cells_offsets;
    std::vector<double> voltages_offsets;
    std::vector<double> current_offsets;
    uint32_t encoder_ppr;

    rclcpp::Publisher<hw_support_interfaces_pkg::msg::DistributionTelemetry>::SharedPtr telemetry_publisher_;
    rclcpp::Publisher<hw_support_interfaces_pkg::msg::EncodersPosition>::SharedPtr encoders_publisher_;
    
    rclcpp::Service<hw_support_interfaces_pkg::srv::RelayChangeState>::SharedPtr relay_change_state_service_;
    rclcpp::Service<hw_support_interfaces_pkg::srv::GetEncoderPosition>::SharedPtr get_encoder_position_service_;

    void on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data);

    void decode_status(std::vector<uint8_t> data);
    void decode_epo_state(std::vector<uint8_t> data);
    void decode_cells(std::vector<uint8_t> data);
    void decode_voltages(std::vector<uint8_t> data);
    void decode_currents(std::vector<uint8_t> data);
    void decode_encoders(std::vector<uint8_t> data);

    void encoders_compute(uint32_t current, uint32_t* last, int32_t* delta, int64_t* continuous);

    void telemetry_publish(void);
    void encoders_publish(void);

    void callback_relay_change_state(
        const hw_support_interfaces_pkg::srv::RelayChangeState_Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::RelayChangeState_Response::SharedPtr response
    );

    void callback_get_encoder_position(
        const hw_support_interfaces_pkg::srv::GetEncoderPosition_Request::SharedPtr request,
        const hw_support_interfaces_pkg::srv::GetEncoderPosition_Response::SharedPtr response
    );

    bool change_relay_state(bool state);

};

#endif // DISTRIBUTION_BOARD_HPP__