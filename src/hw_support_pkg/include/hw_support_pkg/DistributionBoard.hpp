#ifndef DISTRIBUTION_BOARD_HPP__
#define DISTRIBUTION_BOARD_HPP__

#include "GenericBoard.hpp"
#include "DistributionBoardProtocol.hpp"

#include "hw_support_interfaces_pkg/msg/distribution_telemetry.hpp"
#include "hw_support_interfaces_pkg/msg/encoders_position.hpp"

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
    uint32_t left_encoder, right_encoder;

    std::vector<double> cells_scales;
    std::vector<double> voltages_scales;
    std::vector<double> current_scales;
    std::vector<double> cells_offsets;
    std::vector<double> voltages_offsets;
    std::vector<double> current_offsets;
    uint32_t encoder_ppr;

    rclcpp::Publisher<hw_support_interfaces_pkg::msg::DistributionTelemetry>::SharedPtr telemetry_publisher_;
    rclcpp::Publisher<hw_support_interfaces_pkg::msg::EncodersPosition>::SharedPtr encoders_publisher_;
    
    void on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data);

    void decode_status(std::vector<uint8_t> data);
    void decode_epo_state(std::vector<uint8_t> data);
    void decode_cells(std::vector<uint8_t> data);
    void decode_voltages(std::vector<uint8_t> data);
    void decode_currents(std::vector<uint8_t> data);
    void decode_encoders(std::vector<uint8_t> data);

};

#endif // DISTRIBUTION_BOARD_HPP__