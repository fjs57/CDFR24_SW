#include "DistributionBoard.hpp"

#define DISTRIBUTION_BOARD_CELLS_COUNT 3
#define DISTRIBUTION_BOARD_BUS_COUNT 4

DistributionBoardNode::DistributionBoardNode() : GenericBoardNode("Distribution_Board"),
    raw_cells_voltages(DISTRIBUTION_BOARD_CELLS_COUNT),
    raw_voltages(DISTRIBUTION_BOARD_BUS_COUNT),
    raw_currents(DISTRIBUTION_BOARD_BUS_COUNT),
    cells_voltages(DISTRIBUTION_BOARD_CELLS_COUNT),
    voltages(DISTRIBUTION_BOARD_BUS_COUNT),
    currents(DISTRIBUTION_BOARD_BUS_COUNT)
{
    this->declare_parameter("cells_scales",     rclcpp::PARAMETER_DOUBLE_ARRAY );
    this->declare_parameter("voltages_scales",  rclcpp::PARAMETER_DOUBLE_ARRAY );
    this->declare_parameter("current_scales",   rclcpp::PARAMETER_DOUBLE_ARRAY );
    this->declare_parameter("cells_offsets",    rclcpp::PARAMETER_DOUBLE_ARRAY );
    this->declare_parameter("voltages_offsets", rclcpp::PARAMETER_DOUBLE_ARRAY );
    this->declare_parameter("current_offsets",  rclcpp::PARAMETER_DOUBLE_ARRAY );
    this->declare_parameter("encoder_ppr",      rclcpp::PARAMETER_INTEGER     );

    cells_scales     = this->get_parameter("cells_scales"    ).as_double_array();
    voltages_scales  = this->get_parameter("voltages_scales" ).as_double_array();
    current_scales   = this->get_parameter("current_scales"  ).as_double_array();
    cells_offsets    = this->get_parameter("cells_offsets"   ).as_double_array();
    voltages_offsets = this->get_parameter("voltages_offsets").as_double_array();
    current_offsets  = this->get_parameter("current_offsets" ).as_double_array();
    encoder_ppr      = this->get_parameter("encoder_ppr"     ).as_int();
}

DistributionBoardNode::~DistributionBoardNode()
{

}

void DistributionBoardNode::on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data)
{
    RCLCPP_DEBUG(this->get_logger(), "Message received, service=%02d", service);

    if ( service >= distribution_board::services_t::MAX )
    {
        RCLCPP_WARN(this->get_logger(), "unknown service, service=%02d", service);
        return;
    }

    switch ( (distribution_board::services_t)service )
    {
    case distribution_board::services_t::ID :
        RCLCPP_DEBUG(this->get_logger(), "Heartbeat received");       
        break;

    case distribution_board::services_t::STATUS :
        if ( length != DISTRIBUTION_PROTOCOL_RX_SERVICE_STATUS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length STATUS");
        }
        decode_status(data);
        break;

    case distribution_board::services_t::EPO_STATE :
        if (length != DISTRIBUTION_PROTOCOL_RX_SERVICE_EPO_STATE_LENGTH)
        {
            RCLCPP_WARN(this->get_logger(), "Bad length EPO_STATE");
            break;
        }
        decode_epo_state(data);
        break;

    case distribution_board::services_t::CELLS_VOLTAGE :
        if (length != DISTRIBUTION_PROTOCOL_RX_SERVICE_CELLS_VOLTAGE_LENGTH)
        {
            RCLCPP_WARN(this->get_logger(), "Bad length CELLS_VOLTAGE");
            break;
        }
        decode_cells(data);
        break;

    case distribution_board::services_t::VOLTAGES :
        if (length != DISTRIBUTION_PROTOCOL_RX_SERVICE_VOLTAGES_LENGTH)
        {
            RCLCPP_WARN(this->get_logger(), "Bad length VOLTAGES");
            break;
        }
        decode_voltages(data);
        break;

    case distribution_board::services_t::CURRENTS :
        if (length != DISTRIBUTION_PROTOCOL_RX_SERVICE_CURRENTS_LENGTH)
        {
            RCLCPP_WARN(this->get_logger(), "Bad length CURRENTS");
            break;
        }
        decode_currents(data);
        break;

    case distribution_board::services_t::ENCODERS :
        if (length != DISTRIBUTION_PROTOCOL_RX_SERVICE_ENCODERS_LENGTH)
        {
            RCLCPP_WARN(this->get_logger(), "Bad length ENCODERS");
            break;
        }
        decode_encoders(data);
        break;

    

    default:
        RCLCPP_WARN(this->get_logger(), "unhandled service, service=%02d", service);
        break;
    }
}

void DistributionBoardNode::decode_status(std::vector<uint8_t> data)
{
    uint64_t tmp;
    uint8_t it;

    tmp = 0;
    for(it=0;it<8;it++)
    {
        tmp |= (uint64_t)(data[it]) << (it*8);
    }
    set_status_vector(tmp);

    safety_union.vector = tmp;

    RCLCPP_DEBUG(
        this->get_logger(),
        "Status vector %016lx: cells=[%d,%d,%d], volts=[%d,%d,%d,%d], current=[%d,%d,%d,%d], encoders=[%d,%d], relay {state=%d, forcing=%d}",
        safety_union.vector,
        safety_union.fields.cell_1,
        safety_union.fields.cell_2,
        safety_union.fields.cell_3,
        safety_union.fields.volt_general,
        safety_union.fields.volt_logic,
        safety_union.fields.volt_actuators,
        safety_union.fields.volt_motors,
        safety_union.fields.curr_general,
        safety_union.fields.curr_logic,
        safety_union.fields.curr_actuators,
        safety_union.fields.curr_motors,
        safety_union.fields.encoder_r,
        safety_union.fields.encoder_l,
        safety_union.fields.relay,
        safety_union.fields.forcing
    );

    // TODO : complete the status decoding to extract data from the vector
}

void DistributionBoardNode::decode_epo_state(std::vector<uint8_t> data)
{
    safety_union.fields.relay = data[0] & 0x01;
    RCLCPP_DEBUG(this->get_logger(), "relay state : %d", safety_union.fields.relay);
}

void DistributionBoardNode::decode_cells(std::vector<uint8_t> data)
{
    uint8_t it;
    uint16_t tmp;
    for(it=0;it<DISTRIBUTION_BOARD_CELLS_COUNT;it++)
    {
        tmp = (uint16_t)(data[2*it]) & 0x00FF;
        tmp |= ((uint16_t)(data[2*it+1]) << 8)& 0xFF00;
        raw_cells_voltages[it] = (uint32_t)tmp;
        cells_voltages[it] = cells_scales[it] * (double)(raw_cells_voltages[it]) - cells_offsets[it];
    }
    RCLCPP_DEBUG(this->get_logger(), "raw cells voltages : [%d,%d,%d]", raw_cells_voltages[0], raw_cells_voltages[1], raw_cells_voltages[2]);
    RCLCPP_DEBUG(this->get_logger(), "cells voltages : [%f,%f,%f]", cells_voltages[0], cells_voltages[1], cells_voltages[2]);
}

void DistributionBoardNode::decode_voltages(std::vector<uint8_t> data)
{
    uint8_t it;
    uint16_t tmp;
    for(it=0;it<DISTRIBUTION_BOARD_BUS_COUNT;it++)
    {
        tmp = (uint16_t)(data[2*it]) & 0x00FF;
        tmp |= ((uint16_t)(data[2*it+1]) << 8)& 0xFF00;
        raw_voltages[it] = (uint32_t)tmp;
        voltages[it] = voltages_scales[it] * (double)(raw_voltages[it]) - voltages_offsets[it];
    }
    RCLCPP_DEBUG(this->get_logger(), "raw bus voltages : [%d,%d,%d,%d]", raw_voltages[0], raw_voltages[1], raw_voltages[2], raw_voltages[3]);
    RCLCPP_DEBUG(this->get_logger(), "bus voltages : [%f,%f,%f,%f]", voltages[0], voltages[1], voltages[2], voltages[3]);
}

void DistributionBoardNode::decode_currents(std::vector<uint8_t> data)
{
    uint8_t it;
    uint16_t tmp;
    for(it=0;it<DISTRIBUTION_BOARD_BUS_COUNT;it++)
    {
        tmp = (uint16_t)(data[2*it]) & 0x00FF;
        tmp |= ((uint16_t)(data[2*it+1]) << 8)& 0xFF00;
        raw_currents[it] = (uint32_t)tmp;
        currents[it] = current_scales[it] * (double)(raw_currents[it]) - current_offsets[it];
    }
    RCLCPP_DEBUG(this->get_logger(), "raw bus currents : [%d,%d,%d,%d]", raw_currents[0], raw_currents[1], raw_currents[2], raw_currents[3]);
    RCLCPP_DEBUG(this->get_logger(), "bus voltages : [%f,%f,%f,%f]", currents[0], currents[1], currents[2], currents[3]);
}

void DistributionBoardNode::decode_encoders(std::vector<uint8_t> data)
{
    uint8_t it;
    uint16_t tmp;
    uint32_t* encoder[] = {&left_encoder, &right_encoder};
    for(it=0;it<DISTRIBUTION_BOARD_BUS_COUNT;it++)
    {
        tmp = (uint16_t)(data[2*it]) & 0x00FF;
        tmp |= ((uint16_t)(data[2*it+1]) << 8)& 0xFF00;
        *encoder[it] = (uint32_t)tmp;
    }
    RCLCPP_DEBUG(this->get_logger(), "encoders : [%d,%d]", left_encoder, right_encoder);
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DistributionBoardNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}