#ifndef DISTRIBUTION_BOARD_CONFIG_HPP__
#define DISTRIBUTION_BOARD_CONFIG_HPP__

/**
 * messages length per service and direction
 */

#define DISTRIBUTION_PROTOCOL_TX_SERVICE_ID_LENGTH 				0
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_STATUS_LENGTH 			0
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_ACK_ERRORS_LENGTH 		1
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_EPO_STATE_LENGTH			1
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_CELLS_VOLTAGE_LENGTH		0
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_VOLTAGES_LENGTH			0
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_CURRENTS_LENGTH			0
#define DISTRIBUTION_PROTOCOL_TX_SERVICE_ENCODERS_LENGTH			0

#define DISTRIBUTION_PROTOCOL_RX_SERVICE_ID_LENGTH 				    1
#define DISTRIBUTION_PROTOCOL_RX_SERVICE_STATUS_LENGTH 			    5
#define DISTRIBUTION_PROTOCOL_RX_SERVICE_EPO_STATE_LENGTH			1
#define DISTRIBUTION_PROTOCOL_RX_SERVICE_CELLS_VOLTAGE_LENGTH		6
#define DISTRIBUTION_PROTOCOL_RX_SERVICE_VOLTAGES_LENGTH			8
#define DISTRIBUTION_PROTOCOL_RX_SERVICE_CURRENTS_LENGTH			8
#define DISTRIBUTION_PROTOCOL_RX_SERVICE_ENCODERS_LENGTH			4

namespace distribution_board
{

    typedef enum 
    {
		ID = 0,
		STATUS,
		ACK_ERRORS,
		EPO_STATE,
		CELLS_VOLTAGE,
		VOLTAGES,
		CURRENTS,
		ENCODERS,
		MAX
    }   
    services_t;

    typedef union {
        struct _fields{
            // byte 0
            unsigned cell_1 : 4;
            unsigned cell_2 : 4;

            // byte 1
            unsigned cell_3 : 4;
            unsigned volt_general : 4;

            // byte 2
            unsigned volt_logic : 4;
            unsigned volt_actuators : 4;

            // byte 3
            unsigned volt_motors : 4;
            unsigned curr_general : 2;
            unsigned curr_logic : 2;

            // byte 4
            unsigned curr_actuators : 2;
            unsigned curr_motors : 2;
            unsigned encoder_r : 1;
            unsigned encoder_l : 1;
            unsigned relay : 1;
            unsigned forcing : 1;
        } fields;

        uint64_t vector;
        uint8_t bytes[5];
    } Safety_StatusVector_t;

};
#endif // DISTRIBUTION_BOARD_CONFIG_HPP__