#ifndef ACTUATOR_BOARD_HPP__
#define ACTUATOR_BOARD_HPP__

#include "GenericBoard.hpp"

namespace actuator_board
{
    class StepperMotor
    {
    public:

        uint16_t acc;
        uint16_t f_max;
        uint16_t f_min;

        uint16_t target_frequency;
        int32_t target_position;

        uint16_t frequency;
        int32_t position;
        bool direction;
        bool enable;
        uint8_t mode;
    };

};

class ActuatorBoardNode : public GenericBoardNode
{
public :
    ActuatorBoardNode();
    ~ActuatorBoardNode();

private :

    std::vector<int16_t> servo_positions;
    bool servo_enabled;

    std::vector<actuator_board::StepperMotor> motors;
    
    float ADC0_value, ADC1_value, Servo_voltage;
    std::vector<uint8_t> gpio_states;
    
    void on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data);

    void decode_status(std::vector<uint8_t> data);
    void decode_servo_set(std::vector<uint8_t> data);
    void decode_MPP_target_frequency(std::vector<uint8_t> data);
    void decode_MPP_target_position(std::vector<uint8_t> data);
    void decode_MPP_state(std::vector<uint8_t> data);
    void decode_MPP_config(std::vector<uint8_t> data);
    void decode_gpio_state(std::vector<uint8_t> data);

};

#endif // ACTUATOR_BOARD_HPP__