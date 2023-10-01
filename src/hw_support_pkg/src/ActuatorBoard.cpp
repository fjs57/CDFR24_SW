#include "ActuatorBoard.hpp"
#include "ActuatorBoardProtocol.hpp"

#define ACTUATOR_BOARD_SERVO_COUNT 16
#define ACTUATOR_BOARD_MOTOR_COUNT 2
#define ACTUATOR_BOARD_GPIO_COUNT 4
#define ACTUATOR_BOARD_ADC_COUNT 3
#define ACTUATOR_BOARD_ADC_MAX_VALUE 0xFFFF
#define ACTUATOR_BOARD_ADC_SERVO_SCALE 36.3f
#define ACTUATOR_BOARD_ADC_IN_SCALE 3.3f

ActuatorBoardNode::ActuatorBoardNode() : GenericBoardNode("Actuator_Board"),
    servo_positions(ACTUATOR_BOARD_SERVO_COUNT),
    motors(ACTUATOR_BOARD_MOTOR_COUNT),
    gpio_states(ACTUATOR_BOARD_GPIO_COUNT)
{

}

ActuatorBoardNode::~ActuatorBoardNode()
{

}

void ActuatorBoardNode::on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data)
{
    RCLCPP_DEBUG(this->get_logger(), "Message received, service=%02d", service);

    if ( service >= actuator_board::services_t::MAX )
    {
        RCLCPP_WARN(this->get_logger(), "unknown service, service=%02d", service);
        return;
    }

    switch ( (actuator_board::services_t)service )
    {
    case actuator_board::services_t::ID :
        RCLCPP_DEBUG(this->get_logger(), "Heartbeat received");       
        break;

    case actuator_board::services_t::STATUS :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_STATUS_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_status(data);
        break;

    case actuator_board::services_t::SERVO_SET :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_SERVO_SET_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_servo_set(data);
        break;

    case actuator_board::services_t::MPP_TARGET_FREQUENCY :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_MPP_TARGET_FREQUENCY_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_MPP_target_frequency(data);
        break;

    case actuator_board::services_t::MPP_TARGET_POSITION :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_MPP_TARGET_POSITION_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_MPP_target_position(data);
        break;

    case actuator_board::services_t::MPP_STATE :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_MPP_STATE_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_MPP_state(data);
        break;

    case actuator_board::services_t::MPP_CONFIG :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_MPP_CONFIG_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_MPP_config(data);
        break;

    case actuator_board::services_t::GPIO_STATE :
        if ( length != ACTUATOR_BOARD_PROTOCOL_RX_SERVICE_GPIO_STATE_LENGTH )
        {
            RCLCPP_WARN(this->get_logger(), "Bad length");
        }
        decode_gpio_state(data);
        break;

    default:
        RCLCPP_WARN(this->get_logger(), "unhandled service, service=%02d", service);
        break;
    }
}

void ActuatorBoardNode::decode_status(std::vector<uint8_t> data)
{
    set_status_vector(data[0]);
    // TODO : complete the status decoding to extract data from the vector
}

void ActuatorBoardNode::decode_servo_set(std::vector<uint8_t> data)
{
    int16_t position;
    uint8_t id;
    
    id = data[0];
    if ( id >= ACTUATOR_BOARD_SERVO_COUNT )
    {
        RCLCPP_WARN(this->get_logger(), "servo id out of bound, id=%d", id);
        return;
    }

    servo_enabled = data[3] != 0x00;

    position = ( int16_t )( ( ( (uint16_t)(data[2]) << 8 ) & 0xFF00 ) | ( (uint16_t)(data[1]) & 0x00FF ) );

    servo_positions[id] = position;


    RCLCPP_DEBUG(this->get_logger(), "Message decoded : servo num %d at %d degrees", id, position);

}

void ActuatorBoardNode::decode_MPP_target_frequency(std::vector<uint8_t> data)
{
    uint16_t frequency;
    uint8_t id;
    
    id = data[0];
    if ( id >= ACTUATOR_BOARD_MOTOR_COUNT )
    {
        RCLCPP_WARN(this->get_logger(), "motor id out of bound, id=%d", id);
        return;
    }

    frequency = ( ( (uint16_t)(data[2]) << 8 ) & 0xFF00 ) | ( (uint16_t)(data[1]) & 0x00FF );

    motors[id].target_frequency = frequency;

    RCLCPP_DEBUG(this->get_logger(), "Message decoded : motor num %d frequency target %d Hz", id, frequency);

}

void ActuatorBoardNode::decode_MPP_target_position(std::vector<uint8_t> data)
{
    uint32_t position;
    uint8_t id;
    
    id = data[0];
    if ( id >= ACTUATOR_BOARD_MOTOR_COUNT )
    {
        RCLCPP_WARN(this->get_logger(), "motor id out of bound, id=%d", id);
        return;
    }

    position  = ( (uint32_t)(data[1]) <<  0 ) & 0x000000FF;
    position |= ( (uint32_t)(data[2]) <<  8 ) & 0x0000FF00;
    position |= ( (uint32_t)(data[3]) << 16 ) & 0x00FF0000;
    position |= ( (uint32_t)(data[4]) << 24 ) & 0xFF000000;

    motors[id].target_position = (int32_t)position;

    RCLCPP_DEBUG(this->get_logger(), "Message decoded : motor num %d position target %d steps", id, position);

}

void ActuatorBoardNode::decode_MPP_state(std::vector<uint8_t> data)
{
    uint32_t position;
    uint16_t frequency;
    uint8_t id;
    uint8_t bit_vector;
    
    id = data[0];
    if ( id >= ACTUATOR_BOARD_MOTOR_COUNT )
    {
        RCLCPP_WARN(this->get_logger(), "motor id out of bound, id=%d", id);
        return;
    }

    bit_vector = data[1];

    frequency  = ( (uint16_t)(data[2]) << 0 ) & 0x00FF;
    frequency |= ( (uint16_t)(data[3]) << 8 ) & 0xFF00;

    position  = ( (uint32_t)(data[4]) <<  0 ) & 0x000000FF;
    position |= ( (uint32_t)(data[5]) <<  8 ) & 0x0000FF00;
    position |= ( (uint32_t)(data[6]) << 16 ) & 0x00FF0000;
    position |= ( (uint32_t)(data[7]) << 24 ) & 0xFF000000;

    motors[id].position =   (int32_t)position;
    motors[id].frequency =  frequency;
    motors[id].direction =  bit_vector          & 0x01;
    motors[id].enable =     (bit_vector >> 1)   & 0x01;
    motors[id].mode =       (bit_vector >> 2)   & 0x3F;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor num %d pos:%d freq:%d Hz dir:%d en:%d mode:%d",
        id, position, frequency, motors[id].direction,  motors[id].enable, motors[id].mode
    );

}

void ActuatorBoardNode::decode_MPP_config(std::vector<uint8_t> data)
{
    uint16_t acc, f_min, f_max;
    uint8_t id;

    id = data[0];
    if ( id >= ACTUATOR_BOARD_MOTOR_COUNT )
    {
        RCLCPP_WARN(this->get_logger(), "motor id out of bound, id=%d", id);
        return;
    }

    acc     = ( (uint16_t)(data[1]) << 0 ) & 0x00FF;
    acc    |= ( (uint16_t)(data[2]) << 8 ) & 0xFF00;

    f_min   = ( (uint16_t)(data[3]) << 0 ) & 0x00FF;
    f_min  |= ( (uint16_t)(data[4]) << 8 ) & 0xFF00;

    f_max   = ( (uint16_t)(data[5]) << 0 ) & 0x00FF;
    f_max  |= ( (uint16_t)(data[6]) << 8 ) & 0xFF00;

    motors[id].acc      = acc;
    motors[id].f_min    = f_min;
    motors[id].f_max    = f_max;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : motor num %d config acc:%d f_min:%d f_max:%d",
        id, acc, f_min, f_max
    );

}

void ActuatorBoardNode::decode_gpio_state(std::vector<uint8_t> data)
{
    uint8_t gpio_iterator;
    uint8_t adc_iterator;

    uint16_t adc_tmp_value;

    float * adc_values_ptr[] = {&ADC0_value, &ADC1_value, &Servo_voltage};
    float adc_scales[] = {ACTUATOR_BOARD_ADC_IN_SCALE, ACTUATOR_BOARD_ADC_IN_SCALE, ACTUATOR_BOARD_ADC_SERVO_SCALE};

    for(adc_iterator=0; adc_iterator<ACTUATOR_BOARD_ADC_COUNT; adc_iterator++)
    {
        adc_tmp_value  = ( (uint16_t)(data[adc_iterator * 2    ]) << 0 ) & 0x00FF;
        adc_tmp_value |= ( (uint16_t)(data[adc_iterator * 2 + 1]) << 8 ) & 0xFF00;

        *adc_values_ptr[adc_iterator] = ((float)(adc_tmp_value)) / ((float)(ACTUATOR_BOARD_ADC_MAX_VALUE)) * 100.0f * adc_scales[adc_iterator];
    }

    for(gpio_iterator=0; gpio_iterator<ACTUATOR_BOARD_GPIO_COUNT; gpio_iterator++)
    {
        gpio_states[gpio_iterator] = (data[6] >> gpio_iterator) & 0x01;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "Message decoded : GPIO: [%d,%d,%d,%d] ADC0:%.2fV, ADC1:%.2fV, Servo:%.2fV",
        gpio_states[0], gpio_states[1], gpio_states[2], gpio_states[3],
        ADC0_value, ADC1_value, Servo_voltage
    );

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ActuatorBoardNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}