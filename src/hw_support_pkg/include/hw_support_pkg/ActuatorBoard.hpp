#ifndef ACTUATOR_BOARD_HPP_
#define ACTUATOR_BOARD_HPP_

#include "GenericBoard.hpp"

class ActuatorBoardNode : public GenericBoardNode
{
public :
    ActuatorBoardNode();
    ~ActuatorBoardNode();

private :
    void on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data);

};

#endif // ACTUATOR_BOARD_HPP_