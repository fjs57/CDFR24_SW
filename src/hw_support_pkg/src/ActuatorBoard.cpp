#include "ActuatorBoard.hpp"

ActuatorBoardNode::ActuatorBoardNode() : GenericBoardNode("Actuator_Board")
{

}

ActuatorBoardNode::~ActuatorBoardNode()
{

}

void ActuatorBoardNode::on_frame_received(uint32_t service, uint32_t length, std::vector<uint8_t> data)
{
    (void)length; // UNUSED
    (void)data; // UNUSED

    RCLCPP_DEBUG(this->get_logger(), "Message received, service=%02d", service);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ActuatorBoardNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}