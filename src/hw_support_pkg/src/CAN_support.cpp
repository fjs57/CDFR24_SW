#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>

#include "CAN_support.hpp"

/**
 * 
 * default constrictor
 * 
 * Initialize the interfaces and the 
 */
CanSupportNode::CanSupportNode() : Node("CAN_support")
{
    RCLCPP_INFO(this->get_logger(), "Node starting");

    // parameters
    this->declare_parameter("interface_file",       rclcpp::PARAMETER_STRING    );
    this->declare_parameter("baudrate",             rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("filter_addr",          rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("filter_mask",          rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("timeout_seconds",      rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("timeout_microseconds", rclcpp::PARAMETER_INTEGER   );
    this->declare_parameter("log_raw_frames",       rclcpp::PARAMETER_BOOL      );

    parameter_interface_file        =               this->get_parameter("interface_file")       .as_string()     ;
    parameter_baudrate              = (uint32_t)(   this->get_parameter("baudrate")             .as_int()       );
    parameter_filter_addr           = (uint32_t)(   this->get_parameter("filter_addr")          .as_int()       );
    parameter_filter_mask           = (uint32_t)(   this->get_parameter("filter_mask")          .as_int()       );
    parameter_timeout_seconds       = (uint32_t)(   this->get_parameter("timeout_seconds")      .as_int()       );
    parameter_timeout_microseconds  = (uint32_t)(   this->get_parameter("timeout_microseconds") .as_int()       );
    parameter_log_raw_frames        =               this->get_parameter("log_raw_frames")       .as_bool()       ;

    RCLCPP_INFO(this->get_logger(), "interface_file         : %s",      parameter_interface_file.c_str());
    RCLCPP_INFO(this->get_logger(), "baudrate               : %d",      parameter_baudrate              );
    RCLCPP_INFO(this->get_logger(), "filter_addr            : 0x%08x",  parameter_filter_addr           );
    RCLCPP_INFO(this->get_logger(), "filter_mask            : 0x%08x",  parameter_filter_mask           );
    RCLCPP_INFO(this->get_logger(), "timeout_seconds        : %d",      parameter_timeout_seconds       );
    RCLCPP_INFO(this->get_logger(), "timeout_microseconds   : %d",      parameter_timeout_microseconds  );
    RCLCPP_INFO(this->get_logger(), "log_raw_frames         : %d",      parameter_log_raw_frames        );

    // create the topic publisher for received can frames
    can_frame_receiver_publisher_ = this->create_publisher<hw_support_interfaces_pkg::msg::CanFrame>("received_frame", 10);

    // create the service server for frames to send
    can_frame_sender_service_ = this->create_service<hw_support_interfaces_pkg::srv::CanFrame>(
        "send_frame",
        std::bind(
            &CanSupportNode::callback_can_frame_sender_server,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    if(init_can())
    {
        start_receiving_thread();
    }
    else
    {
        
    }

    RCLCPP_INFO(this->get_logger(), "Node started");
}

CanSupportNode::~CanSupportNode()
{
    deinit_can();
    RCLCPP_INFO(this->get_logger(), "Node cleanly destroyed");
}

void CanSupportNode::callback_can_frame_sender_server(
    const hw_support_interfaces_pkg::srv::CanFrame::Request::SharedPtr request,
    const hw_support_interfaces_pkg::srv::CanFrame::Response::SharedPtr response
)
{
    struct can_frame frame;
    uint8_t data_iterator;

    char buf[64];
    uint16_t len_buf = 0;

    len_buf = sprintf(buf, "Received request to send id=%03x len=%d, data=[", request->can_frame.id, request->can_frame.length);

    frame.can_id = request->can_frame.id;
    frame.can_dlc = request->can_frame.length;

    for(data_iterator=0;data_iterator<request->can_frame.length;data_iterator++)
    {
        frame.data[data_iterator] = request->can_frame.data[data_iterator];
    
        if(data_iterator!=0) len_buf += sprintf(&(buf[len_buf]), ", ");
        len_buf += sprintf(&(buf[len_buf]), "%02x", frame.data[data_iterator]);
    }

    response->status = send(frame);

    len_buf += sprintf(&(buf[len_buf]), "] sent?=%d", response->status);

    RCLCPP_ERROR(this->get_logger(), "%s", buf );
}

bool CanSupportNode::init_can(void)
{
    int ret;
    char cmd[256];

    struct ifreq ifr;
    struct sockaddr_can addr;    
    struct can_filter rfilter[1];

    // makes sure that the I/F is well closed
    sprintf(cmd, "sudo ifconfig %s down", parameter_interface_file.c_str());
    system(cmd);

    // Sets the bitrate of the I/F
    sprintf(cmd, "sudo ip link set %s type can bitrate %d", parameter_interface_file.c_str(), parameter_baudrate);
    ret = system(cmd);
    if(ret!=0){
        RCLCPP_ERROR(this->get_logger(), "Cannot set the CAN Interface parameters");
        return false;
    } else {
        RCLCPP_DEBUG(this->get_logger(),"CAN Bus bitrate set to %d", parameter_baudrate);
    }

    // set the IF state as UP
    sprintf(cmd, "sudo ifconfig %s up", parameter_interface_file.c_str());
    ret = system(cmd);
    if(ret!=0){
        RCLCPP_ERROR(this->get_logger(),"Cannot turn on the CAN interface");
        return false;
    }

    // create the socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        RCLCPP_ERROR(this->get_logger(),"Cannot init CAN : socket PF_CAN failed");
        return false;
    }

    struct timeval tv;
    tv.tv_sec = parameter_timeout_seconds;
    tv.tv_usec = parameter_timeout_microseconds;

    if (
        (setsockopt(can_socket, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv))) < 0
    )
    {
        RCLCPP_WARN(this->get_logger(),"Cannot set CAN timeout");
    }

    // specify can0 device
    strcpy(ifr.ifr_name, parameter_interface_file.c_str());
    ret = ioctl(can_socket, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(),"Cannot init CAN : ioctl failed");
        return false;
    }

    // bind the socket to can0
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(can_socket, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(),"Cannot init CAN : bind failed");
        return false;
    }

    rfilter[0].can_id = parameter_filter_addr;
    rfilter[0].can_mask = parameter_filter_mask;

    ret = setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    if(ret!=0){
        RCLCPP_WARN(this->get_logger(),"Cannot init CAN : rx filter cannot be set");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),"CAB Bus filter set to id 0x%03x, mask 0x%03x", rfilter[0].can_id, rfilter[0].can_mask);
    }

    return true;
}

void CanSupportNode::deinit_can(void)
{
    char cmd[256];
    sprintf(cmd, "sudo ifconfig %s down", parameter_interface_file.c_str());
    system(cmd);
    RCLCPP_DEBUG(this->get_logger(),"CAN interface closed");
    stop_receiving_thread();
    send_mutex.unlock();
}

void CanSupportNode::start_receiving_thread(void)
{
    continue_receiving = true;

    receiver_thread = std::thread(
        [this]
        {
            this->receiving_thread_function();
        }
    );
}

void CanSupportNode::receiving_thread_function(void)
{
int nbytes;
    struct can_frame frame;
    char log_buf[128];
    int log_buf_len;

    while (continue_receiving)
    {

        for(size_t it=0;it<sizeof(frame);it++)
        {
            ((uint8_t*)(&frame))[it] = 0x00;
        }

        nbytes = read(can_socket, &frame, sizeof(frame));

        if(nbytes > 0)
        {
            if(parameter_log_raw_frames)
            {
                log_buf_len = sprintf(log_buf, "CAN Frame Received : id=%03x dlc=%d data=", frame.can_id, frame.can_dlc);
                for(int it=0; it<frame.can_dlc; it++)
                {
                    log_buf_len += sprintf(&(log_buf[log_buf_len]),  "%02x ", frame.data[it]);
                }
                RCLCPP_DEBUG(this->get_logger(), "%s", log_buf);
            }

            on_receive(&frame);
        }

    }
}

void CanSupportNode::stop_receiving_thread(void)
{
    continue_receiving = false;
    if(receiver_thread.joinable())
    {
        receiver_thread.join();
    }
}

void CanSupportNode::on_receive(struct can_frame* incomming)
{
    auto to_publish = hw_support_interfaces_pkg::msg::CanFrame();

    to_publish.id = incomming->can_id;
    to_publish.length = incomming->can_dlc;
    to_publish.data = std::vector<uint8_t>(incomming->data, incomming->data + incomming->can_dlc);

    can_frame_receiver_publisher_->publish(to_publish);
}

bool CanSupportNode::send(struct can_frame frame)
{
    uint8_t nbytes;

    send_mutex.lock();
    nbytes = write(can_socket, &frame, sizeof(frame));
    send_mutex.unlock();

    if(nbytes != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Send Error !");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Send success");
    return true;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CanSupportNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
