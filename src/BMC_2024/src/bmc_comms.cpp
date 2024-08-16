#include "BMC_2024/bmc_comms.hpp"
#include "rclcpp/rclcpp.hpp"

void BMCComms::open(const char* socket_interface, uint32_t can_id)
{
    RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), "open can interface");
    // store can_id for other calls
    can_id_ = can_id;
    // setup socket interface
    strcpy(ifr.ifr_name, socket_interface);
    ioctl(natsock, SIOGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    // setup can filter
    rx_filter.can_id = can_id;
    rx_filter.can_mask = CAN_SFF_MASK;
    setsockopt(natsock, SOL_CAN_RAW, CAN_RAW_FILTER, &rx_filter, sizeof(rx_filter));
    // enable can FD
    uint32_t enable_canfd = 1;
    setsockopt(natsock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

    // bind the socket
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
        RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), "Error binding CAN Socket");
    }
}

void BMCComms::enable()
{
    SendParam(BMC_CMD_ENABLE, 0);
    // start the listener thread
    listener_ = std::thread(&BMCComms::read_socket, this);
}

void BMCComms::disable()
{
    SendParam(BMC_CMD_DISABLE, 0);
}

void BMCComms::setVelocity(double velocity)
{
    SendParam(BMC_CMD_VELOCITY, (float)velocity);
}

void BMCComms::setPIDs(float P, float I, float D)
{
    SendParam(BMC_CMD_P_PID, P);
    SendParam(BMC_CMD_I_PID, I);
    SendParam(BMC_CMD_D_PID, D);
}

void BMCComms::update(Motor *motor)
{
    // update motor values
    motor->velocity = velocity_meas * (double)motor->mult;
    motor->position = angle_meas * (double)motor->mult;
    motor->current = current_meas;
}

void BMCComms::close()
{
    printf("\nEnd of Listener Thread.\n");
    // clean up the listener thread
    listener_.join();
}

void BMCComms::SendParam(uint8_t cmd_id, float param)
{
    RCLCPP_INFO(rclcpp::get_logger("SendParams"), "can_id: %.3d", (can_id_));
    RCLCPP_INFO(rclcpp::get_logger("SendParams"), "param: %.3f", (param));
    //RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), "send params");
    // set frame id to id of motor controller as rx
    tx_frame.can_id = (can_id_ << 8) | CAN_EFF_FLAG;
    //tx_frame.can_id = (183 << 8) | CAN_EFF_FLAG;
    // set the frame flags
    tx_frame.flags = tx_frame.flags & ~CANFD_BRS;
    // set byte length of message
    tx_frame.len = 0x5;
    // setup data to be sent
    memcpy(tx_frame.data, &cmd_id, 1);
    memcpy(tx_frame.data + 1, &param, 4);

    if(write(natsock, &tx_frame, sizeof(tx_frame)) < 0)
    {
        //std::string tx_send = std::to_string(tx_bytes);
        std::string err = std::to_string(errno);
        RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), err.c_str());
    }
}

void BMCComms::read_socket()
{//RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), "size=: %.3ld", (read(natsock, &rx_frame, sizeof(rx_frame))));
    while (rclcpp::ok())
    {
        // keep reading while a whole frame is available
        
        //while(read(natsock, &rx_frame, sizeof(rx_frame)) >= sizeof(rx_frame))
        while(read(natsock, &rx_frame, sizeof(rx_frame)) >= static_cast<ssize_t>(sizeof(rx_frame)))
        { //RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), "reading socket");
            // check cmd id
            switch(rx_frame.data[0])
            {
                case BMC_CMD_STATES: {
                    // check length?
                    // temporary storage variables
                    float velocity;
                    float angle;
                    float current;
                    // pointers to storage variables for data unpacking
                    uint8_t* ptrVelocity = (uint8_t *)&velocity;
                    uint8_t* ptrAngle = (uint8_t *)&angle;
                    uint8_t* ptrCurrent = (uint8_t *)&current;
                    // unpack states data
                    // starts with float(4 byte) of measured velocity
                    ptrVelocity[0] = rx_frame.data[4];
                    ptrVelocity[1] = rx_frame.data[5];
                    ptrVelocity[2] = rx_frame.data[6];
                    ptrVelocity[3] = rx_frame.data[7];
                    // continues with float(4 byte) of current angle estimate
                    ptrAngle[0] = rx_frame.data[8];
                    ptrAngle[1] = rx_frame.data[9];
                    ptrAngle[2] = rx_frame.data[10];
                    ptrAngle[3] = rx_frame.data[11];
                    // finish with float (4 byte) of measured DC current
                    ptrCurrent[0] = rx_frame.data[12];
                    ptrCurrent[1] = rx_frame.data[13];
                    ptrCurrent[2] = rx_frame.data[14];
                    ptrCurrent[3] = rx_frame.data[15];
                    // copy temp variables
                    velocity_meas = velocity;
                    angle_meas = angle;
                    current_meas = current;
                    RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), "sending can data vel=");
                    RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), std::to_string(velocity).c_str());
                    //RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), std::to_string(angle).c_str());
                    //RCLCPP_INFO(rclcpp::get_logger("RowbotBMCHardware"), std::to_string(current).c_str());
                }break;

                default: {

                }break;
            }
        }
    }
}