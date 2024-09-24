#ifndef __bmc_comms_H__
#define __bmc_comms_H__

#include <string>
#include <linux/can/raw.h>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#define BMC_CAN_ID_BASE 0xB0

#define BMC_CMD_DISABLE 0xE0
#define BMC_CMD_ENABLE 0xE1
#define BMC_CMD_STATES 0xF0
#define BMC_CMD_VELOCITY 0xF1
#define BMC_CMD_ODOMETRY 0xF2
#define BMC_CMD_DC_CURRENT 0xF3
#define BMC_CMD_P_PID 0xA0
#define BMC_CMD_I_PID 0xA1
#define BMC_CMD_D_PID 0xA2

struct Motor
{
  std::string name = " ";
  uint32_t can_id = BMC_CAN_ID_BASE;
  float mult = 1;
  double cmd = 0;
  double velocity = 0;
  double position = 0;
  double current = 0;
};

class BMCComms
{
    public:
        // opens the socket can interface and sets up the filter
        void open(const char* socket_interface, uint32_t can_id);

        // enable bmc
        void enable();

        // disable bmc
        void disable();

        // Set velocity
        void setVelocity(double velocity);

        // Set PID values
        void setPIDs(float P, float I, float D);

        // read messages available
        void update(Motor *motor);

        // socket listener
        void read_socket();

        // closes the socket can interface
        void close();

    private:
        // listener thread for socket can
        std::thread listener_;
        // sending individual motor controller parameters over can
        void SendParam(uint8_t cmd_id, float param);

        struct sockaddr_can addr;
        struct canfd_frame tx_frame;
        struct canfd_frame rx_frame;
        struct ifreq ifr;
        struct can_filter rx_filter;

        // storing can ID for sending
        uint32_t can_id_;

        // internal variables for storing asynch state reads
        float velocity_meas;
        float angle_meas;
        float current_meas;

        // can socket inst
        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
};
#endif