/*! \file 
 *  \brief socket_can.cpp
 *         This library defines functions to communicate via a CAN interface with external can devices
 *  By: Juan David Galvis
 *  email: juangalvis@kiwicampus.com
 */
#include <socket_can_ros/socketCAN.h>

CANDriver::CANDriver(const char *interface_name)
{

    // Opening socket
    const char *ifname_ = interface_name;
    if ((sckt_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
    }

    strcpy(ifr_.ifr_name, ifname_);
    ioctl(sckt_, SIOCGIFINDEX, &ifr_);

    addr_.can_family = AF_CAN;

    addr_.can_ifindex = ifr_.ifr_ifindex;

    printf("%s at index %d\n", ifname_, ifr_.ifr_ifindex);

    if (bind(sckt_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
    {
        perror("Error in socket bind");
    }

    //Define thread in order to listen to socket
    listen_thread_ = std::thread(&CANDriver::ListenSocket, this);
}

void CANDriver::ListenSocket()
{
    struct can_frame rxmsg;
    bool read_can_port = true;
    printf("ready to read can interface\n");
    while (read_can_port)
    {
        read(sckt_, &rxmsg, sizeof(rxmsg));
        printf("message received\n");
    }
}

int CANDriver::CANWrite(int can_id, int can_data_length, uint8_t *data)
{
    frame_.can_id = can_id;
    frame_.can_dlc = can_data_length;
    for (int i = 0; i < can_data_length; ++i)
    {
        frame_.data[i] = data[i];
    }
    nbytes_ = write(sckt_, &frame_, sizeof(struct can_frame));
    printf("Wrote %d bytes\n", nbytes_);
    return 0;
}