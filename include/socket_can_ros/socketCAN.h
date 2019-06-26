/*! \file 
 *  \brief socketCAN.h.
 *         This library defines functions to communicate via a CAN interface with external can devices
 *  By: Juan David Galvis
 *  email: juangalvis@kiwicampus.com
 */

#ifndef SOCKET_CAN_H_INCLUDED
#define SOCKET_CAN_H_INCLUDED
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string> 
#include <thread>


class CANDriver{
    public:
        CANDriver(const char *interface_name);
        ~CANDriver();

        int CANWrite(int can_id, int can_data_length, uint8_t *data);
        void ListenSocket();

    private:
        int sckt_;
        int nbytes_;
        struct sockaddr_can addr_;
        struct can_frame frame_;
        struct ifreq ifr_;
        const char *ifname_;

        std::thread listen_thread_;  /**< Thread to listen CAN port */

};




#endif