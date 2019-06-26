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

#include <socket_can_ros/socketCAN.h>

#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "socket_can_node");
    ros::Time::init();
    ros::NodeHandle nh("~");
    int rate = 1;
    ros::Rate r(rate);
	const char *interface_name = "vcan0";
    CANDriver *can_driver = new CANDriver(interface_name); // speed controller instance: It includes all the functions that perform PID feedback control
	uint8_t data [2] = {0x11,0x22};
    while (nh.ok())
    {
		can_driver->CANWrite(0x123,2,data);
        ros::spinOnce();
        r.sleep();
    }
	
	return 0;
}