/*! \file 
 *  \brief socket_can_node.cpp.
 *         This file defines the node implementing socket can in ROS
 *  By: Juan David Galvis
 *  email: juangalvis@kiwicampus.com
 */
#include <socket_can_ros/CANRos.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control_node");
    ros::Time::init();
    ros::NodeHandle n("~");
    ros::Rate r(20.0);
    CANRos *can_ros = new CANRos(&n); // Instance of CANRos class which opens socket and hence CAN communication

    ROS_INFO("CAN communication ready!");
    while (n.ok())
    {
        can_ros->ReadCANBus();
        ros::spinOnce();
        r.sleep();
    }
}