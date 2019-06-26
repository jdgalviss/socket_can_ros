/*! \file 
 *  \brief CANRos.h.
 *         This library defines members and member functions for the CANnode
 *  By: Juan David Galvis
 *  email: juangalvis@kiwicampus.com
 */

#ifndef CAN_ROS_H_INCLUDED
#define CAN_ROS_H_INCLUDED
#include <socket_can_ros/socketCAN.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <pwm/PwmOut.h>
#include <utils/console.h>
#include <utils/debugger.h>
#include <vector>

#define CONFIGURATION_CMD_ID 0x01
#define ARM_CMD_ID 0x02
#define WHEELS_INFO_ID 0x03
#define WHEELS_CONTROL_RPM_ID 0x04
#define MOTORS_CONTROL_RAW_ID 0x05
#define MOTORS_CURRENT_ID 0x06
#define ERROR_CMD_ID 0x07
#define TEST_REPORT_CMD_ID 0x08
#define DEBUG_CMD_ID 0x09
#define STATUS_CMD_ID 0x0A

class CANRos{
    public:
        CANRos(ros::NodeHandle *nh);
        ~CANRos();
        void ReadCANBus();


    private:
	    ros::NodeHandle *n_;					 /**< Reference to the nodehandle so, the position controller can call services, get parameters, subscribe to and publish topics */
        std::vector<float> controls_;
        std::vector<float> offsets_;
        std::vector<float> inverts_;


        mavros_msgs::State motor_drive_status_;
        std_msgs::String can_driver_status_msg_;
        pwm::PwmOut pwm_out_msg_;
        std::vector<uint16_t> raw_pwm_out_;


        // Publishers
        ros::Publisher motor_drive_status_pub_;
        ros::Publisher pwm_out_pub_;
        ros::Publisher pwm_status_pub_;

        //Subscribers
        ros::Subscriber actuator_control_sub_;

        // Services
        ros::ServiceServer arming_service_;

        Debugger *dbg_;
        CANDriver *can_driver_;
        float wheel_max_rpm_;
        int debug_level_;
        const char *interface_name_ = "vcan0";
        float radius_;
        float robot_length_;

        // Member functions
        void ActuatorControlCb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        uint8_t RadsToDigital(float control);
        bool Arm(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res);
        void SendMotorsCmd();


};
#endif