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
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <socket_can_ros/PwmOut.h>
#include <socket_can_ros/Configuration.h>
#include <socket_can_ros/Motors.h>
#include <socket_can_ros/TestMotors.h>
#include <utils/console.h>
#include <utils/debugger.h>
#include <sensor_msgs/BatteryState.h>
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

#define WHEELS_BAUDRATE_DEFAULT 0x14
#define BATTERY_STATUS_BAUDRATE_DEFAULT 0x14
#define WHEEL_CONTROL_MODE_DEFAULT 0x00
#define OPERATION_MODE_DEFAULT 0x00
#define NUMBER_BATTERY_CELLS_DEFAULT 0x04
#define MOTORS_MODEL_DEFAULT 0x00

#define RIGHT_FRONT_WHEEL 0x01
#define RIGHT_REAR_WHEEL 0x02
#define LEFT_FRONT_WHEEL 0x04
#define LEFT_REAR_WHEEL 0x08


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
        std::vector<uint16_t> raw_pwm_out_;

        // Published msgs
        mavros_msgs::State motor_drive_status_;
        std_msgs::String can_driver_status_msg_;
        socket_can_ros::PwmOut pwm_out_msg_;
        socket_can_ros::Motors motors_status_;
        socket_can_ros::TestMotors test_motors_response_;
        sensor_msgs::BatteryState battery_msg_;


        // Subscribed to msgs
        socket_can_ros::Configuration chassis_configuration_;

        // Publishers
        ros::Publisher motor_drive_status_pub_;
        ros::Publisher pwm_out_pub_;
        ros::Publisher pwm_status_pub_;
        ros::Publisher motor_status_pub_;
        ros::Publisher test_motors_pub_;
        ros::Publisher battery_pub_; 

        //Subscribers
        ros::Subscriber actuator_control_sub_;
        ros::Subscriber chassis_config_sub_;
        ros::Subscriber motors_sleep_sub_;

        // Services
        ros::ServiceServer arming_service_;

        // Timers
        ros::Timer error_timer_;

        Debugger *dbg_;
        CANDriver *can_driver_;
        float wheel_max_rpm_;
        float motor_max_current_;
        int debug_level_;
        const char *interface_name_ = "vcan0";
        float radius_;
        float robot_length_;

        // Member functions
        void ActuatorControlCb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void ChassisConfigCb(const socket_can_ros::Configuration::ConstPtr &msg);
        void MotorsSleepCb(const std_msgs::Bool::ConstPtr &msg);
        void ErrorTimerCb(const ros::TimerEvent &event);

        uint8_t RadsToDigital(float control);
        bool Arm(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res);
        void SendMotorsCmd();
        void SendChassisConfiguration();
        void PublishChassisStatus(struct can_frame *frame);
};
#endif