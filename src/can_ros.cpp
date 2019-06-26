/*! \file 
 *  \brief can_ros.cpp.
 *         This file defines members and member functions for the CANnode in ROS
 *  By: Juan David Galvis
 *  email: juangalvis@kiwicampus.com
 */

#include <socket_can_ros/CANRos.h>

CANRos::CANRos(ros::NodeHandle *nh)
{
    n_ = nh;
    // ============ Publishers ===========
    // Motors control
    motor_drive_status_pub_ = n_->advertise<mavros_msgs::State>("/kiwibot/driveController/status", 10);
    pwm_out_pub_ = n_->advertise<pwm::PwmOut>("/kiwibot/pwm/out", 10);
    pwm_status_pub_ = n_->advertise<std_msgs::String>("/kiwibot/pwm/status", 10);

    // ============ Subscribers ===========
    // Motors control
    actuator_control_sub_ = n_->subscribe<geometry_msgs::TwistStamped>("/kiwibot/actuator_control", 1000, &CANRos::ActuatorControlCb, this);

    // ============ Services ===========
    // Motors control
    arming_service_ = n_->advertiseService("/mavros/cmd/arming", &CANRos::Arm, this);

    //Initialize Debug level
    dbg_ = new Debugger;
    debug_level_ = 0;
    n_->param<int>("debug", debug_level_, 0);
    dbg_->setDebugLevel(debug_level_);

    //Initialize pwm status and pwm out
    can_driver_status_msg_.data = "OK";
    pwm_out_msg_.info = pwm_out_msg_.DEFAULT;
    motor_drive_status_.mode = "JETHAWK";
    motor_drive_status_.armed = false;
    motor_drive_status_.connected = true;

    // Instance of CAN Driver - open socket
    try
    {
        can_driver_ = new CANDriver(interface_name_);
    }
    catch (const std::system_error &error)
    {
        char buff[100];
        snprintf(buff, sizeof(buff), "ERROR reading can device: %s", error.what());
        std::string errorStr(buff);
        dbg_->print(dbg_->DEBUG_LEVEL_1, "%s", errorStr.c_str());
        can_driver_status_msg_.data = errorStr;
        motor_drive_status_.connected = false;
    }

    // Initialize variables relevant for wheel speed calculation
    radius_ = getEnv("WHEEL_RADIUS", 0.074f);
    robot_length_ = getEnv("ROBOT_LENGTH", 0.4f);
    wheel_max_rpm_ = getEnv("WHEEL_MAX_RPM", 165.0f);
    controls_ = {0.0f, 0.0f, 0.0f};
    offsets_ = {0.0f, 0.0f, 0.0f};
    inverts_ = {1.0f, 1.0f, 1.0f};
    raw_pwm_out_.resize(3);
    inverts_.at(0) = -1; //Invert the Steering control


}

void CANRos::ActuatorControlCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    controls_.at(0) = msg->twist.angular.z; //steering
    controls_.at(1) = msg->twist.linear.x;  //throttle
    SendMotorsCmd();
}

uint8_t CANRos::RadsToDigital(float control)
{
    float rpm = control * 60.0f / (2 * M_PI);
    float digital_value = ((rpm / wheel_max_rpm_) * 255.0f);
    digital_value = digital_value > 255.0 ? 255.0 : digital_value;
    digital_value = digital_value < -255.0 ? -255.0 : digital_value;
    return (uint8_t)std::abs(digital_value);
}

bool CANRos::Arm(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res)
{
    if (req.value && !motor_drive_status_.armed)
    { //arm pixhawk
        motor_drive_status_.armed = true;
        motor_drive_status_pub_.publish(motor_drive_status_);
        dbg_->print(dbg_->DEBUG_LEVEL_1, "Arming Jethawk");
    }
    if (!req.value && motor_drive_status_.armed)
    {
        motor_drive_status_.armed = false;
        motor_drive_status_pub_.publish(motor_drive_status_);
        dbg_->print(dbg_->DEBUG_LEVEL_1, "Disrming Jethawk");
    }
    res.success = true;
    res.result = 0;
    return true;
}

void CANRos::SendMotorsCmd()
{
    n_->param<float>("/kiwibot/pwm/trim_turn", offsets_.at(0), 0.0f); //get trim turn
    // Calculate offsets and inverts for speed cmds
    for (int i = 0; i < 1; i++)
    {
        raw_pwm_out_[i] = (inverts_.at(i) * (controls_.at(i) + offsets_.at(i)));
    }
    // Compute wheel velocity
    float w_left = (2.0f * controls_.at(1) + controls_.at(0) * robot_length_) / (2.0f * radius_);
    float w_right = -(2.0f * controls_.at(1) - controls_.at(0) * robot_length_) / (2.0f * radius_);
    uint8_t w_left_dig = RadsToDigital(w_left);
    uint8_t w_right_dig = RadsToDigital(w_right);
    uint8_t directions = 0xAA; //brake on all motors
    if (w_left > 0)
    {
        if (w_right < 0)
            directions = 0x50; //left CCW - right CW (go forward)
        else
            directions = 0x55; //right CCW - left CCW (turn CCW)
    }
    else
    {
        if (w_right < 0)
            directions = 0x00; //right CW - left CW (turn CW)
        else
            directions = 0x05; //right CCW - left CW (go backward)
    }
    uint8_t data[5] = {directions, w_left_dig, w_left_dig, w_right_dig, w_right_dig};
    can_driver_->CANWrite(MOTORS_CONTROL_RAW_ID, 5, data);

    // Calculate speeds on wheels
    pwm_out_msg_.channels = raw_pwm_out_;
    pwm_out_pub_.publish(pwm_out_msg_);
    motor_drive_status_pub_.publish(motor_drive_status_);
    pwm_status_pub_.publish(can_driver_status_msg_);
}

void CANRos::ReadCANBus(){
    struct can_frame *frame_;
    frame_ = can_driver_->ReadMsg();
    if(frame_){
        for(int i=0; i<frame_->can_dlc; ++i){
            ROS_INFO("index: %i - value: %i \n", i, frame_->data[i]);
        }
    }
}

