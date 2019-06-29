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
        // ============ Publishers ===========
        // Motors control
        motor_drive_status_pub_ = n_->advertise<mavros_msgs::State>("/kiwibot/driveController/status", 10);
        pwm_out_pub_ = n_->advertise<socket_can_ros::PwmOut>("/kiwibot/pwm/out", 10);
        pwm_status_pub_ = n_->advertise<std_msgs::String>("/kiwibot/pwm/status", 10);
        motor_status_pub_ = n_->advertise<socket_can_ros::Motors>("/kiwibot/motor/status", 10);
        test_motors_pub_ = n_->advertise<socket_can_ros::TestMotors>("/kiwibot/test_motors/response", 10);
        battery_pub_ = n_->advertise<sensor_msgs::BatteryState>("/kiwibot/sensors/battery_can", 10);

        // ============ Subscribers ===========
        // Motors control
        actuator_control_sub_ = n_->subscribe<geometry_msgs::TwistStamped>("/kiwibot/actuator_control", 1000, &CANRos::ActuatorControlCb, this);
        chassis_config_sub_ = n_->subscribe<socket_can_ros::Configuration>("/kiwibot/chassis_configuration", 1000, &CANRos::ChassisConfigCb, this);
        motors_sleep_sub_ = n_->subscribe<std_msgs::Bool>("/kiwibot/sleep", 1000, &CANRos::MotorsSleepCb, this);

        // ============ Services ===========
        // Motors control
        arming_service_ = n_->advertiseService("/mavros/cmd/arming", &CANRos::Arm, this);
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
    motor_max_current_ = getEnv("MOTOR_MAX_CURRENT", 165.0f);

    controls_ = {0.0f, 0.0f, 0.0f};
    offsets_ = {0.0f, 0.0f, 0.0f};
    inverts_ = {1.0f, 1.0f, 1.0f};
    raw_pwm_out_.resize(3);
    inverts_.at(0) = -1; //Invert the Steering control

    //Initial configuration
    chassis_configuration_.wheels_baudrate = WHEELS_BAUDRATE_DEFAULT;
    chassis_configuration_.battery_status_baudrate = BATTERY_STATUS_BAUDRATE_DEFAULT;
    chassis_configuration_.wheel_control_mode = WHEEL_CONTROL_MODE_DEFAULT;
    chassis_configuration_.operation_mode = OPERATION_MODE_DEFAULT;
    chassis_configuration_.number_battery_cells = NUMBER_BATTERY_CELLS_DEFAULT;
    chassis_configuration_.motors_model = MOTORS_MODEL_DEFAULT;
    SendChassisConfiguration();

    // Initialize motor status
    motors_status_.rpm = {0.0, 0.0, 0.0, 0.0};
    motors_status_.current = {0.0, 0.0, 0.0, 0.0};
    motors_status_.error_status = {0, 0, 0, 0};
    test_motors_response_.status = {0, 0, 0, 0};

    error_timer_ = n_->createTimer(ros::Duration(0.1), &CANRos::ErrorTimerCb, this);
    error_timer_.stop();
}

void CANRos::ErrorTimerCb(const ros::TimerEvent &event)
{
    error_timer_.stop();
    for (int i = 0; i < 4; ++i)
        motors_status_.error_status[i] = 0;
}

void CANRos::ActuatorControlCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    controls_.at(0) = msg->twist.angular.z; //steering
    controls_.at(1) = msg->twist.linear.x;  //throttle
    SendMotorsCmd();
}

void CANRos::ChassisConfigCb(const socket_can_ros::Configuration::ConstPtr &msg)
{
    chassis_configuration_ = *msg;
    SendChassisConfiguration();
    ROS_INFO("-----------------Chassis Configured!---------------");
}

void CANRos::MotorsSleepCb(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        uint8_t data[1] = {0x02};
        can_driver_->CANWrite(ARM_CMD_ID, 1, data);
    }
    ROS_INFO("-----------------Motors Sleep cmd-----------------");
}

bool CANRos::Arm(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res)
{
    uint8_t data[1];
    data[0] = 0x00;
    if (req.value && !motor_drive_status_.armed)
        data[0] = 0x01;
    if (!req.value && motor_drive_status_.armed)
        data[0] = 0x00;
    can_driver_->CANWrite(ARM_CMD_ID, 1, data);
    res.success = true;
    res.result = 0;
    return true;
}

uint8_t CANRos::RadsToDigital(float control)
{
    float rpm = control * 60.0f / (2 * M_PI);
    float digital_value = ((rpm / wheel_max_rpm_) * 255.0f);
    digital_value = digital_value > 255.0 ? 255.0 : digital_value;
    digital_value = digital_value < -255.0 ? -255.0 : digital_value;
    return (uint8_t)std::abs(digital_value);
}

void CANRos::PublishChassisStatus(struct can_frame *frame)
{
    switch (frame->can_id)
    {
    case WHEELS_INFO_ID:
    {
        uint8_t dir_b = frame->data[0];
        bool directions_bool[4] = {((dir_b & RIGHT_FRONT_WHEEL) == 0x00), ((dir_b & RIGHT_REAR_WHEEL) == 0x00), ((dir_b & LEFT_FRONT_WHEEL) == 0x00), ((dir_b & LEFT_REAR_WHEEL) == 0x00)};
        int directions[4];
        for (int i = 0; i < 4; ++i)
            directions[i] = directions_bool[i] ? 1 : -1;
        for (int i = 0; i < 4; ++i)
            motors_status_.rpm[i] = ((float)directions[i] * (float)frame->data[i + 1]) * wheel_max_rpm_ / 255.0f;
        motors_status_.header.stamp = ros::Time::now();
        motor_status_pub_.publish(motors_status_);
        break;
    }
    case MOTORS_CURRENT_ID:
    {
        for (int i = 0; i < frame->can_dlc; ++i)
            motors_status_.current[i] = (float)frame->data[i] * motor_max_current_ / 255.0;
        break;
    }
    case ERROR_CMD_ID:
    {
        error_timer_.setPeriod(ros::Duration(1.5), true); // IMPORTANT: Check error msg logic (implement timer?)
        error_timer_.start();
        for (int i = 0; i < frame->can_dlc; ++i)
            motors_status_.error_status[i] = frame->data[i];
        break;
    }
    break;
    case TEST_REPORT_CMD_ID:
    {
        for (int i = 0; i < frame->can_dlc; ++i)
            test_motors_response_.status[i] = frame->data[i];
        test_motors_pub_.publish(test_motors_response_);
        break;
    }

    case STATUS_CMD_ID:
    {
        if (frame->data[0] = 0x01)
        {
            motor_drive_status_.armed = true;
            motor_drive_status_pub_.publish(motor_drive_status_);
            dbg_->print(dbg_->DEBUG_LEVEL_1, "Arming Jethawk");
        }
        else
        {
            motor_drive_status_.armed = false;
            motor_drive_status_pub_.publish(motor_drive_status_);
            dbg_->print(dbg_->DEBUG_LEVEL_1, "Disarming Jethawk");
        }
        // battery
        battery_msg_.header.stamp = ros::Time::now();
        battery_msg_.voltage = (float)frame->data[1] + (float)frame->data[2]/255.0; //IMPORTANT: CHECK SCALE
        break;
    }
    default:
        ROS_INFO("si");
        break;
    }
}

void CANRos::SendChassisConfiguration()
{
    uint8_t data[6] = {chassis_configuration_.wheels_baudrate, chassis_configuration_.battery_status_baudrate, chassis_configuration_.wheel_control_mode,
                       chassis_configuration_.operation_mode, chassis_configuration_.number_battery_cells, chassis_configuration_.motors_model};
    can_driver_->CANWrite(CONFIGURATION_CMD_ID, 6, data);
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
    uint8_t directions = 0xAA;
    // brake on all motors
    if (w_left > 0)
    {
        if (w_right < 0)
            directions = 0x05; //left CCW - right CW (go forward)
        else
            directions = 0x55; //right CCW - left CCW (turn CCW)
    }
    else
    {
        if (w_right < 0)
            directions = 0x00; //right CW - left CW (turn CW)
        else
            directions = 0x50; //right CCW - left CW (go backward)
    }
    uint8_t data[5] = {directions, w_left_dig, w_left_dig, w_right_dig, w_right_dig};
    if (chassis_configuration_.wheel_control_mode == 0x00)
        can_driver_->CANWrite(WHEELS_CONTROL_RPM_ID, 5, data);
    else if (chassis_configuration_.wheel_control_mode == 0x01)
        can_driver_->CANWrite(MOTORS_CONTROL_RAW_ID, 5, data);
    // Calculate speeds on wheels
    pwm_out_msg_.channels = raw_pwm_out_;
    pwm_out_pub_.publish(pwm_out_msg_);
    motor_drive_status_pub_.publish(motor_drive_status_);
    pwm_status_pub_.publish(can_driver_status_msg_);
}

void CANRos::ReadCANBus()
{
    struct can_frame *frame;
    frame = can_driver_->ReadMsg();
    if (frame)
    {
        PublishChassisStatus(frame);
        for (int i = 0; i < frame->can_dlc; ++i)
        {
            ROS_INFO("index: %i - value: %i \n", i, frame->data[i]);
        }
    }
}
