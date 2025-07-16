/**
 * Copyright (C) 2017, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>

#include "std_msgs/msg/bool.hpp"
#include <roboteq_interfaces/srv/service.hpp>
#include <roboteq_interfaces/msg/peripheral.hpp>

//#include <diagnostic_updater/diagnostic_updater.hpp>
//#include <diagnostic_updater/publisher.hpp>


//#include <hardware_interface/robot_hw.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

//#include <roboteq_control/RoboteqControllerConfig.h>

#include "configurator/gpio_analog.hpp"
#include "configurator/gpio_pulse.hpp"
#include "configurator/gpio_encoder.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

#include "roboteq_controller/motor.hpp"
#include <map>
#include <any>



using namespace std;
#define hardwareConnected 1

namespace roboteq
{

typedef struct joint
{
    Motor *motor;
    // State of the motor
    double position;
    double velocity;
    double effort;
    double velocity_command;
} joint;


typedef struct _status_flag {
    uint8_t serial_mode : 1;
    uint8_t pulse_mode : 1;
    uint8_t analog_mode : 1;
    uint8_t spectrum : 1;
    uint8_t power_stage_off : 1;
    uint8_t stall_detect : 1;
    uint8_t at_limit : 1;
    uint8_t microbasic_running : 1;
} status_flag_t;
// Reference in pag 245
typedef struct _status_fault {
    uint8_t overheat : 1;
    uint8_t overvoltage : 1;
    uint8_t undervoltage : 1;
    uint8_t short_circuit : 1;
    uint8_t emergency_stop : 1;
    uint8_t brushless_sensor_fault : 1;
    uint8_t mosfet_failure : 1;
} status_fault_t;

// typedef struct channel_config_params{
//     int conversion;
//     int conversion_polarity;
//     int input_deadband;
//     int input_motor_one;
//     int input_motor_two;
//     int input_use;
//     double range_input_center;
//     double range_input_max;
//     double range_input_min;

// } channel_config_params_t;

// typedef struct channels{
//     channel_config_params_t ch1;
//     channel_config_params_t ch2;
//     channel_config_params_t ch3;
//     channel_config_params_t ch4;
//     channel_config_params_t ch5;
//     channel_config_params_t ch6;
// } channels_t;



// typedef struct encoder_params{
//     int PPR;
//     int configuration;
//     int encoder_high_count_limit;
//     int encoder_home_count;
//     int encoder_low_count_limit;
//     int input_motor_one;
//     int input_motor_two;
//     int position;
// } encoder_params_t;

// typedef struct encoder_config{
//     encoder_params_t _1;
//     encoder_params_t _2;
// } encoder_config_t;


// typedef struct in_out{
//     channels_t analog;
//     channels_t pulse;
//     encoder_config_t encoder;
// } in_out_t;

// typedef struct pid_config{
//     double Kp;
//     double Ki;
//     double Kd;
//     int integrator_limit;
//     int loop_error_detection;
//     int position_mode_velocity;
//     double turn_min_to_max;

// } pid_config_t;

// typedef struct pid_param{
//     int closed_loop_position;
//     int closed_loop_velocity;
//     pid_config_t position;
//     pid_config_t torque;
//     pid_config_t velocity;

// } pid_param_t;

// typedef struct motor_def{
//     int number;
//     int ratio;
//     int rotation;
//     int stall_detection;
//     double amper_limit;
//     double max_speed;
//     double max_acceleration;
//     double max_deceleration;
//     int max_forward;
//     int max_reverse;
//     pid_param_t pid;

// } motor_def_t;

// typedef struct motor_controller_params{
//     int mixing;
//     int break_delay;
//     double over_voltage_limit;
//     double over_voltage_hysteresis;
//     double under_voltage_limit;
//     double pwm_frequency;
//     std::vector<string> joints;
//     in_out_t InOut;
//     motor_def_t left_joint;
//     motor_def_t right_joint;

// } motor_controller_params_t;

std::vector<string> inout_channel_input_type{"analog","pulse"};
std::vector<string> channel_config_params_name{"conversion","conversionPolarity","inputDeadband", "inputMotorOne","inputMotorTwo", "inputUse", "rangeInputCenter", "rangeInputMax","rangeInputMin"};
std::vector<string> encoder_params_name{"PPR", "configuration", "encoderHighCountLimit", "encoderHomeCount", "encoderLowCountLimit", "inputMotorOne", "inputMotorTwo", "position"};
std::vector<string> pid_params_name{"Kp","Ki","Kd","integratorLimit", "loopErrorDetection", "positionModeVelocity", "turnMinToMax"};
std::vector<string> joint_params_name{"number", "ratio", "rotation", "stallDetection", "amperLimit", "maxSpeed", "maxAcceleration", "maxDeceleration", "maxForward", "maxReverse", "pid_closedLoopPosition", "pid_closedLoopVelocity", "pid_position", "pid_torque", "pid_velocity" };



class Roboteq : public hardware_interface::SystemInterface //, public diagnostic_updater::DiagnosticTask
{
public:
    /**
     * @brief Roboteq The Roboteq board controller write and read messages about the motor state
     * @param serial The serial controller
     */
    //Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, serial_controller *serial);
    /**
      * @brief The deconstructor
      */
    Roboteq();  // pluginlib dynamically creates a plugin instace using a default constructor. Hence it needs to be defined 
    ~Roboteq();
    /**
     * @brief Switch off roboteq board
     */

    RCLCPP_SHARED_PTR_DEFINITIONS(Roboteq)  //allows to create shared ptr for the class

    rclcpp::Logger logger_ = rclcpp::get_logger("ros2_control_node");

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    std::vector<YAML::Node> motor_config;       //0-> left_motor, 1-> right_motor

    //diagnostic_updater::Updater diagnostic_updater_;
    //motor_controller_params_t roboteq_params;

    std::map<std::string, std::any> roboteq_params;
    std::vector<double> hw_state_pos_, hw_state_vel_, hw_command_;

    void getParams(const hardware_interface::HardwareInfo & info_);
    void setParam(const hardware_interface::HardwareInfo & info_, string name);
    void switch_off();
    /**
     * @brief run Diagnostic thread called every request
     * @param stat the status of diagnostic updater
     */
    //void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
    /**
     * @brief initialize the roboteq controller
     */
    //void initialize();
    /**
     * @brief initializeInterfaces Initialize all motors.
     * Add all Control Interface availbles and add in diagnostic task
     */
    //void initializeInterfaces();
    /**
     * @brief updateDiagnostics
     */
    //void updateDiagnostics();

    //void initializeDiagnostic();

    //void write(const ros::Time& time, const ros::Duration& period);

    //void read(const ros::Time& time, const ros::Duration& period);

    //bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    //void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

private:

    // Serial controller
    serial_controller *mSerial;
    // Diagnostic
    //diagnostic_updater::Updater diagnostic_updater;
    // Publisher status periheral
    //ros::Publisher pub_peripheral;
    //rclcpp::Publisher<roboteq_interfaces::msg::Peripheral>::SharedPtr pub_peripheral;
    // stop publisher
    //ros::Subscriber sub_stop;
    //rclcpp::Subscriber<std_msgs::msg::Bool>::SharedPtr sub_stop;
    // Service board
    //ros::ServiceServer srv_board;
    //rclcpp::Service<roboteq_interfaces::srv::Service>::SharedPtr srv_board;

    /// URDF information about robot
    urdf::Model model;

    /// ROS Control interfaces
    //hardware_interface::JointStateInterface joint_state_interface;
    //hardware_interface::VelocityJointInterface velocity_joint_interface;

    bool motor_loop_;
    // Check if is the first run
    bool _first;
    // Motor definition
    //map<string, Motor*> mMotor;
    std::vector<Motor*> mMotor;

    string _type, _model;
    string _version;
    string _uid;
    // Status Roboteq board
    status_flag_t _flag;
    // Fault flags Roboteq board
    status_fault_t _fault;
    // Volts internal
    double _volts_internal, _volts_five;
    // Tempearture inside the Roboteq board
    double _temp_mcu, _temp_bridge;

    // GPIO enable read
    bool _isGPIOreading;
    //roboteq_interfaces::msg::Peripheral msg_peripheral;
    std::vector<GPIOAnalogConfigurator*> _param_analog;
    std::vector<GPIOPulseConfigurator*> _param_pulse;
    // Encoder
    std::vector<GPIOEncoderConfigurator*> _param_encoder;


    // stop callback
    //void stop_Callback(const std_msgs::msg::Bool::ConstPtr& msg);
    /**
     * @brief getRoboteqInformation Load basic information from roboteq board
     */
    void getRoboteqInformation();
    void initialize();
    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void getControllerFromRoboteq();

    /// Setup variable
    bool setup_controller;

    /// Dynamic reconfigure PID
    // Dynamic reconfigure
    //boost::recursive_mutex mDynServerMutex; // To avoid Dynamic Reconfigure Server warning
    //boost::shared_ptr<dynamic_reconfigure::Server<roboteq_control::RoboteqControllerConfig>> mDynRecServer;
    /**
     * @brief reconfigureCBEncoder when the dynamic reconfigurator change some values start this method
     * @param config variable with all configuration from dynamic reconfigurator
     * @param level
     */
    //void reconfigureCBController(roboteq_control::RoboteqControllerConfig &config, uint32_t level);

    // Default parameter config
    //roboteq_control::RoboteqControllerConfig default_controller_config, _last_controller_config;

    /**
     * @brief getPIDFromRoboteq Load PID parameters from Roboteq board
     */
    //void getControllerFromRoboteq();

    /**
     * @brief service_Callback Internal service to require information from the board connected
     * @param req
     * @param msg
     * @return
     */
    //bool service_Callback(roboteq_control::Service::Request &req, roboteq_control::Service::Response &msg_system);
    /**
     * @brief connectionCallback Check how many subscribers are connected
     * @param pub The information about the subscriber
     */
    //void connectionCallback(const ros::SingleSubscriberPublisher& pub);

};

}


#endif // ROBOTEQ_H
