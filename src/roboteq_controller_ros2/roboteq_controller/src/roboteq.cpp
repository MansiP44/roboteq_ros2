
#include "roboteq_controller/roboteq.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/algorithm/string.hpp>

namespace roboteq
{

Roboteq::Roboteq(){}

hardware_interface::CallbackReturn Roboteq::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    std::cout<<"Returning error from on_init"<<std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//   hw_start_sec_ =
//     hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
//   hw_stop_sec_ =
//     hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

   std::cout<<"in on_init()"<<std::endl;
    std::vector<std::string> joint_names;
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      joint_names.push_back(joint.name.c_str());
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(logger_, "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        hw_command_.push_back(0.0);
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(logger_, "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        hw_state_pos_.push_back(0.0);
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(logger_, "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        hw_state_vel_.push_back(0.0);
      }
    }
    roboteq_params["joint"] = joint_names;
    std::cout<<"Calling getParams()"<<std::endl;
    getParams(info_);
    //std::cout<<""<<std::endl;
    std::any_cast<uint32_t>(roboteq_params["baud_rate"]);
    mSerial = new roboteq::serial_controller(std::any_cast<string>(roboteq_params["serial_port"]), std::any_cast<uint32_t>(roboteq_params["baud_rate"]));
    std::cout<<"mSerial created"<<std::endl;
    mSerial->start();
#if hardwareConnected
    // First run dynamic reconfigurator
    setup_controller = false;
    // Initialize GPIO reading
    _isGPIOreading = false;
    // Load default configuration roboteq board
    this->getRoboteqInformation();

    //srv_board = roboteq_node->create_service<roboteq_interfaces::srv::Service>("system", &Roboteq::service_Callback);

    //motor_loop_ = false;
    _first = false;
    
    // Disable ECHO
    mSerial->echo(false);
    // Disable Script and wait to load all parameters
    mSerial->script(false);
    // Stop motors
    bool stop_motor = mSerial->command("EX");


    RCLCPP_INFO(logger_, "Stop motor: %s", stop_motor ? "true" : "false");
#endif
    auto joint_names_vec = std::any_cast<std::vector<std::string>>(roboteq_params["joint"]);
    
    if(joint_names_vec.size()==0)
      _first = true;

    // Initialize Joints 
    for(unsigned i=0; i < info_.joints.size(); ++i)
    {
        string motor_name = info_.joints.at(i).name;
        std::cout<<"Casting motor num"<<std::endl;
        std::cout << roboteq_params[motor_name+"_number"].type().name() << std::endl;

        int number = std::any_cast<int>(roboteq_params[motor_name+"_number"]);
        std::cout<<"Casted motor num"<<std::endl;
        RCLCPP_INFO(logger_, "Motor[%d] name: %s", number, info_.joints[i].name.c_str());
        mMotor.push_back(new Motor(mSerial, motor_name, number, roboteq_params));
        std::cout<<"Motor created"<<std::endl;
    }

    for(int i = 0; i < 6; ++i)
    {
        _param_pulse.push_back(new GPIOPulseConfigurator(mSerial, mMotor, "InOut", i+1));
    }
    for(int i = 0; i < 6; ++i)
    {
        _param_analog.push_back(new GPIOAnalogConfigurator(mSerial, mMotor, "InOut", i+1));
    }
    for(int i = 0; i < 2; ++i)
    {
        _param_encoder.push_back(new GPIOEncoderConfigurator(mSerial, mMotor, "InOut", i+1));
    }

    // TBD: subscriber and publisher in the form of export iterfaces to be added. Functions that carry functionality of the subscriber callbacks need to be added. 
  

    initialize();
    //initializeInterfaces(); //not required in ros2 as the command and state interfaces are directly loaded through the hardware interface plugins. 
    // interfaces are directly defined and exposed using the ros2_control tag in URDF. The implementation of the interfaces communicate with the hardware
    // is provided by the C++ plugin (roboteq.cpp)  


    //diagnostic_updater_.setHardwareID("RoboteqMotorController");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void Roboteq::setParam(const hardware_interface::HardwareInfo & info_, string name){
    if(info_.hardware_parameters.at(name).find(".")!= std::string::npos)  //if the value is double, it will have decimal point in its string
        roboteq_params[name] = hardware_interface::stod(info_.hardware_parameters.at(name));
    else                                              //use stoi if the value is int
        roboteq_params[name] = std::stoi(info_.hardware_parameters.at(name));
}

void Roboteq::getParams(const hardware_interface::HardwareInfo & info_){
    roboteq_params["serial_port"] = info_.hardware_parameters.at("serial_port");
    std::cout<<"serial_port: "<<std::any_cast<std::string>(roboteq_params["serial_port"])<<std::endl;
    roboteq_params["baud_rate"] = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("baud_rate")));
    roboteq_params["mixing"] = hardware_interface::stod(info_.hardware_parameters.at("mixing"));
    roboteq_params["break_delay"] = hardware_interface::stod(info_.hardware_parameters.at("break_delay"));
    roboteq_params["over_voltage_limit"] = hardware_interface::stod(info_.hardware_parameters.at("over_voltage_limit"));
    roboteq_params["over_voltage_hysteresis"] = hardware_interface::stod(info_.hardware_parameters.at("over_voltage_hysteresis"));
    roboteq_params["under_voltage_limit"] = hardware_interface::stod(info_.hardware_parameters.at("under_voltage_limit"));
    roboteq_params["pwm_frequency"] = hardware_interface::stod(info_.hardware_parameters.at("pwm_frequency"));
    
    string param_name = "InOut";
    for(auto type: inout_channel_input_type){
      for(size_t i= 1;i<7;i++){
        for(auto config: channel_config_params_name){
            string name = param_name + "_" + type + "_"+ to_string(i) + "_" + config;
            setParam(info_,name);
        }
      }
    }

  for(size_t i= 1;i<3;i++){
    for(auto config: encoder_params_name){
        string name = param_name + "_encoder" + "_" + to_string(i) + "_" + config;
        setParam(info_,name);
    }
  }
  auto joint_names_vec = std::any_cast<std::vector<std::string>>(roboteq_params["joint"]);
  
  for(auto joint: joint_names_vec){
    for(auto config: joint_params_name){
      string name;
     
      if(config == "pid_position" || config=="pid_torque" || config=="pid_velocity")
      {
        for(auto pid_config: pid_params_name){
          name = joint + "_" + config + "_" + pid_config;
          setParam(info_,name);

        }
      }
      else{
        name = joint + "_" + config;
        setParam(info_,name);

      }
     

    }

  }


}

void Roboteq::switch_off()
{
    // TODO: Add unregister all interfaces
    // Prevent
    // Controller Spawner error while taking down controllers: 
    // transport error completing service call: unable to receive data from sender, check sender's logs for details
    // Send emergency stop
    // And release motors
    mSerial->command("EX");
}



void Roboteq::getRoboteqInformation()
{
    // Load model roboeq board
    std::cout<<"Quering roboteq"<<std::endl;
    string trn = mSerial->getQuery("TRN");
    std::cout<<"Query done"<<std::endl;
    std::vector<std::string> fields;
    boost::split(fields, trn, boost::algorithm::is_any_of(":"));
    _type = fields[0];
    _model = fields[1];
    // ROS_INFO_STREAM("Model " << _model);
    // Load firmware version
    _version = mSerial->getQuery("FID");
    // Load UID
    _uid = mSerial->getQuery("UID");
}


void Roboteq::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    try
    {
        // Read values (same as before)
        string fault_flag = mSerial->getQuery("FF");
        unsigned char fault = boost::lexical_cast<unsigned int>(fault_flag);
        memcpy(&_fault, &fault, sizeof(fault));

        string status_flag = mSerial->getQuery("FS");
        //unsigned char status = boost::lexical_cast<unsigned int>(status_flag);
        unsigned int full_status = std::stoi(status_flag); 
        uint8_t status = static_cast<uint8_t>(full_status); 
        memcpy(&_flag, &status, sizeof(status));

        string supply_voltage_1 = mSerial->getQuery("V", "1");
        _volts_internal = boost::lexical_cast<double>(supply_voltage_1) / 10;

        string supply_voltage_3 = mSerial->getQuery("V", "3");
        _volts_five = boost::lexical_cast<double>(supply_voltage_3) / 1000;

        string temperature_1 = mSerial->getQuery("T", "1");
        _temp_mcu = boost::lexical_cast<double>(temperature_1);

        string temperature_2 = mSerial->getQuery("T", "2");
        _temp_bridge = boost::lexical_cast<double>(temperature_2);

        // Populate diagnostic fields
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Roboteq diagnostics nominal");
        stat.add("Internal Voltage (V)", _volts_internal);
        stat.add("5V Logic Voltage (V)", _volts_five);
        stat.add("MCU Temperature (°C)", _temp_mcu);
        stat.add("Bridge Temperature (°C)", _temp_bridge);
        //stat.add("Fault Flags", _fault);
        //stat.add("Status Flags", _flag);
    }
    catch (const std::exception &e)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Failed to parse diagnostic data");
    }
}


void Roboteq::initialize()
{
    // Check if is required load paramers
    if(_first)
    {
        // Load parameters from roboteq
        getControllerFromRoboteq();
    }

    // Launch initialization GPIO
    for (vector<GPIOPulseConfigurator*>::iterator it = _param_pulse.begin() ; it != _param_pulse.end(); ++it)
    {
        ((GPIOPulseConfigurator*)(*it))->initConfigurator(true, roboteq_params);
    }
    for (vector<GPIOAnalogConfigurator*>::iterator it = _param_analog.begin() ; it != _param_analog.end(); ++it)
    {
        ((GPIOAnalogConfigurator*)(*it))->initConfigurator(true, roboteq_params);
    }
    for (vector<GPIOEncoderConfigurator*>::iterator it = _param_encoder.begin() ; it != _param_encoder.end(); ++it)
    {
        ((GPIOEncoderConfigurator*)(*it))->initConfigurator(true, roboteq_params);
    }

    // Initialize all motors in list
    for (vector<Motor*>::iterator it = mMotor.begin() ; it != mMotor.end(); ++it)
    {
        Motor* motor = ((Motor*)(*it));
        // Launch initialization motors
        motor->initializeMotor(_first, roboteq_params);
        RCLCPP_DEBUG_STREAM(logger_, "Motor [" << motor->getName() << "] Initialized");
    }
}

void Roboteq::getControllerFromRoboteq()
{
    try
    {
        // Get PWM frequency PWMF
        string str_pwm = mSerial->getParam("PWMF");
        unsigned int tmp_pwm = boost::lexical_cast<unsigned int>(str_pwm);
        double pwm = ((double) tmp_pwm) / 10.0;
        // Set params
        roboteq_params["pwm_frequency"] = pwm;

        // Get over voltage limit OVL
        string str_ovl = mSerial->getParam("OVL");
        unsigned int tmp_ovl = boost::lexical_cast<unsigned int>(str_ovl);
        double ovl = ((double) tmp_ovl) / 10.0;
        // Set params
        roboteq_params["over_voltage_limit"] = ovl;

        // Get over voltage hystersis OVH
        string str_ovh = mSerial->getParam("OVH");
        unsigned int tmp_ovh = boost::lexical_cast<unsigned int>(str_ovh);
        double ovh = ((double) tmp_ovh) / 10.0;
        // Set params
        roboteq_params["over_voltage_hysteresis"] = ovh;

        // Get under voltage limit UVL
        string str_uvl = mSerial->getParam("UVL");
        unsigned int tmp_uvl = boost::lexical_cast<unsigned int>(str_uvl);
        double uvl = ((double) tmp_uvl) / 10.0;
        // Set params
        roboteq_params["under_voltage_limit"] = uvl;

        // Get brake activation delay BKD
        string str_break = mSerial->getParam("BKD");
        int break_delay = boost::lexical_cast<unsigned int>(str_break);
        // Set params
        roboteq_params["break_delay"] = break_delay;

        // Get Mixing mode MXMD
        //string str_mxd = mSerial->getParam("MXMD", "1");
        string str_mxd = mSerial->getParam("MXMD");
        // ROS_INFO_STREAM("MXMD "<< str_mxd);
        int mixed = boost::lexical_cast<unsigned int>(str_mxd);
        // Set params
        roboteq_params["mixing"] = mixed;

    } catch (std::bad_cast& e)
    {
        RCLCPP_WARN(logger_, "Failure parsing feedback data. Dropping message. %s", e.what());
    }
}


//service never called by any client in the original code. Hence commented
/*void Roboteq::service_Callback(const const std::shared_ptr<roboteq_interfaces::srv::Service::Request> req,
         std::shared_ptr<roboteq_interfaces::srv::Service::Response> msg){
    // Convert to lower case
    std::transform(req->service.begin(), req->service.end(), req->service.begin(), ::tolower);
    ROS_INFO_STREAM(logger_, "Name request: " << req->service);     

    if(req->service.compare("msginfo") == 0)
    {
        msg->information = "\nBoard type: " + _type + "\n"
                          "Name board: " + _model + "\n"
                          "Version: " + _version + "\n"
                          "UID: " + _uid + "\n";
    }
    else if(req->service.compare("reset") == 0)
    {
        // Launch reset command
        mSerial->reset();
        // return message
        msg->information = "System reset";
    }
    else if(req->service.compare("save") == 0)
    {
        // Launch reset command
        mSerial->saveInEEPROM();
        // return message
        msg->information = "Parameters saved";
    }
    else
    {
        msg->information = "\nList of commands availabes: \n"
                          "* info      - information about this board \n"
                          "* reset     - " + _model + " board software reset\n"
                          "* save      - Save all paramters in EEPROM \n"
                          "* help      - this help.";
    }
    return true;


}*/


hardware_interface::return_type Roboteq::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
  std::vector<std::string> motors[mMotor.size()];
    std::vector<std::string> fields;

    // Read status motor
    // motor status flags [pag. 246]
    string str_status = mSerial->getQuery("FM");
    // ROS_INFO_STREAM("FM=" << str_status);
    boost::split(fields, str_status, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor command [pag. 250]
    string str_motor = mSerial->getQuery("M");
    // ROS_INFO_STREAM("M =" << str_motor);
    std::cout<<"M =" << str_motor<<std::endl;
    boost::split(fields, str_motor, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor feedback [pag. 244]
    string str_feedback = mSerial->getQuery("F");
    //ROS_INFO_STREAM("F =" << str_feedback);
    std::cout<<"F =" << str_feedback<<std::endl;
    boost::split(fields, str_feedback, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor loop error [pag. 244]
    string str_loop_error = mSerial->getQuery("E");
    // ROS_INFO_STREAM("E =" << str_loop_error);
    boost::split(fields, str_loop_error, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor power [pag. 255]
    string str_motor_power = mSerial->getQuery("P");
    // ROS_INFO_STREAM("P =" << str_motor_power);
    std::cout<<"P =" << str_motor_power<<std::endl;
    boost::split(fields, str_motor_power, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // power supply voltage [pag. 262]
    string str_voltage_supply = mSerial->getQuery("V", "2");
    //ROS_INFO_STREAM("V2=" << str_voltage_supply);
    std::cout<<"V =" << str_voltage_supply<<std::endl;
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(str_voltage_supply);
    }
    // motor Amps [pag. 230]
    string str_motor_amps = mSerial->getQuery("A");
    //ROS_INFO_STREAM("A =" << str_motor_amps);
    std::cout<<"A =" << str_motor_amps<<std::endl;
    boost::split(fields, str_motor_amps, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor battery amps [pag. 233]
    string str_motor_battery_amps = mSerial->getQuery("BA");
    //ROS_INFO_STREAM("BA=" << str_motor_battery_amps);
    boost::split(fields, str_motor_battery_amps, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // position encoder value [pag. 236]
    string str_position_encoder_absolute = mSerial->getQuery("C");
    //ROS_INFO_STREAM("C =" << str_position_encoder_absolute);
    boost::split(fields, str_position_encoder_absolute, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor track [pag. 260]
    string str_motor_track = mSerial->getQuery("TR");
    //ROS_INFO_STREAM("TR=" << str_motor_track);
    boost::split(fields, str_motor_track, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }

    // send list  //readVector need a publisher like mechanism to publish the data
    for(int i = 0; i < mMotor.size(); ++i) {
        //get number motor initialization
        unsigned int idx = mMotor[i]->mNumber-1;
        // Read and decode vector
        mMotor[i]->readVector(motors[idx]);
    }

    //Also need some kind of publisher mechanism
    // Read data from GPIO 
    // _isGPIOreading is false by default. Set to true if any subsciber to the topic found
    // if(_isGPIOreading)
    // {
    //     msg_peripheral.header.stamp = ros::Time::now();
    //     std::vector<std::string> fields;
    //     // Get Pulse in status [pag. 256]
    //     string pulse_in = mSerial->getQuery("PI");
    //     boost::split(fields, pulse_in, boost::algorithm::is_any_of(":"));
    //     // Clear msg list
    //     msg_peripheral.pulse_in.clear();
    //     for(int i = 0; i < fields.size(); ++i)
    //     {
    //         try
    //         {
    //             msg_peripheral.pulse_in.push_back(boost::lexical_cast<unsigned int>(fields[i]));
    //         }
    //         catch (std::bad_cast& e)
    //         {
    //             msg_peripheral.pulse_in.push_back(0);
    //         }
    //     }
    //     // Get analog input values [pag. 231]
    //     string analog = mSerial->getQuery("AI");
    //     boost::split(fields, analog, boost::algorithm::is_any_of(":"));
    //     // Clear msg list
    //     msg_peripheral.analog.clear();
    //     for(int i = 0; i < fields.size(); ++i)
    //     {
    //         try
    //         {
    //             msg_peripheral.analog.push_back(boost::lexical_cast<double>(fields[i]) / 1000.0);
    //         }
    //         catch (std::bad_cast& e)
    //         {
    //             msg_peripheral.analog.push_back(0);
    //         }
    //     }

    //     // Get Digital input values [pag. 242]
    //     string digital_in = mSerial->getQuery("DI");
    //     boost::split(fields, digital_in, boost::algorithm::is_any_of(":"));
    //     // Clear msg list
    //     msg_peripheral.digital_in.clear();
    //     for(int i = 0; i < fields.size(); ++i)
    //     {
    //         try
    //         {
    //             msg_peripheral.digital_in.push_back(boost::lexical_cast<unsigned int>(fields[i]));
    //         }
    //         catch (std::bad_cast& e)
    //         {
    //             msg_peripheral.digital_in.push_back(0);
    //         }
    //     }

    //     string digital_out = mSerial->getQuery("DO");
    //     unsigned int num = 0;
    //     try
    //     {
    //         num = boost::lexical_cast<unsigned int>(digital_out);
    //     }
    //     catch (std::bad_cast& e)
    //     {
    //         num = 0;
    //     }
    //     int mask = 0x0;
    //     // Clear msg list
    //     msg_peripheral.digital_out.clear();
    //     for(int i = 0; i < 8; ++i)
    //     {
    //         msg_peripheral.digital_out.push_back((mask & num));
    //         mask <<= 1;
    //     }

    //     // Send GPIO status
    //     pub_peripheral.publish(msg_peripheral);
    // }

    //diagnostic_updater_.force_update();
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type Roboteq::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Commenting for testing  
  int i = 0;
    for (vector<Motor*>::iterator it = mMotor.begin() ; it != mMotor.end(); ++it)
    {
        Motor* motor = ((Motor*)(*it));
        // Launch initialization motors
        motor->writeCommandsToHardware(hw_command_[i]);
        //RCLCPP_INFO(logger_, "Motor [%s] ] Send commands" << motor->getName() << "");
        i++;
    }
    return hardware_interface::return_type::OK;
}
hardware_interface::CallbackReturn Roboteq::on_configure(const rclcpp_lifecycle::State & previous_state) {return hardware_interface::CallbackReturn::SUCCESS;}
hardware_interface::CallbackReturn Roboteq::on_activate(const rclcpp_lifecycle::State & previous_state) {     //functionality of doSwitch() in ros1 roboteq controller added in on_activate()
  //diagnostic_updater_.add("Roboteq Status", this, &Roboteq::diagnosticCallback);
  // setting motor in run mode
  RCLCPP_INFO(logger_, "On activate");
  for (int i = 0; i < mMotor.size(); i++)
  {
      // Stop the motor and delete the object
      mMotor[i]->stopMotor();
  }

  mSerial->command("EX");
  for (int i = 0; i < mMotor.size(); i++)
  {
      mMotor[i]->setOperativeMode(roboteq_params);
  }

  // Enable motor
  mSerial->command("MG");

  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Roboteq::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        int i = 0;
    for ( i = 0; i < mMotor.size(); i++)
    {
        // Stop the motor and delete the object
        mMotor[i]->stopMotor();
        delete mMotor[i];
    }
    for ( i = 0; i < _param_pulse.size(); i++)
    {
        delete _param_pulse[i];
    }
    for ( i = 0; i < _param_analog.size(); i++)
    {
        delete _param_analog[i];
    }
    for ( i = 0; i < _param_encoder.size(); i++)
    {
        delete _param_encoder[i];
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Roboteq::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  auto joint_names_vec = std::any_cast<std::vector<std::string>>(roboteq_params["joint"]);
    for(int i= 0; i<joint_names_vec.size(); i++){
      RCLCPP_INFO(logger_, "export_state_interfaces for motor %i", i);
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
          joint_names_vec[i],
          hardware_interface::HW_IF_POSITION,
          &hw_state_pos_[i]
        )
      );
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
          joint_names_vec[i],
          hardware_interface::HW_IF_VELOCITY,
          &hw_state_vel_[i]
        )
      );
    }

  return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> Roboteq::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  auto joint_names_vec = std::any_cast<std::vector<std::string>>(roboteq_params["joint"]);
    for(int i= 0; i<joint_names_vec.size(); i++){
      RCLCPP_INFO(logger_, "adding command vel for motor %s %f",joint_names_vec[i].c_str() , hw_command_[i]);
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint_names_vec[i].c_str(),
          hardware_interface::HW_IF_VELOCITY,
          &hw_command_[i]
        )
      );
    }

  return command_interfaces;
}

Roboteq::~Roboteq(){}

}  //namespace


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roboteq::Roboteq, hardware_interface::SystemInterface)