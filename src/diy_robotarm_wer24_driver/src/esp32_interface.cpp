//Source File for the Controller talking to our ESP32 Hardware
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

//ESP32 specific header
#include "esp32_robot_driver/esp32_interface.hpp"
#include "esp32_robot_driver/robot_connection.hpp"

//Destruct Instances if there are already some to prevent some weired behavior (calls class from esp32_inteface.hpp)
ESP32Hardware::~ESP32Hardware()

//generate an instance from RobotConnection class in robot_connection.hpp which handles communication over TCP-IP to the ESP
RobotConnection robotConnection;

//redefine/ override the base controller methods to our custom purposes (following methods constructed in esp32_interface.hpp)
namespace esp32_robot_driver {

//###################################################################################################################
//on_init is called when initializing the controller (init variables, declare node parameters used by the controller)
//###################################################################################################################
  hardware_interface::CallbackReturn ESP32Hardware::on_init(const hardware_interface::HardwareInfo & info) {

    // Return an error if the modified controller can't be initialized:
    if (hardware_interface::SystemInterface::on_init(info) !=  hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR; 
    }
    //Call methods from robot_connection.hpp, resize there initialized arrays to match the amount of joints provided in the urdf (info_) and init them with NaN or 0
    // READ-VALUES (Axis Positions)
    robotConnection.hw_states_axisPositions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // READ/WRITE-Values (Axis Setpoints)
    robotConnection.hw_cmd_axisSetpoints.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN()); 
    robotConnection.hw_cmd_axisSetpoints_prev.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    

    for (const hardware_interface::ComponentInfo & joint : info_.joints) { //loops over all defined joints (refering to urdf or info_.joints)
      //Setup the Command Interfaces and feedback in terminal:  
      if (joint.command_interfaces.size() != 1)  {
        RCLCPP_FATAL(rclcpp::get_logger("ESP32_Driver"),
          "Joint '%s' has %zu command interfaces, but only expects 1.",
          joint.name.c_str(), 
          joint.command_interfaces.size()
        );
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(rclcpp::get_logger("ESP32_Driver"),
          "Joint '%s' found %s interface, but expects %s.", 
          joint.name.c_str(), 
          joint.command_interfaces[0].name.c_str(), 
          hardware_interface::HW_IF_POSITION
        );
        return hardware_interface::CallbackReturn::ERROR;
      }

      //Setup the Command Interfaces and feedback in terminal:  
      if (joint.state_interfaces.size() != 1)  {  //checks state interfaces of the axis
        RCLCPP_FATAL(rclcpp::get_logger("ESP32_Driver"),
          "Joint '%s' has %zu state interfaces, but expects 2.",
          joint.name.c_str(), 
          joint.state_interfaces.size()
        );
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(rclcpp::get_logger("ESP32_Driver"),
          "State Interface 0 for Joint '%s' must be %s.", 
          joint.name.c_str(),
          hardware_interface::HW_IF_POSITION
        );
        return hardware_interface::CallbackReturn::ERROR;
      }

      //further feedback in terminal in case of fatal_error
      RCLCPP_FATAL(rclcpp::get_logger("ESP32_Driver"), "Joint is called  \"%s\"", joint.name.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS; 
  }

//###########################################################################################################################
// on_configure is called after on_init, parameters from ros2_control_urdf.xcacro are read here and passed to the controller
//###########################################################################################################################
  hardware_interface::CallbackReturn ESP32Hardware::on_configure(const rclcpp_lifecycle::State & previous_state) {

    //Get the parameters for the robot:
    std::string tf_prefix = info_.hardware_parameters["tf_prefix"];
    std::string ipAddress = info_.hardware_parameters["robot_ip"];
    std::string ssid = info_.hardware_parameters["robot_ssid"];

    std::string errorMessage = "During Robot Connection: ";

    //Try to connect to the robot with the given configuration by calling the desired method in RobotConnection Class 
    if (robotConnection.initialize(ipAddress, ssid, errorMessage) == false) {
      RCLCPP_FATAL(rclcpp::get_logger("ESP32_Driver"), errorMessage.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } 

    // // Set initial position values (defined in diy_robotarm_wer24_description package): //wird doch eigentlich in on_init schon gemacht
    // for (auto i = 0; i < robotConnection.hw_states_axisPositions.size(); i++) {
    //   robotConnection.hw_states_axisPositions[i] = 0;
    //   robotConnection.hw_cmd_axisSetpoints[i] = 0;
    // }

    RCLCPP_INFO(rclcpp::get_logger("ESP32_Driver"), "Successfully configured the ESP32_Driver!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }


//############################################################################################################################
// export_state_interfaces is called to match the defined interfaces from the urdf to set them up and 
// registered for communication with the underlying hardware --> Configure interfaces
//############################################################################################################################
  std::vector<hardware_interface::StateInterface> ESP32Hardware::export_state_interfaces() {

    std::vector<hardware_interface::StateInterface> state_interfaces;

//adds a hardware_interface::StateInterface object to the state_interfaces vector with the specified joint name,
//interface type (position) and a pointer to the corresponding position value (in vector used in RobotConnection class)
    for (uint8_t i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION, 
        &robotConnection.hw_states_axisPositions[i]
      ));
    }
    return state_interfaces;
  }

//##################################################################################################################################
// export_command_interfaces is called to match the defined interfaces from the urdf to set them up and 
// registered for communication with the underlying hardwareterfaces --> Configure interfaces
//##################################################################################################################################
  std::vector<hardware_interface::CommandInterface> ESP32Hardware::export_command_interfaces() {

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint8_t i = 0; i < info_.joints.size(); i++) { 
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &robotConnection.hw_cmd_axisSetpoints[i]  
      ));
    }
    return command_interfaces;
  }

//###############################################################################################################################
// [realtime-loop] on_activate is called when the control loop gets activated --> interfaces get established/ datatransfer starts
//###############################################################################################################################
  hardware_interface::CallbackReturn ESP32Hardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("ESP32_Driver"), "on_activate has been called, motors are on power!");
    robotConnection.toggleActuatorPower(true);
    return hardware_interface::CallbackReturn::SUCCESS;
  };

//################################################################################################################################
// [realtime-loop] on_deactivate is called when the control loop gets deactivated --> interfaces get closed and datatransfer stops
//################################################################################################################################
  hardware_interface::CallbackReturn ESP32Hardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("ESP32_Driver"), "on_deactivate has been called, motors are free!");
    robotConnection.toggleActuatorPower(false);
    return hardware_interface::CallbackReturn::SUCCESS;
  };

//###########################################################################################################################
// [realtime-loop] read is called to read previous setpoints from the hardware --> read control-value (Istwert)
//###########################################################################################################################
  hardware_interface::return_type ESP32Hardware::read(const  rclcpp::Time & time, const rclcpp::Duration & period) {
    robotConnection.readData(); 
    return hardware_interface::return_type::OK;
  }

//###########################################################################################################################
// [realtime-loop] write is called to write new commands to the hardware --> write setpoints to reach a control-error = 0
//###########################################################################################################################
  hardware_interface::return_type ESP32Hardware::write(const  rclcpp::Time & time, const rclcpp::Duration & period) {
    // robotConnection.fakeUpdate();    // Fake the output values
    std::stringstream message;

    // Add a log string if the axis setpoints receive new values --> outcommentet to save runtime in realtime loop!
    for (size_t i = 0; i < robotConnection.hw_cmd_axisSetpoints.size(); i++) {
      if (robotConnection.hw_cmd_axisSetpoints[i] != robotConnection.hw_cmd_axisSetpoints_prev[i]) message << "New Setpint: Axis[" << i << "] := " << robotConnection.hw_cmd_axisSetpoints[i] << std::endl;
      robotConnection.hw_cmd_axisSetpoints_prev[i] = robotConnection.hw_cmd_axisSetpoints[i];
    }

    // Add a log string if the digital outputs receive new values
    for (size_t i = 0; i < robotConnection.hw_cmd_digitalOutputs.size(); i++) {
      if (robotConnection.hw_cmd_digitalOutputs[i] != robotConnection.hw_cmd_digitalOutputs_prev[i]) message << "New Setpoint: Output[" << i << "] := " << robotConnection.hw_cmd_digitalOutputs[i] << std::endl;
      robotConnection.hw_cmd_digitalOutputs_prev[i] = robotConnection.hw_cmd_digitalOutputs[i];
    }

    // Send the new data to the robot by using RobotConnection class
    robotConnection.sendData();

    // Print the log messages, if required
    std::string output = message.str().c_str();
    if (output != "") RCLCPP_INFO(rclcpp::get_logger("ESP32_Driver"), output.c_str());
      
    return hardware_interface::return_type::OK;
  }

} //close namespace

//export class to plugin (makes driver accessable for ros2) <controller_name_namespace>::<ControllerName>, see esp32_interface_plugin.xml
PLUGINLIB_EXPORT_CLASS(esp32_robot_driver::ESP32Hardware, hardware_interface::SystemInterface)