#ifndef ROBOT_CONNECTION_HPP_
#define ROBOT_CONNECTION_HPP_

#include <iostream>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <array>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <functional>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "DataFormat.hpp"

#include "helpers.hpp"

class RobotConnection;

struct RobotConnectionArgs {
  RobotConnection * robot;
  std::string ipAddress;
  uint8_t port;
} connectionConfig;

Communication::PcToRobot_t request;
Communication::RobotToPc_t response;

pthread_t threadId_;

class RobotConnection {
private:
  std::string textOutput;

  uint8_t messageNumber_ = 0;
  int socketConnection_;

  uint64_t sendTime = 0;
  int bytesSent = 0;
  uint64_t responseTime_;


  bool newMessageRevceived_ = false;
  std::string newMessage_ = "";
  double movementScale = 100;

public:
  // ros2_control Command Interfaces:
  std::vector<double> hw_cmd_axisSetpoints;     // Axis Setpoints
  std::vector<double> hw_cmd_digitalOutputs;    // Digital Output Setpoints

  // ros2_control State Interfaces:
  std::vector<double> hw_states_axisPositions;  // Axis Position
  std::vector<double> hw_states_digitalInputs;  // Digital Input States

  // Arrays to contain the previous setpoints used for logging changes:
  std::vector<double> hw_cmd_axisSetpoints_prev;  // Axis Setpoints
  std::vector<double> hw_cmd_digitalOutputs_prev; // Digital Output Setpoints


  // Update the digital outputs:
  void setDigitalOutputs(std::vector<uint8_t> & digitalOutputSetpoints) {
    for (auto i = 0; i < digitalOutputSetpoints.size(); i++) {
      hw_cmd_digitalOutputs[i] = (double) digitalOutputSetpoints[i];
    }
  }

  // Get all function for the custom publishers.
  // This function is called in a timer callback, after which the data is published with the various publishers.
  void getDataToPublish(std::vector<uint8_t> & digitalInputStates, std::vector<uint8_t> & digitalOutputStates, std::string & statusMessage, std::vector<std::string> & outputLines) {
    std::ostringstream status;

    // Copy the states of digital inputs and outputs:
    for (auto digitalInputState : hw_states_digitalInputs) digitalInputStates.push_back((uint8_t) digitalInputState);
    for (auto digitalOutputState : hw_cmd_digitalOutputs) digitalOutputStates.push_back((uint8_t) digitalOutputState);

    // Create a JSON payload containing the communication delay:
    status << "{\"delay\":" << responseTime_ << "}";
    statusMessage = status.str();

    // Add all received text lines to the outputLines Vector:
    size_t pos = textOutput.find('\n');
    while (pos != std::string::npos) {
      std::string line = textOutput.substr(0, pos);
      outputLines.push_back(line);
      textOutput.erase(0, pos + 1);
      pos = textOutput.find('\n');
    }

  };

  // Enable or disable the actuator power
  // This is called in the ros2_control hardware interface functions "CustomHardware::on_activate" and "CustomHardware::on_deactivate"
  void toggleActuatorPower(bool enable = false) {
    request.enablePower = enable;
  };

  // DEBUG Function ot fake the axis positions to the setpoints:
  void fakeUpdate() {
    for (auto i = 0; i < hw_cmd_axisSetpoints.size(); i++) {
      hw_states_axisPositions[i] = hw_cmd_axisSetpoints[i];
    }
  };

  // Callback function to handle incoming command strings.
  // It is called whenever the command topic subscriber receives a new message.
  bool messageHandler(std::string & message) {
    newMessageRevceived_ = true;
    newMessage_ = message + "\n";   // Append the message to the buffer string:
    return true;
  }

  // Function to read the data response from the robot.
  // It is called in the ros2_control hardware interface function "CustomHardware::read" to read the current states:
  bool readData() {
    if (bytesSent == 0) return true;  // Return if no response is expected

    // Read data from the socket
    int bytesReceived = recv(socketConnection_, &response, sizeof(response), 0);
    
    responseTime_ = MyUtils::micros() - sendTime;   // Calculate the communciation delay

    // Update the digital input states from the received values:
    for (auto i = 0; i < hw_states_digitalInputs.size(); i++) {
      hw_states_digitalInputs[i] = (double) response.digitalInputs[i];
    }

    // Update the axis position states from the received values:
    for (auto i = 0; i < hw_states_axisPositions.size(); i++) {
      hw_states_axisPositions[i] = (double) response.jointPositions[i] / 57295.7795131;
    }

    // Handle incoming text message responses:
    if (response.cmd == CustomCommand::logMessage) {
      std::string receivedText;
      receivedText.reserve(sizeof(response.customData) + 2);
      memcpy((void *) receivedText.data(), (void *) &response.customData[0], sizeof(response.customData));
      textOutput.append(receivedText.c_str());
    }

  }

  // Function to send data to the robot.
  // It is called in the ros2_control hardware interface function "CustomHardware::write" to write Setpoints:
  bool sendData() {
    request.messageNumber++;

    // Copy the axis setpoints to the request message:
    for (auto i = 0; i < hw_cmd_axisSetpoints.size(); i++) {
      request.jointSetpoints[i] = (int32_t) (hw_cmd_axisSetpoints[i] * 57295.7795131);    //57295.7795131 = 1000 * 180/pi
    } 

    // Copy the digital output setpoints to the request message:
    for (auto i = 0; i < hw_cmd_digitalOutputs.size(); i++) {
      request.digitalOutputs[i] = (bool) hw_cmd_digitalOutputs[i];
    }

    // Check if a new text command message has been received and 
    if (newMessageRevceived_) {
      request.cmd = CustomCommand::handleString;

      // Enable or disable the emergency stop flag:
      if (newMessage_ == "emergency_stop\n") {
        request.emergencyStop = true;
        newMessageRevceived_ = false;
        newMessage_ = "";
      } else if (newMessage_ == "clear_emergency_stop\n") {
        request.emergencyStop = false;
        newMessageRevceived_ = false;
        newMessage_ = "";
        
      // A regular command has been received.
      } else {
        memcpy((void *) &request.customData[0], newMessage_.data(), sizeof(request.customData) - 1);
        request.customData[sizeof(request.customData) - 1] = NULL;

        // If the message is short enough to fit in the data buffer, flag the message received as false.
        if (newMessage_.size() < sizeof(request.customData) - 1) {  
          newMessageRevceived_ = false;
          newMessage_ = "";

        // If the message is too large for the data buffer, remove the data being sent in this cycle and send the rest in the next cycle(s):
        } else {
          newMessage_.erase(0, sizeof(request.customData) - 1);
        }
      }
    }

    // Start the communication delay measurement and send the data to the robot:
    sendTime = MyUtils::micros();
    bytesSent = send(socketConnection_, &request, sizeof(request), 0);

    // Reset some flags:
    request.cmd = CustomCommand::none;

    // Raise an error if no bytes have been sent:
    if (bytesSent == -1) {
      std::cerr << "Error sending Message";
      return false;
    }
    return true;
  }

  // Connect to the robot hardware on the given IP Address. 
  // If the robot connects directly to the PC, the ssid of the hotspot should be passed to check if it's up and running.
  // It is called in the ros2_control hardware interface function "CustomHardware::on_configure"
  bool initialize(std::string ipAddress, std::string ssid, std::string & errorMessage) {
    // If required, check if the hotspot connection is active:
    if (ssid != "" && MyUtils::hotspotIsActive(ssid, errorMessage) == false) return false;

    // Check if the robot can be pinged:
    if (MyUtils::checkConnection(ipAddress, errorMessage) == false) return false;

    // Create a socket handle:
    socketConnection_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socketConnection_ == -1) { std::cerr << "Error creating socket" << std::endl; return false; }   

    // Setup the network connection parameters:
    struct sockaddr_in receiverAddr;
    receiverAddr.sin_family = AF_INET;
    receiverAddr.sin_port = htons(80);
    receiverAddr.sin_addr.s_addr = inet_addr(ipAddress.c_str());
    
    request.messageNumber = 0;

    // Try to connect to the robot:
    if (connect(socketConnection_, (struct sockaddr*) &receiverAddr, sizeof(receiverAddr)) == -1) {
      std::cerr << "Error connecting to the Robot" << std::endl;
      return false;
    }
    return true;
  };
};

#endif