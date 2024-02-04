#ifndef DATAFORMAT_HPP
#define DATAFORMAT_HPP

#include <stdint.h>

#ifdef __linux__ 
#elif ESP32
#endif

// Custom Command Headers
enum class CustomCommand : uint8_t {
  none = 0,
  handleString,
  setRgbLed,
  logMessage,
  logWarning,
  logError,
  someOtherCommand
};

// Error Codes
enum class ErrorCode : uint8_t {
  noError = 0,
  errorA,
  errorB,
  errorC
};

namespace Communication {
  // Structured Binary Data that ROS sends to the Robot
  // (Setpoints, etc...)
  // ROS2 Package "custom_hardware"
  union PcToRobot_t {
    struct __attribute__((packed)) {
      bool enablePower;
      uint8_t messageNumber;
      bool emergencyStop;
      uint8_t reserved[1];

      int32_t jointSetpoints[6];
      bool digitalOutputs[8];

      enum CustomCommand cmd;
      uint8_t customData[27];
    };

    uint8_t receiveBuffer[64];
  };// request;

  // Structured Binary Data that the Robot returns to ROS
  // (Measurements, etc...)
  union RobotToPc_t {
    struct __attribute__((packed)) {
      bool active;
      uint8_t messageNumber;
      enum ErrorCode errorCode;
      uint8_t reserved[1];

      int32_t jointPositions[6];
      bool digitalInputs[8];

      enum CustomCommand cmd;
      uint8_t customData[27];
    };
    uint8_t sendBuffer[64];
  };// response;
}

#endif
