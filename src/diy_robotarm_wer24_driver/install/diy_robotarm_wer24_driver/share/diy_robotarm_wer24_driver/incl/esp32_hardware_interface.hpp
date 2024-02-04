#ifndef ESP32_HARDWARE_INTERFACE_HPP_
#define ESP32_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

//https://docs.ros.org/en/ros2_packages/rolling/api/io_context/generated/program_listing_file__tmp_ws_src_transport_drivers_io_context_include_msg_converters_std_msgs.hpp.html
#include <unistd.h>
#include <pthread.h>
#include <functional>
#include <sstream>
#include <iomanip>


#include "visibility_control.h"
#include "RobotConnection.hpp"
using namespace std::chrono_literals;

RobotConnection * pRobot;

class RosPublishers : public rclcpp::Node {
public:
  RosPublishers(RobotConnection * pRobot) : Node("text_publisher"), count_(0) {
    statusPublisher_ = this->create_publisher<std_msgs::msg::String>("custom_hardware/status", 10);
    digitalInputPub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("custom_hardware/digital_inputs/read", 10);
    digitalOutputPub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("custom_hardware/digital_outputs/read", 10);
    textOutputPub_ = this->create_publisher<std_msgs::msg::String>("custom_hardware/command/output", 10);

    pRobot_ = pRobot;
    timer_ = this->create_wall_timer(20ms, std::bind(&RosPublishers::timer_callback, this) );
  }
  
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr digitalInputPub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr digitalOutputPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr textOutputPub_;

  void timer_callback() {
    auto inputStates = std_msgs::msg::UInt8MultiArray();
    auto outputStates = std_msgs::msg::UInt8MultiArray();
    auto statusMessage = std_msgs::msg::String();
    auto textMessage = std_msgs::msg::String();
    std::vector<std::string> newMessages;

    pRobot_->getDataToPublish(inputStates.data, outputStates.data, statusMessage.data, newMessages);
    
    digitalInputPub_->publish(inputStates);
    digitalOutputPub_->publish(outputStates);
    statusPublisher_->publish(statusMessage);

    // if (newMessages.size() != 0) {
    //   // textMessage.data = "Timer callback: Received some messages";
    //   textOutputPub_->publish(textMessage);
    // }
    
    for (std::string message : newMessages) {
      textMessage.data = message;
      textOutputPub_->publish(textMessage);
    }
  }

  RobotConnection * pRobot_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  size_t count_;
};
// IOs kann bestimmt weg
class RosSubscribers : public rclcpp::Node {
public:
  RosSubscribers(RobotConnection * pRobot) : Node("text_subscriber") {
    // digitalOutputSub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    //   "custom_hardware/digital_outputs/write", 
    //   10,
    //   std::bind(&RosSubscribers::digital_output_write_callback, this, std::placeholders::_1)
    // );

    customCommandSub_ = this->create_subscription<std_msgs::msg::String>(
      "custom_hardware/command/input",
      10,
      std::bind(&RosSubscribers::custom_command_callback, this, std::placeholders::_1)
    );
    pRobot_ = pRobot;
  }

private:
  void custom_command_callback(const std_msgs::msg::String::SharedPtr msg) const {
    pRobot_->messageHandler(msg->data);
  };

  void digital_output_write_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const {
    pRobot_->setDigitalOutputs(msg->data);
  }

  RobotConnection * pRobot_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr customCommandSub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr digitalOutputSub_;
};


void * subscriberThread(void * robot) {
  rclcpp::spin(std::make_shared<RosSubscribers>((RobotConnection *) robot));
};


void * publisherThread(void * robot) {
  rclcpp::spin(std::make_shared<RosPublishers>((RobotConnection *) robot));
};


// System interface, alles andere nicht muss (Namen gut benennen)
namespace custom_hardware {
  class CustomHardware :  public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CustomHardware)

    CUSTOM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    CUSTOM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const   rclcpp_lifecycle::State   & previous_state) override;

    CUSTOM_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CUSTOM_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CUSTOM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CUSTOM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CUSTOM_HARDWARE_PUBLIC
    hardware_interface::return_type read(const  rclcpp::Time & time, const rclcpp::Duration & period) override;

    CUSTOM_HARDWARE_PUBLIC
    hardware_interface::return_type write(const  rclcpp::Time & time, const rclcpp::Duration & period) override;
    

    // Initialization function
    void initSubscribersAndPublishers() {
      pthread_create(&textThread_, NULL, subscriberThread, (void *) &robotConnection);
      pthread_create(&textPubThread_, NULL, publisherThread, (void *) &robotConnection);
    }; 

    
  private:
    int counter_;

    RobotConnection robotConnection;    
    pthread_t textThread_;
    pthread_t textPubThread_;
    pid_t pid;
  };
}

#endif