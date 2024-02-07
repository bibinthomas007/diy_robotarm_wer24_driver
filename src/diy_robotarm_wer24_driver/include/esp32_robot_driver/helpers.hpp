#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <chrono>


#ifndef HELPERS_HPP_
#define HELPERS_HPP_

namespace MyUtils {
  //https://stackoverflow.com/a/478960

  // Function to execute a bash command:
  std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) { throw std::runtime_error("popen() failed!"); }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) { result += buffer.data(); }
    return result;
  }

  // Get the current time in microseconds
  uint64_t micros(){
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  } 

  // Try to ping the device specified by the IP address
  bool checkConnection(std::string ipAddress, std::string & errorMessage) {
    std::string command = "ping -c 1 -w 1 " + ipAddress;
    std::string pingResult = exec(command.c_str());
    if (pingResult.find("0 received") != std::string::npos) {
      errorMessage += "Device with IP-Address \"" + ipAddress + "\" cannot be reached. Please ensure the device is connected to the network and retry!";
      return false;
    } else return true;
  }

  // Check if the connection is currently active on the PC
  bool hotspotIsActive(std::string connectionName, std::string & errorMessage) { //DIY_Robotics_2 
    std::string command = "nmcli -f GENERAL.STATE con show " + connectionName;
    std::string networkActivationResult = MyUtils::exec(command.c_str());

    if (networkActivationResult.find("activated") == std::string::npos) {
      errorMessage += "Connection \"" + connectionName + "\" is not active. Use \"sudo nmcli connection up " + connectionName + "\" to enable the hotspot";
      return false;
    } else return true;
  }
}

#endif