// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors:
// - Denis Stogl (example_14: RRBotActuatorWithoutFeedback)
// - Ahsan Yusob (adapted for CarlikeBot + socket)
//

#include "ros2_control_demo_bicycledrivebot_carlike/carlikebot_actuator_over_socket.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_bicycledrivebot_carlike
{
hardware_interface::CallbackReturn CarlikeBotActuatorHardwareOverSocket::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  socket_port_ =
    static_cast<uint16_t>(std::stoi(info_.hardware_parameters["example_param_socket_port"]));
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // CarlikeBotActuatorHardwareOverSocket has exactly two command interface and two joint
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface
    if (joint_is_steering)
    {
      steering_joint_ = joint.name;
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());
      traction_joint_ = joint.name;

      // Drive joints have a velocity command interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // Initialize objects for fake mechanical connection
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Creating socket failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto server = gethostbyname("localhost");

  address_.sin_family = AF_INET;
  bcopy(
    reinterpret_cast<char *>(server->h_addr), reinterpret_cast<char *>(&address_.sin_addr.s_addr),
    server->h_length);
  address_.sin_port = htons(socket_port_);

  const int max_retries = 5;
  const int initial_delay_ms = 1000;  // Initial delay of 1 second

  RCLCPP_INFO(get_logger(), "Trying to connect to port %d.", socket_port_);

  int retries = 0;
  int delay_ms = initial_delay_ms;
  bool connected = false;

  while (retries < max_retries)
  {
    if (connect(sock_, (struct sockaddr *)&address_, sizeof(address_)) == 0)
    {
      connected = true;
      break;
    }

    RCLCPP_WARN(
      get_logger(), "Connection attempt %d failed: %s. Retrying in %d ms...", retries + 1,
      strerror(errno), delay_ms);

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    delay_ms *= 2;  // Exponential backoff
    retries++;
  }

  if (!connected)
  {
    RCLCPP_FATAL(
      get_logger(), "Connection over socket failed after %d attempts: %s", retries,
      strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Successfully connected to port %d.", socket_port_);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotActuatorHardwareOverSocket::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotActuatorHardwareOverSocket::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  shutdown(sock_, SHUT_RDWR);  // shutdown socket

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotActuatorHardwareOverSocket::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values for joints
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotActuatorHardwareOverSocket::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

//nothing to read.
hardware_interface::return_type CarlikeBotActuatorHardwareOverSocket::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_bicycledrivebot_carlike::CarlikeBotActuatorHardwareOverSocket::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  double front_wheel_position_command_;
  double rear_wheel_velocity_command_;
  uint8_t buffer_[sizeof(double) * 2]; //TODO: replace with sensor measurement data (front_wheel_steer, rear_lon_vel)

  ss << "Writing..." << std::endl;
  ss << std::fixed << std::setprecision(2);

  auto steering_name_ = steering_joint_ + "/" + hardware_interface::HW_IF_POSITION;
  ss << "Writing steering command: " << get_command(steering_name_) << std::endl;
  front_wheel_position_command_ = get_command(steering_name_);
  
  auto traction_name_ = traction_joint_ + "/" + hardware_interface::HW_IF_VELOCITY;
  ss << "Writing traction command: " << get_command(traction_name_) << std::endl;
  rear_wheel_velocity_command_ = get_command(traction_name_);

  ss << "Sending steering data command: " << front_wheel_position_command_ << std::endl;
  ss << "    and traction data command: " << rear_wheel_velocity_command_ << std::endl;
  RCLCPP_INFO(get_logger(), ss.str().c_str());
  std::memcpy(&buffer_[0], &front_wheel_position_command_, sizeof(double));
  std::memcpy(&buffer_[sizeof(double)], &rear_wheel_velocity_command_, sizeof(double));

  // Simulate sending commands to the hardware
  send(sock_, buffer_, sizeof(buffer_), 0);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_bicycledrivebot_carlike

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_bicycledrivebot_carlike::CarlikeBotActuatorHardwareOverSocket, hardware_interface::ActuatorInterface)