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
// - Denis Stogl (example_14: RRBotSensorPositionFeedback)
// - Ahsan Yusob (adapted for CarlikeBotSensor + socket)
//

#include "ros2_control_demo_bicycledrivebot_carlike/carlikebot_sensor_over_socket.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_bicycledrivebot_carlike
{
hardware_interface::CallbackReturn CarlikeBotSensorHardwareOverSocket::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = hardware_interface::stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  socket_port_ =
    static_cast<uint16_t>(std::stoi(info_.hardware_parameters["example_param_socket_port"]));
  
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position state interface
    if (joint_is_steering)
    {
      steering_joint_ = joint.name;
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());
      traction_joint_ = joint.name;

      // Drive joints have a velocity state interface and a position state interface

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }


  clock_ = rclcpp::Clock();

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  // Initialize objects for fake mechanical connection
  obj_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (obj_socket_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Creating socket failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Setting socket options.");
  if (setsockopt(obj_socket_, SOL_SOCKET, SO_REUSEADDR, &sockoptval_, sizeof(sockoptval_)))
  {
    RCLCPP_FATAL(get_logger(), "Setting socket failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  address_length_ = sizeof(address_);

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(socket_port_);

  RCLCPP_INFO(get_logger(), "Binding to socket address.");
  if (bind(obj_socket_, reinterpret_cast<struct sockaddr *>(&address_), sizeof(address_)) < 0)
  {
    RCLCPP_FATAL(
      get_logger(), "Binding to socket failed: %s",
      strerror(errno));  // Print the error message
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Storage and Thread for incoming data
  std::array<uint8_t, 16> initial_data{};
  rt_incomming_data_ptr_.writeFromNonRT(initial_data);

  incoming_data_thread_ = std::thread(
    [this]()
    {
      // Await and accept connection
      RCLCPP_INFO(get_logger(), "Listening for connection on port %d.", socket_port_);
      if (listen(obj_socket_, 1) < 0)
      {
        RCLCPP_FATAL(get_logger(), "Cannot listen from the server.");
        return hardware_interface::CallbackReturn::ERROR;
      }

      sock_ = accept(
        obj_socket_, reinterpret_cast<struct sockaddr *>(&address_),
        reinterpret_cast<socklen_t *>(&address_length_));
      if (sock_ < 0)
      {
        RCLCPP_FATAL(get_logger(), "Cannot accept on the server.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_logger(), "Accepting on socket.");

      int incoming_data_read_rate = 1000;  // Hz
      RCLCPP_INFO(
        get_logger(),
        "Creating thread for incoming data and read them with %d Hz to not miss any data.",
        incoming_data_read_rate);

      // Variables for reading from a socket
      const size_t reading_size_bytes = 1024;
      char buffer[reading_size_bytes] = {0};

      // Use nanoseconds to avoid chrono's rounding
      std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000 / incoming_data_read_rate));

      RCLCPP_INFO(get_logger(), "Receiving data");
      while (rclcpp::ok() & is_running_)
      {
        if (recv(sock_, buffer, reading_size_bytes, 0) > 0)
        {
          std::array<uint8_t, 16> data_buffer{};
          std::memcpy(data_buffer.data(), buffer, 16);
          std::stringstream hex_stream;
          hex_stream << "Buffer: ";
          for (unsigned char c : data_buffer) {
            hex_stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
          }
          RCLCPP_INFO(get_logger(), "%s", hex_stream.str().c_str());
          rt_incomming_data_ptr_.writeFromNonRT(data_buffer);
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Data not yet received from socket.");
          std::array<uint8_t, 16> initial_data{};
          rt_incomming_data_ptr_.writeFromNonRT(initial_data);
        }

        bzero(buffer, reading_size_bytes);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000 / incoming_data_read_rate));
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    });
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSensorHardwareOverSocket::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // set some default values for joints
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  last_measured_velocity_ = 0;
  last_measured_position_ = 0;

  // In general after a hardware is configured it can be read
  last_timestamp_ = clock_.now();

  RCLCPP_INFO(get_logger(), "Configuration successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSensorHardwareOverSocket::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  incoming_data_thread_.join();      // stop reading thread
  shutdown(sock_, SHUT_RDWR);        // shutdown socket
  shutdown(obj_socket_, SHUT_RDWR);  // shutdown socket

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSensorHardwareOverSocket::on_activate(
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

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSensorHardwareOverSocket::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Flag to stop listening to socket
  is_running_ = false;
  
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

hardware_interface::return_type CarlikeBotSensorHardwareOverSocket::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading..." << std::endl;

  // Sensor reading
  command_data_received = *(rt_incomming_data_ptr_.readFromRT());
  double pos;
  std::memcpy(&pos, &command_data_received[0], sizeof(double));
  if (!std::isnan(pos))
  {
    std::memcpy(&measured_position, &command_data_received[0], sizeof(double));
    std::memcpy(&measured_velocity, &command_data_received[sizeof(double)], sizeof(double));
    last_measured_position_ = measured_position;
    last_measured_velocity_ = measured_velocity;
  }

  // integrate wheel velocity to position (example)
  const auto steering_position_name_ = steering_joint_ + "/" + hardware_interface::HW_IF_POSITION;
  const auto traction_velocity_name_ = traction_joint_ + "/" + hardware_interface::HW_IF_VELOCITY;
  const auto traction_position_name_ = traction_joint_ + "/" + hardware_interface::HW_IF_POSITION;
  
  ss << std::fixed << std::setprecision(2);
  ss << "Example: Deriving joint states directly from joint command.." << std::endl;
  
  set_state(steering_position_name_, measured_position);
  ss << "Got state(steering_position) " << measured_position << " for joint '" << steering_position_name_ << "'" << std::endl;
  
  set_state(traction_velocity_name_, measured_velocity);
  ss << "Got state(traction_velocity) " << measured_velocity << " for joint '" << traction_velocity_name_ << "'" << std::endl;
  
  const auto traction_position_data =
    get_state(traction_position_name_) + (last_measured_velocity_ * duration.seconds()) / hw_slowdown_;
  set_state(traction_position_name_, traction_position_data);
  ss << "Got state(traction_position) " << traction_position_data << " for joint '" << traction_position_name_ << "'" << std::endl;
  
  RCLCPP_INFO(get_logger(), ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_bicycledrivebot_carlike

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_bicycledrivebot_carlike::CarlikeBotSensorHardwareOverSocket, hardware_interface::SensorInterface)