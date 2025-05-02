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

#ifndef ROS2_CONTROL_DEMO_BICYCLEDRIVEBOT_CARLIKE__CARLIKEBOT_ACTUATOR_OVER_SOCKET_HPP_
#define ROS2_CONTROL_DEMO_BICYCLEDRIVEBOT_CARLIKE__CARLIKEBOT_ACTUATOR_OVER_SOCKET_HPP_

#include <netinet/in.h>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace ros2_control_demo_bicycledrivebot_carlike
{
class CarlikeBotActuatorHardwareOverSocket : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotActuatorHardwareOverSocket);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the CarlikeBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // joint names
  std::string steering_joint_;
  std::string traction_joint_;

  // Fake "mechanical connection" between actuator and sensor using sockets
  struct sockaddr_in address_;
  uint16_t socket_port_;
  int sockoptval_ = 1;
  int sock_;
};

}  // namespace ros2_control_demo_bicycledrivebot_carlike

#endif  // ROS2_CONTROL_DEMO_BICYCLEDRIVEBOT_CARLIKE__CARLIKEBOT_ACTUATOR_OVER_SOCKET_HPP_