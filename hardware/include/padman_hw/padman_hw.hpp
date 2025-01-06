// Copyright 2020 ros2_control Development Team
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

#ifndef PADMAN_HW_HPP_
#define PADMAN_HW_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ros2_socketcan/socket_can_common.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

namespace padman_hw
{

enum MSG_IDS_REL{
    CMD = 0,
    STATE = 1,
    TARGET_TORQUE = 2,
    TARGET_POSITION = 3,
    STATE_POSITION = 4,
    STATE_VELOCITY = 5,
    STATE_EFFORT = 6};

enum CMD_IDS{
    REQ_STATUS=1,
    INIT_FOC=2,
    FIND_JOINTLIMITS=3,
    CMD_CTRL_TORQUE=4,
    CMD_CTRL_POSITION=5,
    REBOOT=6
};

enum STATES{
    UNINITIALIZED=0,
    INITIALIZED_FOC=1,
    FIND_LIMIT_LOWER=2,
    FIND_LIMIT_UPPER=3,
    INITIALIZED_JOINT=4,
    CTRL_TORQUE=5,
    CTRL_POSITION=6
};

const int ID_RANGE = 100;

using TimePoint = std::chrono::system_clock::time_point;
using MatrixTimePoint = Eigen::Matrix<TimePoint, Eigen::Dynamic, Eigen::Dynamic>;

class PadmanSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PadmanSystemPositionOnlyHardware);

  PadmanSystemPositionOnlyHardware();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void can_receive();

protected:
  hardware_interface::CallbackReturn canbus_init();
  hardware_interface::return_type request_and_wait_jointstate();
  hardware_interface::CallbackReturn canbus_reset_joints();
  hardware_interface::CallbackReturn canbus_init_joint_foc(int id);
  hardware_interface::CallbackReturn canbus_init_jointlimits(int id);
  hardware_interface::CallbackReturn canbus_activate_positionctrl(int id);

private:
  
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // canbus
  std::string can_interface_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender_;
  std::unique_ptr<std::thread> can_receiver_thread_;
  std::chrono::nanoseconds can_interval_ns_;
  std::chrono::nanoseconds can_timeout_ns_;
  bool can_enable_fd_;
  bool can_use_bus_time_;

  MatrixTimePoint msg_timestamps; // first dimension joint, second dimension message type
  std::vector<STATES> joint_state;
};

}  // namespace ros2_control_demo_example_1

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
