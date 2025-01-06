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

#include "padman_hw/padman_hw.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
// Missing header for ioctl



// // includes for CAN bus
// #include <iostream>
// #include <cstring>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <sys/socket.h>
// #include <net/if.h>
// #include <unistd.h>
// #include <sys/ioctl.h> 
// #include <CanDriver.hpp>
// // end includes for CAN bus


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"



using namespace std::chrono;

namespace padman_hw
{

PadmanSystemPositionOnlyHardware::PadmanSystemPositionOnlyHardware(){
  can_interface_ = "can0";//this->declare_parameter("interface", "can0");
  can_use_bus_time_ = false; //this->declare_parameter<bool>("use_bus_time", false);
  can_enable_fd_ = false; //this->declare_parameter<bool>("enable_can_fd", false);
  double timeout_sec = 0.01; //this->declare_parameter("timeout_sec", 0.01);
  can_timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));
}


hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  msg_timestamps = MatrixTimePoint(3, MSG_IDS_REL::STATE_EFFORT+1); // magic numbers to be removed: n_joints x n_messages per joint. set all to -1 as uninitialized
  joint_state =  std::vector<STATES>(3, STATES::UNINITIALIZED);
    // Initialize all elements to default-constructed time_point
  msg_timestamps.setConstant(TimePoint{});
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
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
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

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
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::canbus_init(){
RCLCPP_DEBUG(this->get_logger(), "------------------------------- canbus_init.");
RCLCPP_INFO(this->get_logger(), "------------------------------- canbus_init.");
  // Create CAN sender

    try {
    can_sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface_, can_enable_fd_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening CAN sender: %s - %s",
      can_interface_.c_str(), ex.what());
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Sender successfully configured.");

  // Create CAN receiver

  try {
    RCLCPP_INFO(get_logger(), "creating can receiver");
    can_receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface_, can_enable_fd_);
    // apply CAN filters
    //auto filters = get_parameter("filters").as_string();
    //can_receiver_->SetCanFilters(drivers::socketcan::SocketCanReceiver::CanFilterList(filters));
    RCLCPP_INFO(get_logger(), "created receiver");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening CAN receiver: %s - %s",
      can_interface_.c_str(), ex.what());
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Receiver successfully configured.");

  can_receiver_thread_ = std::make_unique<std::thread>(&PadmanSystemPositionOnlyHardware::can_receive, this);



  RCLCPP_INFO(get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    // RCLCPP_INFO(get_logger(), "Initializing CAN bus");
    // const char *can_interface = "can0";

    // struct sockaddr_can addr;
    // struct ifreq ifr;

    // // Create a socket
    // sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // if (sock < 0) {
    //     perror("Socket");
    //     return 1;
    // }

    // // Specify the CAN interface
    // strncpy(ifr.ifr_name, can_interface, IFNAMSIZ - 1);
    // if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    //     perror("ioctl");
    //     close(sock);
    //     return 1;
    // }

    // // Bind the socket to the CAN interface
    // addr.can_family = AF_CAN;
    // addr.can_ifindex = ifr.ifr_ifindex;
    // if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    //     perror("Bind");
    //     close(sock);
    //     return 1;
    // }
    // return 0;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::canbus_reset_joints(){
  // // python
  //   //   reset_msg = can.Message(
  //   //     arbitration_id=0x00, data=[0, 0, 0, 0, 0, 0], is_extended_id=False
  //   // )




 //if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
    int can_id = 0;
    drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
    size_t dlc = 6;
    uint8_t data[6] = {0,0,0,0,0,0};

    try {
      can_sender_->send(data, dlc, send_id, can_timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        can_interface_.c_str(), ex.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  //}




  //   int can_id = 0;
  //   RCLCPP_INFO(get_logger(), "Resetting joints");
  //   // Set up a CAN frame to send
  //   struct can_frame frame;
  //   frame.can_id = can_id;//0x123;             // 11-bit standard CAN ID
  //   frame.can_dlc = 6;                // Data length code (4 bytes)
  //   frame.data[0] = 0;
  //   frame.data[1] = 0;
  //   frame.data[2] = 0;
  //   frame.data[3] = 0;
  //   frame.data[4] = 0;
  //   frame.data[5] = 0;

  //   // Send the CAN frame
  //   if (::write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
  //       perror("Write");
  //       close(sock);
  //       return 1;
  //   }
  //   RCLCPP_INFO(get_logger(), "CAN frame sent!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::canbus_init_joint_foc(int id){
  //   // python:
  //   //    data = [np.uint8(CMD_IDS.INIT_FOC)]#[0x00, 0x00]
  //   //    message = can.Message(arbitration_id=MSG_IDS_REL.CMD+(self.id+1)*ID_RANGE, is_extended_id=False, data=data)
  


    int can_id = MSG_IDS_REL::CMD+(id+1)*ID_RANGE;
    drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
    drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
    size_t dlc = 1;
    uint8_t data[1] = {CMD_IDS::INIT_FOC};

    try {
      can_sender_->send(data, dlc, send_id, can_timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        can_interface_.c_str(), ex.what());
      return hardware_interface::CallbackReturn::ERROR;
    }


  //   int id_range=100;
  //   int can_id = MSG_IDS_REL::CMD+(id+1)*id_range;
  //   RCLCPP_INFO(get_logger(), "Initializing FOC for joint %i", id);
  //   // Set up a CAN frame to send
  //   struct can_frame frame;
  //   frame.can_id = can_id;//0x123;             // 11-bit standard CAN ID
  //   frame.can_dlc = 1;                // Data length code (4 bytes)
  //   frame.data[0] = CMD_IDS::INIT_FOC;

  //   // Send the CAN frame
  //   if (::write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
  //       perror("Write");
  //       close(sock);
  //       return 1;
  //   }
  //   RCLCPP_INFO(get_logger(), "CAN frame sent!");
  // return 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::canbus_init_jointlimits(int id){



  //   // python:
  //       // data = [np.uint8(CMD_IDS.FIND_JOINTLIMITS)]#[0x00, 0x00]
  //       // message = can.Message(arbitration_id=MSG_IDS_REL.CMD+(self.id+1)*ID_RANGE, is_extended_id=False, data=data)
  //       // self.bus.send(message, timeout=0.2)



    int can_id = MSG_IDS_REL::CMD+(id+1)*ID_RANGE;
    drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
    drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
    size_t dlc = 1;
    uint8_t data[1] = {CMD_IDS::FIND_JOINTLIMITS};

    try {
      can_sender_->send(data, dlc, send_id, can_timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        can_interface_.c_str(), ex.what());
      return hardware_interface::CallbackReturn::ERROR;
    }


  //   int id_range=100;
  //   int can_id = MSG_IDS_REL::CMD+(id+1)*ID_RANGE;
  //   RCLCPP_INFO(get_logger(), "Initializing FOC for joint %i", id);
  //   // Set up a CAN frame to send
  //   struct can_frame frame;
  //   frame.can_id = can_id;//0x123;             // 11-bit standard CAN ID
  //   frame.can_dlc = 1;                // Data length code (4 bytes)
  //   frame.data[0] = CMD_IDS::FIND_JOINTLIMITS;

  //   // Send the CAN frame
  //   if (::write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
  //       perror("Write");
  //       close(sock);
  //       return 1;
  //   }
  //   RCLCPP_INFO(get_logger(), "CAN frame sent!");
  // return 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}










hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // for (int i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  // }
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }



canbus_init();



  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }



rclcpp::sleep_for(std::chrono::seconds(1));
  canbus_reset_joints();
rclcpp::sleep_for(std::chrono::seconds(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  request_and_wait_jointstate();


  // see if the joints have been initialized before, and if yes, then skip finding joint limits
  bool is_need_initialization = false;
  for(int i_joint=0; i_joint<3; i_joint++){
    is_need_initialization = is_need_initialization || joint_state[i_joint]<STATES::INITIALIZED_JOINT;
  }

  if(is_need_initialization){

  
  
  canbus_init_joint_foc(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  canbus_init_joint_foc(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  canbus_init_joint_foc(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  for (int i = 0; i < 5; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  canbus_init_jointlimits(0);
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  canbus_init_jointlimits(2);

  for (int i = 0; i < 10; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  canbus_init_jointlimits(1);

  for (int i = 0; i < 10; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  } else{
    RCLCPP_INFO(get_logger(), "Skipping initialization!");
  }

  for (int i = 0; i < 3; i++)
  {
    
    RCLCPP_INFO(get_logger(), "Activating Position CTRL for joint %d", i);
    canbus_activate_positionctrl(i);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PadmanSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
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

hardware_interface::return_type PadmanSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // what we're doing here is certainly not the fastest way to obtain joint states, but it's a simple and 
  // controllable way: when read() is called we send a request for positions on canbus and wait for the answers.
  // until we received all we stay in this function.
  // to be optimized...
  

  // broadcast request for joint positions (currently joint i sends its position when it received the position of joint i-1 -> we trigger
  // the cascade by sending pseudo position for non-existant joint 0)
  int can_id = MSG_IDS_REL::STATE_POSITION;
  drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
  drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
  size_t dlc = 1;
  uint8_t data[1] = {0};

  try {
    can_sender_->send(data, dlc, send_id, can_timeout_ns_);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Error sending CAN message: %s - %s",
      can_interface_.c_str(), ex.what());
    return hardware_interface::return_type::ERROR;
  }

  // wait until we got all the positions



  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // std::stringstream ss;
  // ss << "Reading states:";

  // for (const auto & [name, descr] : joint_state_interfaces_)
  // {
  //   // Simulate RRBot's movement
  //   auto new_value = get_state(name) + (get_command(name) - get_state(name)) / hw_slowdown_;
  //   set_state(name, new_value);
  //   ss << std::fixed << std::setprecision(2) << std::endl
  //      << "\t" << get_state(name) << " for joint '" << name << "'";
  // }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PadmanSystemPositionOnlyHardware::request_and_wait_jointstate(){
  for(int i_joint=0; i_joint<3; i_joint++){
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "LOOPING %d",i_joint);
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::duration<float> t_since_last = now - msg_timestamps(i_joint, MSG_IDS_REL::STATE);
    std::stringstream ss;
    ss << "Time since last STATUS for joint "<<i_joint<<":"<<t_since_last.count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    int can_id = (i_joint+1)*ID_RANGE + MSG_IDS_REL::CMD;
    drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
    drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
    size_t dlc = 1;
    uint8_t data[1] = {CMD_IDS::REQ_STATUS};
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "SENDING");
    try {
      can_sender_->send(data, dlc, send_id, can_timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        can_interface_.c_str(), ex.what());
      return hardware_interface::return_type::ERROR;
    }
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "SENT");

    while(std::chrono::system_clock::now() - msg_timestamps(i_joint, MSG_IDS_REL::STATE) > std::chrono::seconds(1) && rclcpp::ok()){
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Waiting for joint state");

    now = std::chrono::system_clock::now();
        // Convert time_point to time_t (Unix timestamp)
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::time_t msg_time_t = std::chrono::system_clock::to_time_t(msg_timestamps(i_joint, MSG_IDS_REL::STATE));
    ss << "Time since last STATUS for joint \t"<<i_joint<<":"<<(std::chrono::system_clock::now() - msg_timestamps(i_joint, MSG_IDS_REL::STATE)).count()<<
          " now: \t"<< std::ctime(&now_time_t) << 
          "timestamp:\t"<<std::ctime(&msg_time_t);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", ss.str().c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));
    }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Received Jointstate");

  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "LOOP END");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PadmanSystemPositionOnlyHardware::canbus_activate_positionctrl(int i_joint){
  // PYTHON
        //   data = [np.uint8(CMD_IDS.CTRL_POSITION)]#[0x00, 0x00]
        // message = can.Message(arbitration_id=MSG_IDS_REL.CMD+(self.id+1)*self.id_range, is_extended_id=False, data=data)
    int can_id = (i_joint+1)*ID_RANGE + MSG_IDS_REL::CMD;
    drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
    drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
    size_t dlc = 1;
    uint8_t data[1] = {CMD_IDS::CMD_CTRL_POSITION};
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "SENDING");
    try {
      can_sender_->send(data, dlc, send_id, can_timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        can_interface_.c_str(), ex.what());
      return hardware_interface::return_type::ERROR;
    }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PadmanSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // std::stringstream ss;
  // ss << "Writing commands:";

  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   // Simulate sending commands to the hardware
  //   ss << std::fixed << std::setprecision(2) << std::endl
  //      << "\t" << get_command(name) << " for joint '" << name << "'";
  // }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (int i_joint=0; i_joint<3; i_joint++)
  {
    std::string name = "joint"+std::to_string(i_joint+1)+"/position";
    float target = get_command(name);
    canbus_send_targetposition(i_joint, target);
  }

  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PadmanSystemPositionOnlyHardware::canbus_send_targetposition(int i_joint, float target){
  // PYTHON
        // self.target_data = [0x00, 0x00, 0x00, 0x00]
        // self.target_msg = can.Message(arbitration_id=self.joints[0].id_range+MSG_IDS_REL.TARGET_POSITION, is_extended_id=False, data=self.target_data)

        // # Start periodic CAN sender
        // self.sender = PeriodicCANSender(bus, ids = [self.joints[i].id_range*(i+1)++MSG_IDS_REL.TARGET_POSITION for i in range(self.n_joints)], datas = [self.target_data]*self.n_joints, interval=0.01)  # 20 Hz
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "CREATING MSG");
  int can_id = (i_joint+1)*ID_RANGE + MSG_IDS_REL::TARGET_POSITION;
  drivers::socketcan::FrameType type = drivers::socketcan::FrameType::DATA;// also possible: FrameType::REMOTE;FrameType::ERROR;
  drivers::socketcan::CanId send_id(can_id, 0, type, drivers::socketcan::StandardFrame);
  size_t dlc = 4;
  uint8_t data[4];

  // data = reinterpret_cast<uint8_t*>(&target);
  std::memcpy(data, reinterpret_cast<uint8_t*>(&target), sizeof(data));

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "SENDING");
  try {
    can_sender_->send(data, dlc, send_id, can_timeout_ns_);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Error sending CAN message: %s - %s",
      can_interface_.c_str(), ex.what());
    return hardware_interface::return_type::ERROR;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "SENT");
  return hardware_interface::return_type::OK;
}

void PadmanSystemPositionOnlyHardware::can_receive()
{
  drivers::socketcan::CanId receive_id{};

  // if (!enable_fd_) {

    while (rclcpp::ok()) {
      // RCLCPP_INFO(get_logger(), "LOOP CANBUS MSG");
      // if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      //   //std::this_thread::sleep_for(100ms); //chrono literal in cpp14
      //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
      //   continue;
      // }
      unsigned int data_buffer;
      try {
        
        //RCLCPP_INFO(this->get_logger(), "Trying to receive with databuffer of size: %i", sizeof(data_buffer));
        receive_id = can_receiver_->receive(&data_buffer, can_interval_ns_);
        //RCLCPP_INFO(this->get_logger(), "Received msg w id: %i", receive_id);
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Error receiving CAN message: %s - %s",
          can_interface_.c_str(), ex.what());
        continue;
      }

      // switch over message types:
      int i_joint = receive_id.identifier() / ID_RANGE -1 ;
      int i_cmd = receive_id.identifier() % ID_RANGE;

      //RCLCPP_INFO(get_logger(), "RECEIVED CANBUS MSG");

      switch(i_cmd){
        case MSG_IDS_REL::STATE_POSITION:
          if(i_joint >=0){
            std::string name = "joint"+std::to_string(i_joint+1)+"/position";
            float new_value;


            // Copy the bit pattern of `int` to `float`
            std::memcpy(&new_value, &data_buffer, sizeof(new_value));

            // std::stringstream ss;
            // ss << "Reading states:";
            // ss << std::fixed << std::setprecision(2) << std::endl
            //       << "\t" << new_value << " for joint '" << name << "'";
            
            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());


            //RCLCPP_INFO(get_logger(), (std::string("TRYING TO SET: \t")+name+std::string(" to: ")+std::string(std::to_string((double)new_value))).c_str());
            set_state(name, (double)new_value);
            //RCLCPP_INFO(get_logger(), (std::string("Successfully SET it:\t")+name+std::string(" to: ")+std::string(std::to_string((double)new_value))).c_str());


        }
        break;

      case MSG_IDS_REL::STATE:
        // RCLCPP_INFO(get_logger(), "RECEIVED STATE MSG");
        
        joint_state[i_joint]=padman_hw::STATES(data_buffer);

      }
      // did we receive the position of a real joint (i.e. not our pseudo joint request)
      
      if(i_joint>=0){ // we ignore messages we sent ourselves and just listen in the joint specific ranges
        // RCLCPP_INFO(get_logger(), (std::string("TRYING TO SET msg_timestamps(")+std::to_string(i_joint)+std::string(", ")+std::to_string(i_cmd)+std::string(")")).c_str());
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        msg_timestamps(i_joint, i_cmd)=now;
      }

      if (can_use_bus_time_) {
        // frame_msg.header.stamp =
        //   rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
      } else {
        // frame_msg.header.stamp = this->now();
      }

      // frame_msg.id = receive_id.identifier();
      // frame_msg.is_rtr = (receive_id.frame_type() == FrameType::REMOTE);
      // frame_msg.is_extended = receive_id.is_extended();
      // frame_msg.is_error = (receive_id.frame_type() == FrameType::ERROR);
      // frame_msg.dlc = receive_id.length();
      // frames_pub_->publish(std::move(frame_msg));
    }

  // } else {
  //   ros2_socketcan_msgs::msg::FdFrame fd_frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
  //   fd_frame_msg.header.frame_id = "can";

  //   while (rclcpp::ok()) {
  //     if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
  //       std::this_thread::sleep_for(100ms);
  //       continue;
  //     }

  //     fd_frame_msg.data.resize(64);

  //     try {
  //       receive_id = receiver_->receive_fd(fd_frame_msg.data.data<void>(), interval_ns_);
  //     } catch (const std::exception & ex) {
  //       RCLCPP_WARN_THROTTLE(
  //         this->get_logger(), *this->get_clock(), 1000,
  //         "Error receiving CAN FD message: %s - %s",
  //         interface_.c_str(), ex.what());
  //       continue;
  //     }

  //     fd_frame_msg.data.resize(receive_id.length());

  //     if (use_bus_time_) {
  //       fd_frame_msg.header.stamp =
  //         rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
  //     } else {
  //       fd_frame_msg.header.stamp = this->now();
  //     }

  //     fd_frame_msg.id = receive_id.identifier();
  //     fd_frame_msg.is_extended = receive_id.is_extended();
  //     fd_frame_msg.is_error = (receive_id.frame_type() == FrameType::ERROR);
  //     fd_frame_msg.len = receive_id.length();
  //     fd_frames_pub_->publish(std::move(fd_frame_msg));
  //   }
  // }
}



}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  padman_hw::PadmanSystemPositionOnlyHardware, hardware_interface::SystemInterface)
