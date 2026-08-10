// Copyright 2023 Personal Robotics Lab, University of Washington
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Ethan K. Gordon

#include "ada_hardware/gen3.hpp"

// kinova api
// #include <Kinova.API.UsbCommandLayerUbuntu.h>
// #include <KinovaTypes.h>

// ROS
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ada_hardware
{

// Destructor
Gen3::~Gen3()
{
  try{stopMotion();} catch(...) {}
  try{setTorqueMode(false);} catch(...) {}

  if(session_manager_){
    Kinova::Api::Session::CreateSessionInfo session_info;
    try{session_manager_->CloseSession();} catch(...) {}
  }
  if(session_manager_udp_){
    try{session_manager_udp_->CloseSession();} catch(...) {}
  }

  //reset smart pointers in reverse construction order
  actuator_config_.reset();
  base_cyclic_.reset();
  base_.reset();
  device_manager_.reset();
  session_manager_.reset();
  session_manager_udp_.reset();
  router_udp_.reset();
  transport_udp_.reset();
  router_tcp_.reset();
  transport_tcp_.reset();

  //setTorqueMode(false);

  //EraseAllTrajectories();

  //SetCartesianControl();

  //StopControlAPI();

  //CloseAPI();
}


// Init: Read info and configure command/state buffers
hardware_interface::CallbackReturn Gen3::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read connection parameters from <hardware><param> tags in the urdf/xacro
  if(info_.hardware_parameters.count("robot_ip") == 0) {
    RCLCPP_FATAL(rclcpp::get_logger("Gen3"), "Missing required parameter 'robot_ip'.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  robot_ip_ = info_.hardware_parameters.at("robot_ip");

  if(info_.hardware_parameters.count("username")){
    username_ = info_.hardware_parameters.at("username");
  }
  if(info_.hardware_parameters.count("password")){
    password_ = info_.hardware_parameters.at("password");
  }
  if(info_.hardware_parameters.count("tcp_port")){
    tcp_port_ = std::stoi(info_.hardware_parameters.at("tcp_port"));
  }
  if(info_.hardware_parameters.count("udp_port")){
    udp_port_ = std::stoi(info_.hardware_parameters.at("udp_port"));
  }

  // Resize state/command buffers
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_ = integration_level_t::kUNDEFINED;
  control_connected_ = {false, false};

  position_offsets_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Validate interface counts for every joint
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Gen3"), "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    read_only_ = false;
    if (joint.command_interfaces.size() == 0) {
      read_only_ = true;
    } else if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Gen3"), "Joint '%s'has %zu command interfaces. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto check_interface = [&](const std::string & name, const std::string & kind){
      if(!(name == hardware_interface::HW_IF_POSITION || 
           name == hardware_interface::HW_IF_VELOCITY ||
           name == hardware_interface::HW_IF_EFFORT)) {
             RCLCPP_FATAL( rclcpp::get_logger("Gen3"), "Joint '%s'has %s %s interface. Expected %s, %s, or %s.",
              joint.name.c_str(), name.c_str(), kind.c_str(),
              hardware_interface::HW_IF_POSITION,
              hardware_interface::HW_IF_VELOCITY,
              hardware_interface::HW_IF_EFFORT);
            return false;
           }
      return true;
    };

    //CHECK WHY THIS NEEDS TO BE CHANGED
    if (!read_only_) {
      for (const hardware_interface::InterfaceInfo & interface_info : joint.command_interfaces) {
        if (!(interface_info.name == hardware_interface::HW_IF_POSITION ||
              interface_info.name == hardware_interface::HW_IF_VELOCITY ||
              interface_info.name == hardware_interface::HW_IF_EFFORT)) {
          RCLCPP_FATAL(
            rclcpp::get_logger("Gen3"), "Joint '%s' has %s command interface. Expected %s, %s, or %s.",
            joint.name.c_str(), interface_info.name.c_str(),
            hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }

    //CHECK WHY THIS NEEDS TO BE CHANGED
    for (const hardware_interface::InterfaceInfo & interface_info : joint.state_interfaces) {
      if (!(interface_info.name == hardware_interface::HW_IF_POSITION ||
            interface_info.name == hardware_interface::HW_IF_VELOCITY ||
            interface_info.name == hardware_interface::HW_IF_EFFORT)) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Gen3"), "Joint '%s' has %s state interface. Expected %s, %s, or %s.",
          joint.name.c_str(), interface_info.name.c_str(),
          hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_EFFORT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Gen3::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Gen3::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

// Mode Switching
// All joints must be the same mode.
hardware_interface::return_type Gen3::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Only command if commanding is possible
  if(read_only_) return hardware_interface::return_type::ERROR;

  // Prepare stopping command modes
  /*std::vector<integration_level_t> old_modes = {};
  for (std::string key : stop_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        old_modes.push_back(integration_level_t::kPOSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        old_modes.push_back(integration_level_t::kVELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        old_modes.push_back(integration_level_t::kEFFORT);
      }
    }
  }*/
  auto classify = [&](const std::vector<std::string> & ifaces){
    std::vector<integration_level_t> modes;

    for(const std::string & key : ifaces){
        for(std::size_t i = 0; i < info_.joints.size(); i++){
          if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
            modes.push_back(integration_level_t::kPOSITION);
          }
          else if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
            modes.push_back(integration_level_t::kVELOCITY);
          }
          else if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT){
            modes.push_back(integration_level_t::kEFFORT);
          }
        }
    }
    return modes;
  };

  // Handle Stop
  auto old_modes = classify(stop_interfaces);
  if (!old_modes.empty()) {
    // Criterion: All hand or all arm or all joints must be stopped at the same time
    if (
      old_modes.size() != num_dofs_.first && old_modes.size() != num_dofs_.second &&
      old_modes.size() != info_.joints.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Gen3"), "Must stop all hand, arm, or robot joints simultaneously.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: All joints must have the same (existing) command mode
    if (!std::all_of(old_modes.begin() + 1, old_modes.end(), [&](integration_level_t mode) {
          return mode == control_level_;
        })) {
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "All stopped joints must be in the same mode.");
      return hardware_interface::return_type::ERROR;
    }

    // Record removal of connected control
    if (old_modes.size() == num_dofs_.second) {
      control_connected_.second = false;
    } else if (old_modes.size() == num_dofs_.first) {
      control_connected_.first = false;
    } else {
      control_connected_.first = control_connected_.second = false;
    }
    if (!control_connected_.first && !control_connected_.second) {
      // Stop motion
      {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (!stopMotion()) {
          RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not stop robot.");
          return hardware_interface::return_type::ERROR;
        }
      }
      control_level_ = integration_level_t::kUNDEFINED;
    }
  }

  // Prepare for new command modes
  /*std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(integration_level_t::kPOSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(integration_level_t::kVELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes.push_back(integration_level_t::kEFFORT);
      }
    }
  }*/

  // Handle Start
  auto new_modes = classify(start_interfaces);
  if (!new_modes.empty()) {
    // Criterion: All hand or all arm or all joints must be given new command mode at the same time
    if (
      new_modes.size() != num_dofs_.first && new_modes.size() != num_dofs_.second &&
      new_modes.size() != info_.joints.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Gen3"), "Must request all hand, arm, or robot joints simultaneously.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: All joints must have the same command mode
    if (!std::all_of(new_modes.begin() + 1, new_modes.end(), [&](integration_level_t mode) {
          return mode == new_modes[0];
        })) {
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "All joints must be the same command mode.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: Joints must not be in use.
    bool inUse = (new_modes.size() == num_dofs_.second && control_connected_.second) ||
                 (new_modes.size() == num_dofs_.first && control_connected_.first) ||
                 (control_connected_.first && control_connected_.second);
    if (inUse) {
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Joints already in use.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: if finger joints only, must be the same mode as what's already here
    if (new_modes.size() == num_dofs_.second) {
      if (control_level_ != integration_level_t::kUNDEFINED && new_modes[0] != control_level_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("Gen3"), "Hand controller can't override arm control mode.");
        return hardware_interface::return_type::ERROR;
      }
    }

    // Set the new command mode
    // Record addition of connected control
    if (new_modes.size() == num_dofs_.second) {
      control_connected_.second = true;
    } else if (new_modes.size() == num_dofs_.first) {
      control_connected_.first = true;
    } else {
      control_connected_.first = control_connected_.second = true;
    }
    if (control_connected_.first || control_connected_.second) {
      // Stop motion
      {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (!stopMotion()) {
          RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not stop robot before mode switch.");
          return hardware_interface::return_type::ERROR;
        }
      }
      control_level_ = new_modes[0];
    }
  }

  return hardware_interface::return_type::OK;
}

// Configure: init api, make sure joint number matches robot
hardware_interface::CallbackReturn Gen3::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) // GURNOORK ADDED ONE HERE
{
  try{
    transport_tcp_ = std::make_unique<Kinova::Api::TransportClientTcp>();
    router_tcp_ = std::make_unique<Kinova::Api::RouterClient>(transport_tcp_.get(),[](Kinova::Api::KError err) {
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "TCP router error: %s", err.toString().c_str());
    });
    transport_tcp_->connect(robot_ip_, tcp_port_);

    transport_udp_ = std::make_unique<Kinova::Api::TransportClientUdp>();
    router_udp_ = std::make_unique<Kinova::Api::RouterClient>(transport_udp_.get(),[](Kinova::Api::KError err) {
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "UDP router error: %s", err.toString().c_str());
    });
    transport_udp_->connect(robot_ip_, udp_port_);

    //Open session
    Kinova::Api::Session::CreateSessionInfo session_info;
    session_info.set_username(username_);
    session_info.set_password(password_);
    session_info.set_session_inactivity_timeout(60000);
    session_info.set_connection_inactivity_timeout(2000);

    session_manager_ = std::make_unique<Kinova::Api::SessionManager>(router_tcp_.get());
    session_manager_->CreateSession(session_info);
    session_manager_udp_ = std::make_unique<Kinova::Api::SessionManager>(router_udp_.get());
    session_manager_udp_->CreateSession(session_info);

    //Instantiate client stubs
    base_ = std::make_unique<Kinova::Api::Base::BaseClient>(router_tcp_.get());
    base_cyclic_ = std::make_unique<Kinova::Api::BaseCyclic::BaseCyclicClient>(router_udp_.get());
    actuator_config_ = std::make_unique<Kinova::Api::ActuatorConfig::ActuatorConfigClient>(router_tcp_.get());
    device_manager_ = std::make_unique<Kinova::Api::DeviceManager::DeviceManagerClient>(router_tcp_.get());
  }
  catch(const Kinova::Api::KDetailedException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Kortex exception during configure %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch(const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Kortex exception during configure %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  //Verify joint count matches hardware
  try{
    auto actuator_count = base_->GetActuatorCount();
    num_dofs_.first = actuator_count.count(); //arm joints
    num_dofs_.second = 0; //gripper joints, need to confirm if this correct

    //Check for attached gripper
    if(info_.joints.size() > num_dofs_.first){
      num_dofs_.second = info_.joints.size() - num_dofs_.first;
    }

    if(info_.joints.size() != num_dofs_.first + num_dofs_.second){
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Provided number of joints (%zu) does not match robot (%zu arm + %zu gripper).", info_.joints.size(), num_dofs_.first, num_dofs_.second);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch(const Kinova::Api::KDetailedException & ex){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not get actuator count: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_offsets_.resize(num_dofs_.first, 0.0);

  //start in position mode
  if(!read_only_){
    if(!setTorqueMode(false)){
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not disable torque mode on configure.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate: start control api, angular control, 0 values
hardware_interface::CallbackReturn Gen3::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/ ) // GURNOORK ADDED ONE HERE
{
  // Initialize Default Values
  auto ret = read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not read default position.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initializeOffsets()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    if (std::isnan(hw_states_positions_[i])) {
      hw_states_positions_[i] = 0.0;
    }
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_states_efforts_[i])) {
      hw_states_efforts_[i] = 0.0;
    }
    if (std::isnan(hw_commands_positions_[i])) {
      hw_commands_positions_[i] = hw_states_positions_[i];
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_commands_efforts_[i])) {
      hw_commands_efforts_[i] = 0.0;
    }
    control_level_ = integration_level_t::kUNDEFINED;
  }

  if(!read_only_) {
    if (!setTorqueMode(false)) {
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not set torque mode on configure");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// This makes the reported joint values to start within urdf limits
static const double hardcoded_pos_midpoints[7] = {0.0, M_PI, M_PI, 0.0, 0.0, 0.0, 0.0};
bool Gen3::initializeOffsets()
{
  // Clear and re-read offsets
  //position_offsets_.clear();
  position_offsets_.assign(num_dofs_.first, 0.0);
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    return false;
  }

  // Next, we wrap the positions so they are within -pi to pi of
  // the hardcoded midpoints, and add that to the offset.
  for (size_t i = 0; i < num_dofs_.first; i++) {
    while (hw_states_positions_[i] < hardcoded_pos_midpoints[i] - M_PI) {
      hw_states_positions_[i] += 2.0 * M_PI;
      position_offsets_[i] += 2.0 * M_PI;
    }
    while (hw_states_positions_[i] > hardcoded_pos_midpoints[i] + M_PI) {
      hw_states_positions_[i] -= 2.0 * M_PI;
      position_offsets_[i] -= 2.0 * M_PI;
    }
  }
  return true;
}

// Deactivate: stop trajectories, cartesian control, stop control api
hardware_interface::CallbackReturn Gen3::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/ ) // GURNOORK ADDED ONE HERE
{
  if (!setTorqueMode(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not disable torque mode on deactivate.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!stopMotion()) {
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not stop robot on deactivate.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Cleanup: close api, make sure joint number matches robot
hardware_interface::CallbackReturn Gen3::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/ ) // GURNOORK ADDED ONE HERE
{
  try{
    if(session_manager_){
      session_manager_->CloseSession();
    }
    if(session_manager_udp_){
      session_manager_udp_->CloseSession();
    }
  }
  catch(...){ }

  //Smart pointers clean up transports/routers on destruction
  actuator_config_.reset();
  base_cyclic_.reset();
  base_.reset();
  device_manager_.reset();
  session_manager_.reset();
  session_manager_udp_.reset();
  router_udp_.reset();
  transport_udp_.reset();
  router_tcp_.reset();
  transport_tcp_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Shutdown: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn Gen3::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.label() == "active") {
    try{setTorqueMode(false);} catch(...){}
    try{stopMotion();} catch(...){}
  }
  return on_cleanup(previous_state);
}

// Error: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn Gen3::on_error(
  const rclcpp_lifecycle::State & /* previous_state */ ) // GURNOORK ADDED ONE HERE
{
  try{setTorqueMode(false);} catch(...){}
  try{stopMotion();} catch(...){}

  return on_cleanup(rclcpp_lifecycle::State());
}

/////// Inline Unit Conversions
inline static double degreesToRadians(double degrees) { return (M_PI / 180.0) * degrees; }

inline static double radiansToDegrees(double radians) { return (180.0 / M_PI) * radians; }

inline static double radiansToFingerTicks(double radians)
{
  return (6800.0 / 80) * radians * 180.0 /
         M_PI;  // this magic number was found in the kinova-ros code,
                // kinova_driver/src/kinova_arm.cpp
}

inline static double fingerTicksToRadians(double ticks)
{
  return ticks * (80 / 6800.0) * M_PI /
         180.0;  // this magic number was found in the kinova-ros code,
                 // kinova_driver/src/kinova_arm.cpp
}

// Write Operations
bool Gen3::setTorqueMode(bool torqueMode)
{
  if(!actuator_config_){
    return true; //not yet configured, nothing to set
  }

  Kinova::Api::ActuatorConfig::ControlModeInformation ctrl_mode_info;
  ctrl_mode_info.set_control_mode(
    torqueMode ? Kinova::Api::ActuatorConfig::ControlMode::TORQUE : Kinova::Api::ActuatorConfig::ControlMode::POSITION
  );

  try{
    for(std::size_t i = 1; i <= num_dofs_.first; i++){
      actuator_config_->SetControlMode(ctrl_mode_info, static_cast<uint32_t>(i));
    }
  }
  catch(const Kinova::Api::KDetailedException & ex){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not set torque mode: %s", ex.what());
    return false;
  }

  return true;
}

bool Gen3::stopMotion(){
  if(!base_){
    return true;
  }

  try{base_->Stop();}
  catch(const Kinova::Api::KDetailedException & ex){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not stop robot: %s", ex.what());
    return false;
  }
  return true;
}

bool Gen3::sendVelocityCommand(const std::vector<double> & command)
{
  if(command.size() != num_dofs_.first + num_dofs_.second){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Incorrect command size (%zu), expected (%zu).", command.size(), num_dofs_.first, num_dofs_.second);
    return false;
  }
  
  //Arm joints
  Kinova::Api::Base::JointSpeeds joint_speeds;
  for(std::size_t i = 0; i<num_dofs_.first; i++){
    auto * js = joint_speeds.add_joint_speeds();
    js->set_joint_identifier(static_cast<uint32_t>(i));
    js->set_value(static_cast<float>(radiansToDegrees(command.at(i))));
  }

  try{
    base_->SendJointSpeedsCommand(joint_speeds);
  }
  catch(const Kinova::Api::KDetailedException & ex){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not send velocity command: %s", ex.what());
    return false;
  }

  // gripper
  if(num_dofs_.second > 0){
    Kinova::Api::Base::GripperCommand gripper_cmd;
    gripper_cmd.set_mode(Kinova::Api::Base::GRIPPER_SPEED);
    auto * finger = gripper_cmd.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    //Gripper command is normalized to [1,0]
    //map from rad/s if needed

    finger->set_value(static_cast<float>(command.at(num_dofs_.first)));
    try{
      base_->SendGripperCommand(gripper_cmd);
    }
    catch(const Kinova::Api::KDetailedException & ex){
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not send gripper velocity: %s", ex.what());
    return false;
    }
  }

  return true;
}

bool Gen3::sendPositionCommand(const std::vector<double> & command)
{
  if(command.size() != num_dofs_.first + num_dofs_.second){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Incorrect command size (%zu), expected (%zu).", command.size(), num_dofs_.first, num_dofs_.second);
    return false;
  }
  
  static std::vector<double> prev_command;

  //Arm joints
  //only send a new traj if the target has changed
  if(command != prev_command){
    Kinova::Api::Base::ConstrainedJointAngles joint_angles;
    for(std::size_t i = 0; i<num_dofs_.first; i++){
      auto * ja = joint_angles.mutable_joint_angles()->add_joint_angles();
      ja->set_joint_identifier(static_cast<uint32_t>(i));
      //remove offset before sending, robot expects raw hardware degrees
      double ang_deg = radiansToDegrees(command.at(i) - position_offsets_[i]);
      //warp to [0, 360) for Kortex API
      while(ang_deg < 0.0){
        ang_deg += 360.0;
      }
      while(ang_deg >= 360.0){
        ang_deg -= 360.0;
      }
      ja->set_value(static_cast<float>(ang_deg));
    }

    try{
      base_->PlayJointTrajectory(joint_angles);
      prev_command = command;
    }
    catch(const Kinova::Api::KDetailedException & ex){
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not send position command: %s", ex.what());
      return false;
    }
  }

  // gripper
  if(num_dofs_.second > 0){
    Kinova::Api::Base::GripperCommand gripper_cmd;
    gripper_cmd.set_mode(Kinova::Api::Base::GRIPPER_POSITION);
    auto * finger = gripper_cmd.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    //Gripper command is normaloized [1,0]
    //TODO: adjust mapping to URDF limits
    finger->set_value(static_cast<float>(command.at(num_dofs_.first)));
    try{
      base_->SendGripperCommand(gripper_cmd);
    }
    catch(const Kinova::Api::KDetailedException & ex){
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not send gripper position: %s", ex.what());
      return false;
    }
  }

  return true;
}

bool Gen3::sendEffortCommand(const std::vector<double> & command)
{
  if(command.size() != num_dofs_.first + num_dofs_.second){
    RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Incorrect command size (%zu), expected (%zu).", command.size(), num_dofs_.first, num_dofs_.second);
    return false;
  }

  Kinova::Api::BaseCyclic::Command cyclic_cmd;
  cyclic_cmd.set_frame_id(cyclic_feedback_.frame_id() + 1);

  for(std::size_t i = 0; i < num_dofs_.first; i++){
    auto * actuator_cmd = cyclic_cmd.add_actuators();
    // Flags: SERVOING (bit 0) | TORQUE_FEED_FORWARD (bit 4)
    // These are defined in the BaseCyclic ActuatorCommand proto.
    // Using raw values for portability across API versions.
    actuator_cmd->set_flags(0x11);
    actuator_cmd->set_position(cyclic_feedback_.actuators(static_cast<int>(i)).position());  // hold current position
    actuator_cmd->set_velocity(0.0f);
    actuator_cmd->set_torque_joint(static_cast<float>(command.at(i)));

  }

  try{
    base_cyclic_->Refresh(cyclic_cmd);
  }
  catch(const Kinova::Api::KDetailedException & ex){
      RCLCPP_ERROR(rclcpp::get_logger("Gen3"), "Could not send effort command: %s", ex.what());
      return false;
  }

  return true;
}

hardware_interface::return_type Gen3::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ ) // GURNOORK ADDED two HERE
{
  if(read_only_) return hardware_interface::return_type::OK;

  const std::lock_guard<std::mutex> lock(mMutex);
  std::vector<double> zero(num_dofs_.first + num_dofs_.second, 0.0);
  bool ret = true;

  switch (control_level_) {
    case integration_level_t::kVELOCITY:
      ret = sendVelocityCommand(hw_commands_velocities_);
      break;
    case integration_level_t::kPOSITION:
      ret = sendPositionCommand(hw_commands_positions_);
      break;
    case integration_level_t::kEFFORT:
      ret = sendEffortCommand(hw_commands_efforts_);
      break;
    default:
      // Stop Bot
      ret = sendVelocityCommand(zero);
  }

  return ret ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

// Read Operation
hardware_interface::return_type Gen3::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ ) // GURNOORK ADDED ONE HERE
{
  try{
    cyclic_feedback_ = base_cyclic_->RefreshFeedback();
  }
  catch(const Kinova::Api::KDetailedException & ex){
      RCLCPP_WARN(rclcpp::get_logger("Gen3"), "Could not read feedback: %s", ex.what());
      return hardware_interface::return_type::OK;
  }

  //Arm joints (indices 0...num_dofs_.first-1)
  for(std::size_t i = 0; i < num_dofs_.first; i++){
    const auto & actuator = cyclic_feedback_.actuators(static_cast<int>(i));
    hw_states_positions_[i] = degreesToRadians(static_cast<double>(actuator.position())) + position_offsets_[i];
    hw_states_velocities_[i] = degreesToRadians(static_cast<double>(actuator.velocity()));
    hw_states_efforts_[i] = static_cast<double>(actuator.torque());
  }

  //Gripper joints (indices num_dofs_.first...end)
  if(num_dofs_.second > 0 && cyclic_feedback_.has_interconnect()){
    const auto & gripper_fb = cyclic_feedback_.interconnect().gripper_feedback();
    if (gripper_fb.motor_size() > 0) {
      std::size_t gi = num_dofs_.first;
      hw_states_positions_[gi] = static_cast<double>(gripper_fb.motor(0).position());
      hw_states_velocities_[gi] = static_cast<double>(gripper_fb.motor(0).velocity());
      hw_states_efforts_[gi] = static_cast<double>(gripper_fb.motor(0).current_motor());
    }
  }

  return hardware_interface::return_type::OK;
}

};  // namespace ada_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ada_hardware::Gen3, hardware_interface::SystemInterface)
