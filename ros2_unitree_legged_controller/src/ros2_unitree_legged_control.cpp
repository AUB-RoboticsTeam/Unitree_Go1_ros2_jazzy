#include <limits>
// Copyright 2020 PAL Robotics S.L.
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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath> //!!
#include <chrono>
#include "ros2_unitree_legged_control/unitree_joint_control_tool.h"
#include "ros2_unitree_legged_control/ros2_unitree_legged_control.hpp"

// Implement missing control functions
double clamp(double& value, double min, double max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

double computeVel(double current_position, double last_position, double last_velocity, double duration) {
    if (duration <= 0.0 || !std::isfinite(duration)) {
        return last_velocity;
    }
    double raw_vel = (current_position - last_position) / duration;
    // Low-pass filter: 0.8 weight on previous, 0.2 on new
    double filtered_vel = 0.8 * last_velocity + 0.2 * raw_vel;
    return std::isfinite(filtered_vel) ? filtered_vel : 0.0;
}

double computeTorque(double current_position, double current_velocity, ServoCmd& cmd) {
    double torque = 0.0;
    
    // Position control component
    if (std::fabs(cmd.pos - posStopF) > 1e-5 && cmd.posStiffness > 0.0) {
        double pos_error = cmd.pos - current_position;
        torque += cmd.posStiffness * pos_error;
    }
    
    // Velocity control component  
    if (std::fabs(cmd.vel - velStopF) > 1e-5 && cmd.velStiffness > 0.0) {
        double vel_error = cmd.vel - current_velocity;
        torque += cmd.velStiffness * vel_error;
    }
    
    // Add feedforward torque
    torque += cmd.torque;
    
    return std::isfinite(torque) ? torque : 0.0;
}

// Helper function: sane() to validate float values
float sane(float val) {
    if (!std::isfinite(val)) {
        return 0.0f;
    }
    return val;
}
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/parameter_client.hpp"

namespace ros2_unitree_legged_control
{
UnitreeLeggedController::UnitreeLeggedController()
: controller_interface::ControllerInterface(),
  joints_command_subscriber_(nullptr), rt_controller_state_publisher_(nullptr), state_publisher_(nullptr)
{
  // memset(&lastCmd, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorCmd));
  // memset(&lastState, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorState));
  lastCmd = ros2_unitree_legged_msgs::msg::MotorCmd();
  lastState = ros2_unitree_legged_msgs::msg::MotorState();
  memset(&servoCmd, 0, sizeof(ServoCmd));
}

controller_interface::CallbackReturn UnitreeLeggedController::on_init()
{

  try
  {
    declare_parameters();

    // Initialize variables and pointers
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void UnitreeLeggedController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  // Allow injecting full URDF text via YAML so controllers can read limits
  if (!get_node()->has_parameter("robot_description")) {
    get_node()->declare_parameter<std::string>("robot_description", std::string(""));
  }
}

controller_interface::CallbackReturn UnitreeLeggedController::read_parameters(){
  using namespace std::chrono_literals;
  auto try_get_robot_description_from = [&](const std::string & remote_node) -> bool {
    try {
      auto client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), remote_node);
      if (!client->wait_for_service(2s)) {
        return false;
      }
      auto params = client->get_parameters({"robot_description"});
      if (!params.empty() && params.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        urdf_string_ = params.front().as_string();
        if (!urdf_string_.empty() && model_.initString(urdf_string_)) {
          RCLCPP_INFO(get_node()->get_logger(), "Fetched URDF from %s", remote_node.c_str());
          return true;
        }
      }
    } catch (...) {
      return false;
    }
    return false;
  };

  // Read joint name here, used in state and interface configuration
  // Custom param struct is generated accodring to yaml file under ros2_unitree_legged_control_parameters.hpp

  if (!param_listener_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init. param_listener does not exist");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();

    if (params_.joint_name.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joint_name' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    bool have_urdf = false;
    if (get_node()->get_parameter("robot_description", urdf_string_)) {
      if (!urdf_string_.empty() && model_.initString(urdf_string_)) {
        have_urdf = true;
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "robot_description parameter present but URDF parse failed; will try fallbacks");
      }
    }
    if (!have_urdf) {
      have_urdf = try_get_robot_description_from("/robot_state_publisher_node") ||
                  try_get_robot_description_from("/controller_manager");
    }
    if (!have_urdf) {
      RCLCPP_WARN(get_node()->get_logger(), "No robot_description available; joint limits will be disabled");
    }

    joint_name_ = params_.joint_name;
    joint_ = model_.getJoint(joint_name_);
  if (joint_ && joint_->limits) { 
    RCLCPP_INFO_STREAM(get_node()->get_logger(), 
      "Configured joint: " << joint_name_ 
      << " with limits: effort=" << joint_->limits->effort 
      << " vel=" << joint_->limits->velocity 
      << " lower=" << joint_->limits->lower 
      << " upper=" << joint_->limits->upper); 
  } else { 
    RCLCPP_WARN(get_node()->get_logger(), 
      "Configured joint: %s with NO URDF limits available (clamping will be unbounded).", joint_name_.c_str()); 
  }

    // if(joint_name_.find("hip") != std::string::npos){
    //   joint_type_ = 0;
    // }
    // else if(joint_name_.find("thigh") != std::string::npos){
    //   joint_type_ = 1;
    // }
    // else if(joint_name_.find("calf") != std::string::npos){
    //   joint_type_ = 2;
    // }

    

    return controller_interface::CallbackReturn::SUCCESS;
}

void UnitreeLeggedController::setCmdCallback(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg){
  lastCmd.mode = msg->mode;
  lastCmd.q = msg->q;
  lastCmd.kp = msg->kp;
  lastCmd.dq = msg->dq;
  lastCmd.kd = msg->kd;
  lastCmd.tau = msg->tau;

  rt_command_.writeFromNonRT(lastCmd); 
}

controller_interface::CallbackReturn UnitreeLeggedController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // command callback
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/command", rclcpp::SystemDefaultsQoS(), std::bind(&UnitreeLeggedController::setCmdCallback, this, std::placeholders::_1));

  state_publisher_ = get_node()->create_publisher<StateType>(
    "~/state", rclcpp::SystemDefaultsQoS()
  );

  // rt_controller_state_publisher_.reset(
  //   new realtime_tools::RealtimePublisher<StateType>(state_publisher_)
  // );
  rt_controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<StateType>>(state_publisher_);

  // Initialize rt_command buffer
  rt_command_.initRT(CmdType());

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UnitreeLeggedController::command_interface_configuration() const
{
  // controller_interface::InterfaceConfiguration command_interfaces_config;
  // command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // command_interfaces_config.names = command_interface_types_;
  std::vector<std::string> joint_names;
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);

  return {controller_interface::interface_configuration_type::INDIVIDUAL, joint_names};

}

controller_interface::InterfaceConfiguration UnitreeLeggedController::state_interface_configuration()
  const
{ 
  std::vector<std::string> joint_names;
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::INDIVIDUAL, joint_names};
}

controller_interface::CallbackReturn UnitreeLeggedController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  // TODO
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  //   ordered_interfaces;
  // if (
  //   !controller_interface::get_ordered_interfaces(
  //     command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
  //   command_interface_types_.size() != ordered_interfaces.size())
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
  //     command_interface_types_.size(), ordered_interfaces.size());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // reset command buffer if a command came through callback when controller was inactive
  // rt_command_ = realtime_tools::RealtimeBuffer<CmdType>();
  RCLCPP_INFO(get_node()->get_logger(), "accessing state interface");
  double init_pose = 0.0; if (auto opt = state_interfaces_[0].get_optional()) { init_pose = *opt; } else { RCLCPP_WARN(get_node()->get_logger(), "No initial position value available; defaulting to 0"); }
  RCLCPP_INFO(get_node()->get_logger(), "accessing state interface successful");
  // DON'T lock to current position during activation (may be mid-air during free-fall)
  // Instead use PosStopF to indicate no position control until first real command
  lastCmd.q = PosStopF;  // Stop position control initially
  lastState.q = static_cast<float>(init_pose);  // Only track actual position
  lastCmd.dq = 0.0f;
  lastState.dq = 0.0f;
  lastCmd.tau = 0.0f;
  lastState.tau_est = 0.0f;
  // On first activation, allow free-fall by using zero damping mode
  // This prevents unnatural holding forces during startup
  if (first_activation_) {
    RCLCPP_INFO(get_node()->get_logger(), "[%s] First activation - entering free-fall mode (zero damping)", joint_name_.c_str());
    lastCmd.mode = PMSM;  // Position-velocity mode but with zero stiffness
    lastCmd.kp   = 0.0f;
    lastCmd.kd   = 0.0f;  // Zero damping for free-fall
    lastCmd.tau  = 0.0f;
    first_activation_ = false;
  } else {
    // Subsequent activations use brake as normal
    RCLCPP_INFO(get_node()->get_logger(), "[%s] Reactivation - applying brake damping", joint_name_.c_str());
    lastCmd.mode = BRAKE;
    lastCmd.kp   = 0.0f;
    lastCmd.kd   = 20.0f;   // damping
    lastCmd.tau  = 0.0f;
  }

  // ensure servoCmd is zeroed for first update cycle
  servoCmd = ServoCmd{};
  activation_time_ = get_node()->get_clock()->now();
  RCLCPP_INFO(get_node()->get_logger(), "[%s] activate successful (will hold zero torque for %.1f seconds to allow free-fall)", joint_name_.c_str(), FREE_FALL_DURATION);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeLeggedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ = realtime_tools::RealtimeBuffer<CmdType>();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UnitreeLeggedController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto joint_commands = rt_command_.readFromRT();
  
  // CRITICAL: Match Humble behavior - do nothing if no command received yet
  // This prevents applying default BRAKE damping before guide controller starts
  if (!joint_commands)
  {
    publishState();
    return controller_interface::return_type::OK;
  }
  
  lastCmd = *(joint_commands);

  // --- Sanitize incoming command ---
  lastCmd.q   = sane(lastCmd.q);
  lastCmd.dq  = sane(lastCmd.dq);
  lastCmd.kp  = sane(lastCmd.kp);
  lastCmd.kd  = sane(lastCmd.kd);
  lastCmd.tau = sane(lastCmd.tau);

  // Supported modes only
  if (lastCmd.mode != PMSM && lastCmd.mode != BRAKE) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "[%s] Unsupported mode %u; forcing BRAKE",
                         joint_name_.c_str(), lastCmd.mode);
    lastCmd.mode = BRAKE;
  }

  // --- Build servoCmd from lastCmd ---
  if (lastCmd.mode == PMSM) {
    servoCmd.pos = lastCmd.q;
    positionLimits(servoCmd.pos);

    servoCmd.posStiffness = lastCmd.kp;
    if (std::fabs(lastCmd.q - PosStopF) < 1e-5) {
      servoCmd.posStiffness = 0.0f;
    }

    servoCmd.vel = lastCmd.dq;
    velocityLimits(servoCmd.vel);

    servoCmd.velStiffness = lastCmd.kd;
    if (std::fabs(lastCmd.dq - VelStopF) < 1e-5) {
      servoCmd.velStiffness = 0.0f;
    }

    servoCmd.torque = lastCmd.tau;
    effortLimits(servoCmd.torque);
  }
  else { // BRAKE
    servoCmd.posStiffness = 0;
    servoCmd.vel = 0;
    servoCmd.velStiffness = 20;
    servoCmd.torque = 0;
    effortLimits(servoCmd.torque);
  }

  // --- Read state interfaces ---
  double currentPos = 0.0, currentVel = 0.0, calcTorque = 0.0;

  if (auto opt = state_interfaces_[0].get_optional()) {
    currentPos = *opt;
  } else {
    // nothing we can do without position
    publishState();
    return controller_interface::return_type::OK;
  }

  // Guard dt to avoid divide-by-zero in velocity estimation
  const double raw_dt = period.seconds();
  const double dt = (std::isfinite(raw_dt) && raw_dt > 1e-6) ? raw_dt : 1e-3;

  currentVel = computeVel(currentPos,
                          static_cast<double>(lastState.q),
                          static_cast<double>(lastState.dq),
                          dt);

  // --- Check if still in free-fall period (zero torque) ---
  double elapsed_since_activation = (get_node()->get_clock()->now() - activation_time_).seconds();
  if (elapsed_since_activation < FREE_FALL_DURATION) {
    // During free-fall: output zero torque regardless of control mode
    calcTorque = 0.0;
    if (elapsed_since_activation < 0.1) {  // Log only first 100ms
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
        "[%s] Free-fall phase (%.2f/%.1f s) - zero torque",
        joint_name_.c_str(), elapsed_since_activation, FREE_FALL_DURATION);
    }
  } else {
    // After free-fall: compute torque normally
    calcTorque = computeTorque(currentPos, currentVel, servoCmd);
    if (!std::isfinite(calcTorque)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "[%s] calcTorque became non-finite (q=%.3f dq=%.3f pos=%.3f vel=%.3f kp=%.3f kd=%.3f tau=%.3f). Forcing 0.",
        joint_name_.c_str(),
        static_cast<double>(lastCmd.q), static_cast<double>(lastCmd.dq),
        currentPos, currentVel,
        static_cast<double>(lastCmd.kp), static_cast<double>(lastCmd.kd),
        static_cast<double>(lastCmd.tau));
      calcTorque = 0.0;
    }
  }

  // Optional: clamp torque to URDF effort limit (safe even if no limits; your helper skips)
  effortLimits(calcTorque);

  // --- Sanity asserts (interface names) ---
  assert (command_interfaces_[0].get_interface_name() == hardware_interface::HW_IF_EFFORT);
  assert (state_interfaces_[0].get_interface_name() == hardware_interface::HW_IF_POSITION);
  assert (state_interfaces_[1].get_interface_name() == hardware_interface::HW_IF_EFFORT);

  // --- Write command ---
  bool ok = command_interfaces_[0].set_value(calcTorque);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "[%s] set_value(calcTorque) failed", joint_name_.c_str());
  }

  // --- Update lastState & publish ---
  lastState.q  = static_cast<float>(currentPos);
  lastState.dq = static_cast<float>(currentVel);
  if (auto tau_opt = state_interfaces_[1].get_optional()) {
    lastState.tau_est = static_cast<float>(*tau_opt);
  }

  publishState();
  return controller_interface::return_type::OK;
}


void UnitreeLeggedController::publishState(){
  if (rt_controller_state_publisher_ && rt_controller_state_publisher_->trylock()) {
    rt_controller_state_publisher_->msg_.q = lastState.q;
    rt_controller_state_publisher_->msg_.dq = lastState.dq;
    rt_controller_state_publisher_->msg_.tau_est = lastState.tau_est;
    rt_controller_state_publisher_->unlockAndPublish();    
  }
}

void UnitreeLeggedController::getGains(double &p, double &i, double &d, double &i_max, double &i_min){
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

 void UnitreeLeggedController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void UnitreeLeggedController::positionLimits(double &position){
  if (!joint_ || !joint_->limits) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "[%s] No URDF limits; skipping position clamp.", joint_name_.c_str());
    return;
  }
  clamp(position, joint_->limits->lower, joint_->limits->upper);
  // switch (joint_type_){
  //   double upper, lower;
  //   case 0:
  //   // hip
  //     lower = -0.863, upper = 0.863;
  //     clamp(position, lower, upper);
  //     break;

  //   case 1:
  //   // thigh
  //     lower = -0.686, upper = 4.501;
  //     clamp(position, lower, upper);
  //     break;

  //   case 2:
  //   // calf
  //     lower = -2.818, upper = -0.888;
  //     clamp(position, lower, upper);
  //     break;

  //   default:
  //     break;
  // }
}

void UnitreeLeggedController::velocityLimits(double &velocity){
  if (!joint_ || !joint_->limits || !std::isfinite(joint_->limits->velocity)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "[%s] No URDF velocity limit; skipping velocity clamp.", joint_name_.c_str());
    return;
  }
  clamp(velocity, -joint_->limits->velocity, joint_->limits->velocity);
  // switch (joint_type_){
  //   case 0:
  //   // hip
  //     lower = -30.1, upper = 30.1;
  //     clamp(velocity, lower, upper);
  //     break;

  //   case 1:
  //   // thigh
  //     lower = -30.1, upper = 30.1;
  //     clamp(velocity, lower, upper);
  //     break;

  //   case 2:
  //   // calf
  //     lower = -20.06, upper = 20.06;
  //     clamp(velocity, lower, upper);
  //     break;

  //   default:
  //     break;
  // }
}

void UnitreeLeggedController::effortLimits(double &effort){
  if (!joint_ || !joint_->limits || !std::isfinite(joint_->limits->effort) || joint_->limits->effort <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "[%s] No URDF effort limit; skipping effort clamp.", joint_name_.c_str());
    return;
  }
  clamp(effort, -joint_->limits->effort, joint_->limits->effort);
  // double upper, lower;
  // switch (joint_type_){
  //   case 0:
  //   // hip
  //     lower = -23.7, upper = 23.7;
  //     clamp(effort, lower, upper);
  //     break;

  //   case 1:
  //   // thigh
  //     lower = -23.7, upper = 23.7;
  //     clamp(effort, lower, upper);
  //     break;

  //   case 2:
  //   // calf
  //     lower = -35.5, upper = 35.5;
  //     clamp(effort, lower, upper);
  //     break;

  //   default:
  //     break;
  // }
}

}  // namespace ros2_unitree_legged_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_unitree_legged_control::UnitreeLeggedController, controller_interface::ControllerInterface)
