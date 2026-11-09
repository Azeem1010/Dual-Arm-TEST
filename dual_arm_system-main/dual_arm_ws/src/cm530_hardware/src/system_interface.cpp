#include "cm530_hardware/system_interface.hpp"

#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <stdexcept>

#include "cm530_hardware/posix_serial.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cm530_hardware
{
namespace
{

bool parse_bool(const std::string & text, const std::string & name)
{
  if (text == "true") {
    return true;
  }
  if (text == "false") {
    return false;
  }
  throw std::runtime_error(std::format("{} must be true or false, got '{}'", name, text));
}

std::chrono::milliseconds optional_ms(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  std::chrono::milliseconds fallback)
{
  const auto found = info.hardware_parameters.find(name);
  if (found == info.hardware_parameters.end()) {
    return fallback;
  }
  const int value = std::stoi(found->second);
  if (value <= 0) {
    throw std::runtime_error(std::format("{} must be positive", name));
  }
  return std::chrono::milliseconds{value};
}

}  // namespace

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    port_ = info_.hardware_parameters.at("port");
    calibration_file_ = info_.hardware_parameters.at("calibration_file");
    enable_motion_ = parse_bool(info_.hardware_parameters.at("enable_motion"), "enable_motion");
    command_timeout_ = optional_ms(info_, "command_timeout_ms", command_timeout_);
    startup_listen_ = optional_ms(info_, "startup_listen_ms", startup_listen_);
    feedback_stale_ = optional_ms(info_, "feedback_stale_ms", feedback_stale_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid hardware parameters: %s", error.what());
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(get_logger(), "Expected %zu joints, got %zu", kJointCount, info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const auto & joint = info_.joints[index];
    if (joint.name != kJointNames[index]) {
      RCLCPP_ERROR(
        get_logger(), "Joint %zu is '%s', expected '%s'", index, joint.name.c_str(),
        std::string{kJointNames[index]}.c_str());
      return CallbackReturn::ERROR;
    }
    const bool interfaces_ok =
      joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
      joint.state_interfaces.size() == 2 &&
      joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
      joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY;
    if (!interfaces_ok) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs position command and position+velocity state", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    position_state_[index] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    velocity_state_[index] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    position_command_[index] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
  }
  return CallbackReturn::SUCCESS;
}

bool CM530SystemInterface::require_ok(const LinkResult & result, const std::string & action)
{
  if (result.ok()) {
    return true;
  }
  RCLCPP_ERROR(get_logger(), "%s failed: %s", action.c_str(), result.detail.c_str());
  return false;
}

void CM530SystemInterface::publish_positions(const std::array<RawPose, kArmCount> & poses)
{
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const int raw = poses[index / kJointsPerArm][index % kJointsPerArm];
    last_rad_[index] = calibration_->raw_to_rad(index, raw);
    set_state(position_state_[index], last_rad_[index]);
    set_state(velocity_state_[index], 0.0);
  }
}

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    calibration_ = Calibration::from_file(calibration_file_);
    if (enable_motion_) {
      const auto blockers = calibration_->motion_blockers(is_pseudo_terminal(port_));
      if (!blockers.empty()) {
        for (const auto & blocker : blockers) {
          RCLCPP_ERROR(get_logger(), "Motion refused: %s", blocker.c_str());
        }
        return CallbackReturn::ERROR;
      }
    }
    link_ = std::make_unique<Link>(std::make_unique<PosixSerial>(port_), command_timeout_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Configuration refused: %s", error.what());
    return CallbackReturn::ERROR;
  }

  if (!require_ok(link_->handshake(startup_listen_), "Handshake")) {
    release_link();
    return CallbackReturn::ERROR;
  }
  for (const Arm arm : kArms) {
    const std::string name{arm_name(arm)};
    const LinkResult home = link_->transact(Command::get_home(arm));
    if (!require_ok(home, "GET_HOME " + name)) {
      release_link();
      return CallbackReturn::ERROR;
    }
    if (home.pose != calibration_->expected_home(arm)) {
      RCLCPP_ERROR(get_logger(), "Board HOME for %s differs from the calibration: %s", name.c_str(), home.detail.c_str());
      release_link();
      return CallbackReturn::ERROR;
    }
    const LinkResult read = link_->transact(Command::read(arm));
    if (!require_ok(read, "READ " + name) || !read.pose) {
      release_link();
      return CallbackReturn::ERROR;
    }
    latest_raw_[static_cast<std::size_t>(arm)] = *read.pose;
  }
  publish_positions(latest_raw_);
  RCLCPP_INFO(
    get_logger(), "CM-530 connected on %s (%s)", port_.c_str(),
    enable_motion_ ? "motion authorized" : "read-only");
  return CallbackReturn::SUCCESS;
}

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!link_ || !link_->healthy()) {
    RCLCPP_ERROR(get_logger(), "Activation refused: serial link is not healthy");
    return CallbackReturn::ERROR;
  }
  for (const Arm arm : kArms) {
    const std::string name{arm_name(arm)};
    const LinkResult read = link_->transact(Command::read(arm));
    if (!require_ok(read, "READ " + name) || !read.pose) {
      return CallbackReturn::ERROR;
    }
    latest_raw_[static_cast<std::size_t>(arm)] = *read.pose;
    if (enable_motion_) {
      // The firmware requires a target before torque, so hold the measured pose.
      if (!require_ok(link_->transact(Command::ax(arm, *read.pose)), "AX " + name) ||
        !require_ok(link_->transact(Command::torque_enable(arm, true)), "TORQUE on " + name))
      {
        return CallbackReturn::ERROR;
      }
    }
  }
  publish_positions(latest_raw_);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    set_command(position_command_[index], last_rad_[index]);
  }
  fault_logged_ = false;
  const auto now = std::chrono::steady_clock::now();
  last_stamp_.fill(now);
  SchedulerOptions options;
  options.enable_motion = enable_motion_;
  options.max_step_raw = calibration_->max_step_raw();
  scheduler_ = std::make_unique<Scheduler>(*link_, options, latest_raw_);
  scheduler_->start();
  return CallbackReturn::SUCCESS;
}

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  scheduler_.reset();
  if (enable_motion_ && link_ && link_->healthy()) {
    // Torque stays on: dropping it would let the arms fall.
    for (const Arm arm : kArms) {
      const std::string name{arm_name(arm)};
      if (!require_ok(link_->transact(Command::hold(arm)), "HOLD " + name)) {
        return CallbackReturn::ERROR;
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

void CM530SystemInterface::release_link()
{
  scheduler_.reset();
  link_.reset();
}

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_link();
  return CallbackReturn::SUCCESS;
}

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_link();
  return CallbackReturn::SUCCESS;
}

CM530SystemInterface::CallbackReturn CM530SystemInterface::on_error(const rclcpp_lifecycle::State &)
{
  // The firmware has no STOP or watchdog and the link state is unknown, so
  // nothing further is sent. Motors keep their last target until the hardware
  // emergency stop or a deliberate operator action.
  RCLCPP_ERROR(get_logger(), "CM-530 fault: communication stopped; use the hardware emergency stop if needed");
  release_link();
  return CallbackReturn::SUCCESS;
}

CM530SystemInterface::ReturnType CM530SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!scheduler_) {
    return ReturnType::OK;
  }
  if (scheduler_->faulted()) {
    if (!fault_logged_) {
      RCLCPP_ERROR(get_logger(), "%s", scheduler_->fault().c_str());
      fault_logged_ = true;
    }
    return ReturnType::ERROR;
  }
  ArmSamples samples;
  if (!scheduler_->snapshot(samples)) {
    return ReturnType::OK;
  }
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t arm = 0; arm < kArmCount; ++arm) {
    if (now - samples[arm].stamp > feedback_stale_) {
      RCLCPP_ERROR(get_logger(), "Feedback for arm %zu is stale", arm + 1);
      return ReturnType::ERROR;
    }
    if (samples[arm].stamp == last_stamp_[arm]) {
      continue;
    }
    const double dt = std::chrono::duration<double>(samples[arm].stamp - last_stamp_[arm]).count();
    last_stamp_[arm] = samples[arm].stamp;
    for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
      const std::size_t index = arm * kJointsPerArm + joint;
      const double position = calibration_->raw_to_rad(index, samples[arm].raw[joint]);
      set_state(position_state_[index], position);
      // The firmware reports no velocity; this is a finite difference of READ samples.
      set_state(velocity_state_[index], (position - last_rad_[index]) / dt);
      last_rad_[index] = position;
    }
  }
  return ReturnType::OK;
}

CM530SystemInterface::ReturnType CM530SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!scheduler_ || !enable_motion_) {
    return ReturnType::OK;
  }
  for (const Arm arm : kArms) {
    const auto arm_index = static_cast<std::size_t>(arm);
    RawPose target{};
    for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
      const std::size_t index = arm_index * kJointsPerArm + joint;
      const double command = get_command<double>(position_command_[index]);
      const auto raw = calibration_->rad_to_raw(index, command);
      if (!raw) {
        RCLCPP_ERROR(get_logger(), "Command %f rad for joint %zu is outside its calibrated limits", command, index);
        return ReturnType::ERROR;
      }
      target[joint] = *raw;
    }
    // A busy lock only delays the target to the next control cycle.
    static_cast<void>(scheduler_->submit(arm, target));
  }
  return ReturnType::OK;
}

}  // namespace cm530_hardware

PLUGINLIB_EXPORT_CLASS(cm530_hardware::CM530SystemInterface, hardware_interface::SystemInterface)
