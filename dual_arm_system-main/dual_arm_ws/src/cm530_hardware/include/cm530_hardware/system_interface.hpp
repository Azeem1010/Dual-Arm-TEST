#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cm530_hardware/calibration.hpp"
#include "cm530_hardware/link.hpp"
#include "cm530_hardware/scheduler.hpp"
#include "hardware_interface/system_interface.hpp"

namespace cm530_hardware
{

// Read-only unless enable_motion is true and the calibration authorizes motion.
class CM530SystemInterface final : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  using ReturnType = hardware_interface::return_type;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ReturnType read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  ReturnType write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool require_ok(const LinkResult & result, const std::string & action);
  void publish_positions(const std::array<RawPose, kArmCount> & poses);
  void release_link();

  std::string port_;
  std::string calibration_file_;
  bool enable_motion_{false};
  std::chrono::milliseconds command_timeout_{500};
  std::chrono::milliseconds startup_listen_{200};
  std::chrono::milliseconds feedback_stale_{1000};

  std::optional<Calibration> calibration_;
  std::unique_ptr<Link> link_;
  std::unique_ptr<Scheduler> scheduler_;
  std::array<RawPose, kArmCount> latest_raw_{};

  std::array<std::string, kJointCount> position_state_;
  std::array<std::string, kJointCount> velocity_state_;
  std::array<std::string, kJointCount> position_command_;
  std::array<double, kJointCount> last_rad_{};
  std::array<std::chrono::steady_clock::time_point, kArmCount> last_stamp_{};
  bool fault_logged_{false};
};

}  // namespace cm530_hardware
