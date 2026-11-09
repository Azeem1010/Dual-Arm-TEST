#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "cm530_hardware/protocol.hpp"

namespace cm530_hardware
{

struct JointCalibration
{
  double zero_raw{0.0};  // raw count at URDF q=0; may be fractional
  int direction{1};      // +1 when raw counts increase along the URDF axis
  int min_raw{0};
  int max_raw{kMaxRaw};
};

// Strictly validated calibration. Loading throws std::runtime_error when any
// joint field is missing or null, so no unmeasured value can reach a motor.
class Calibration
{
public:
  static Calibration from_yaml(const std::string & text);
  static Calibration from_file(const std::string & path);

  double raw_to_rad(std::size_t joint, int raw) const noexcept;
  // Empty when the angle is non-finite or outside the joint's raw limits.
  std::optional<std::uint16_t> rad_to_raw(std::size_t joint, double radians) const noexcept;

  // Empty result means motion is authorized. Mock-serial evidence is accepted
  // only on a pseudo-terminal, so it can never authorize a physical port.
  std::vector<std::string> motion_blockers(bool port_is_pseudo_terminal) const;

  const RawPose & expected_home(Arm arm) const noexcept;
  int max_step_raw() const noexcept { return max_step_raw_.value_or(0); }

private:
  Calibration() = default;

  std::array<JointCalibration, kJointCount> joints_{};
  std::array<RawPose, kArmCount> expected_home_{};
  std::string status_;
  std::string evidence_kind_;
  bool hardware_use_allowed_{false};
  bool moving_speed_verified_{false};
  std::optional<int> max_step_raw_;
};

}  // namespace cm530_hardware
