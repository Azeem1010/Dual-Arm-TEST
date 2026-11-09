#include "cm530_hardware/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace cm530_hardware
{
namespace
{

// AX-12A: 300 degrees over 1023 counts, identical to dual_arm_coordinator/home_pose.py.
constexpr double kRadiansPerCount = (300.0 * std::numbers::pi / 180.0) / 1023.0;

template<typename T>
T require(const YAML::Node & parent, const std::string & key, const std::string & context)
{
  const YAML::Node node = parent[key];
  if (!node.IsDefined() || node.IsNull()) {
    throw std::runtime_error(std::format("{}.{} is missing or null; it must be measured", context, key));
  }
  return node.as<T>();
}

int require_raw(const YAML::Node & parent, const std::string & key, const std::string & context)
{
  const int value = require<int>(parent, key, context);
  if (value < 0 || value > kMaxRaw) {
    throw std::runtime_error(std::format("{}.{}={} is outside 0..{}", context, key, value, kMaxRaw));
  }
  return value;
}

RawPose parse_home(const YAML::Node & node, std::string_view arm)
{
  if (!node.IsSequence() || node.size() != kJointsPerArm) {
    throw std::runtime_error(std::format("expected_home.{} must list four raw values", arm));
  }
  RawPose pose{};
  for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
    const int value = node[joint].as<int>();
    if (value < 0 || value > kMaxRaw) {
      throw std::runtime_error(std::format("expected_home.{} value {} is outside 0..{}", arm, value, kMaxRaw));
    }
    pose[joint] = static_cast<std::uint16_t>(value);
  }
  return pose;
}

JointCalibration parse_joint(const YAML::Node & joints, std::size_t index)
{
  const std::string name{kJointNames[index]};
  const YAML::Node node = joints[name];
  if (!node.IsDefined() || !node.IsMap()) {
    throw std::runtime_error(std::format("joints.{} is missing", name));
  }
  const int expected_id = kMotorIds[index / kJointsPerArm][index % kJointsPerArm];
  if (const int id = require<int>(node, "motor_id", name); id != expected_id) {
    throw std::runtime_error(std::format("{} motor_id {} does not match firmware ID {}", name, id, expected_id));
  }
  JointCalibration joint;
  joint.zero_raw = require<double>(node, "zero_raw", name);
  if (!std::isfinite(joint.zero_raw) || joint.zero_raw < 0.0 || joint.zero_raw > kMaxRaw) {
    throw std::runtime_error(std::format("{}.zero_raw is outside 0..{}", name, kMaxRaw));
  }
  joint.direction = require<int>(node, "direction", name);
  if (joint.direction != 1 && joint.direction != -1) {
    throw std::runtime_error(std::format("{}.direction must be +1 or -1", name));
  }
  joint.min_raw = require_raw(node, "min_raw", name);
  joint.max_raw = require_raw(node, "max_raw", name);
  if (joint.min_raw >= joint.max_raw) {
    throw std::runtime_error(std::format("{} requires min_raw < max_raw", name));
  }
  return joint;
}

}  // namespace

Calibration Calibration::from_yaml(const std::string & text)
{
  try {
    const YAML::Node root = YAML::Load(text);
    Calibration calibration;
    calibration.status_ = require<std::string>(root, "status", "calibration");
    calibration.evidence_kind_ = require<std::string>(root, "evidence_kind", "calibration");
    calibration.hardware_use_allowed_ = require<bool>(root, "hardware_use_allowed", "calibration");
    calibration.moving_speed_verified_ = require<bool>(root, "moving_speed_verified", "calibration");
    if (const YAML::Node step = root["max_step_raw"]; step.IsDefined() && !step.IsNull()) {
      const int value = step.as<int>();
      if (value <= 0 || value > kMaxRaw) {
        throw std::runtime_error(std::format("max_step_raw must be within 1..{}", kMaxRaw));
      }
      calibration.max_step_raw_ = value;
    }
    const YAML::Node home = root["expected_home"];
    if (!home.IsDefined() || !home.IsMap()) {
      throw std::runtime_error("expected_home is missing");
    }
    for (const Arm arm : kArms) {
      const std::string name{arm_name(arm)};
      calibration.expected_home_[static_cast<std::size_t>(arm)] = parse_home(home[name], name);
    }
    const YAML::Node joints = root["joints"];
    if (!joints.IsDefined() || !joints.IsMap() || joints.size() != kJointCount) {
      throw std::runtime_error("joints must define exactly eight joints");
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      calibration.joints_[index] = parse_joint(joints, index);
    }
    return calibration;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(std::format("invalid calibration YAML: {}", error.what()));
  }
}

Calibration Calibration::from_file(const std::string & path)
{
  try {
    return from_yaml(YAML::Dump(YAML::LoadFile(path)));
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(std::format("cannot read calibration {}: {}", path, error.what()));
  }
}

double Calibration::raw_to_rad(std::size_t joint, int raw) const noexcept
{
  const JointCalibration & config = joints_[joint];
  return config.direction * (raw - config.zero_raw) * kRadiansPerCount;
}

std::optional<std::uint16_t> Calibration::rad_to_raw(std::size_t joint, double radians) const noexcept
{
  if (!std::isfinite(radians)) {
    return std::nullopt;
  }
  const JointCalibration & config = joints_[joint];
  const double raw = std::round(config.zero_raw + config.direction * radians / kRadiansPerCount);
  if (raw < config.min_raw || raw > config.max_raw) {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>(raw);
}

std::vector<std::string> Calibration::motion_blockers(bool port_is_pseudo_terminal) const
{
  std::vector<std::string> blockers;
  if (status_ != "PASS") {
    blockers.push_back(std::format("calibration status is {}, not PASS", status_));
  }
  if (!hardware_use_allowed_) {
    blockers.emplace_back("hardware_use_allowed is false");
  }
  if (!moving_speed_verified_) {
    blockers.emplace_back("moving_speed_verified is false");
  }
  if (!max_step_raw_) {
    blockers.emplace_back("max_step_raw is not set");
  }
  const bool evidence_ok =
    evidence_kind_ == "physical" || (evidence_kind_ == "mock_serial_test" && port_is_pseudo_terminal);
  if (!evidence_ok) {
    blockers.push_back(std::format("evidence_kind {} cannot authorize this port", evidence_kind_));
  }
  return blockers;
}

const RawPose & Calibration::expected_home(Arm arm) const noexcept
{
  return expected_home_[static_cast<std::size_t>(arm)];
}

}  // namespace cm530_hardware
