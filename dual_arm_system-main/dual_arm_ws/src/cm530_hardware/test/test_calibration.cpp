#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <format>
#include <limits>
#include <numbers>
#include <string>

#include "cm530_hardware/calibration.hpp"

using cm530_hardware::Calibration;
using cm530_hardware::kJointNames;
using cm530_hardware::kMotorIds;

namespace
{

constexpr double kCount = (300.0 * std::numbers::pi / 180.0) / 1023.0;

std::string document(
  const std::string & status = "PASS", const std::string & evidence = "physical",
  const std::string & allowed = "true", const std::string & speed = "true",
  const std::string & step = "20", const std::string & zero = "631.35",
  const std::string & direction = "1", int motor_id_override = -1)
{
  std::string text = std::format(
    "status: {}\nevidence_kind: {}\nhardware_use_allowed: {}\nmoving_speed_verified: {}\nmax_step_raw: {}\n"
    "expected_home:\n  arm1: [90, 567, 712, 825]\n  arm2: [590, 567, 712, 825]\njoints:\n",
    status, evidence, allowed, speed, step);
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    const int id = (index == 0 && motor_id_override >= 0) ? motor_id_override : kMotorIds[index / 4][index % 4];
    text += std::format(
      "  {}: {{motor_id: {}, zero_raw: {}, direction: {}, min_raw: 100, max_raw: 900}}\n",
      kJointNames[index], id, zero, direction);
  }
  return text;
}

}  // namespace

TEST(Calibration, ConvertsLikeTheCoordinator)
{
  const auto calibration = Calibration::from_yaml(document());
  EXPECT_NEAR(calibration.raw_to_rad(1, 712), (712 - 631.35) * kCount, 1e-12);
  const auto reversed = Calibration::from_yaml(document("PASS", "physical", "true", "true", "20", "512", "-1"));
  EXPECT_NEAR(reversed.raw_to_rad(0, 512 + 100), -100 * kCount, 1e-12);
  EXPECT_EQ(reversed.rad_to_raw(0, -100 * kCount), 612);
}

TEST(Calibration, RoundTripsEveryRawValue)
{
  const auto calibration = Calibration::from_yaml(document());
  for (int raw = 100; raw <= 900; ++raw) {
    EXPECT_EQ(calibration.rad_to_raw(3, calibration.raw_to_rad(3, raw)), raw);
  }
}

TEST(Calibration, RejectsOutOfLimitAndNonFiniteCommands)
{
  const auto calibration = Calibration::from_yaml(document("PASS", "physical", "true", "true", "20", "512"));
  EXPECT_FALSE(calibration.rad_to_raw(0, (900 - 512 + 1) * kCount));
  EXPECT_FALSE(calibration.rad_to_raw(0, (100 - 512 - 1) * kCount));
  EXPECT_FALSE(calibration.rad_to_raw(0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(calibration.rad_to_raw(0, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(calibration.rad_to_raw(0, 0.0), 512);
}

TEST(Calibration, ShippedTemplateCannotLoad)
{
  const char * const path = std::getenv("CM530_TEMPLATE");
  ASSERT_NE(path, nullptr);
  EXPECT_THROW(static_cast<void>(Calibration::from_file(path)), std::runtime_error);
}

TEST(Calibration, RejectsInvalidDocuments)
{
  EXPECT_THROW(static_cast<void>(Calibration::from_yaml(document("PASS", "physical", "true", "true", "20", "null"))), std::runtime_error);
  EXPECT_THROW(static_cast<void>(Calibration::from_yaml(document("PASS", "physical", "true", "true", "20", "2000"))), std::runtime_error);
  EXPECT_THROW(static_cast<void>(Calibration::from_yaml(document("PASS", "physical", "true", "true", "20", "512", "0"))), std::runtime_error);
  EXPECT_THROW(static_cast<void>(Calibration::from_yaml(document("PASS", "physical", "true", "true", "20", "512", "1", 99))), std::runtime_error);
  EXPECT_THROW(static_cast<void>(Calibration::from_yaml(document("PASS", "physical", "true", "true", "0"))), std::runtime_error);
  EXPECT_THROW(static_cast<void>(Calibration::from_yaml("status: [")), std::runtime_error);
}

TEST(Calibration, AuthorizesMotionOnlyWithFullPhysicalEvidence)
{
  EXPECT_TRUE(Calibration::from_yaml(document()).motion_blockers(false).empty());
  EXPECT_FALSE(Calibration::from_yaml(document("NOT_RUN")).motion_blockers(false).empty());
  EXPECT_FALSE(Calibration::from_yaml(document("PASS", "physical", "false")).motion_blockers(false).empty());
  EXPECT_FALSE(Calibration::from_yaml(document("PASS", "physical", "true", "false")).motion_blockers(false).empty());
  EXPECT_FALSE(Calibration::from_yaml(document("PASS", "physical", "true", "true", "null")).motion_blockers(false).empty());
  EXPECT_FALSE(Calibration::from_yaml(document("PASS", "gazebo")).motion_blockers(false).empty());
}

TEST(Calibration, MockEvidenceNeverAuthorizesARealPort)
{
  const auto mock = Calibration::from_yaml(document("PASS", "mock_serial_test"));
  EXPECT_FALSE(mock.motion_blockers(false).empty());
  EXPECT_TRUE(mock.motion_blockers(true).empty());
}
