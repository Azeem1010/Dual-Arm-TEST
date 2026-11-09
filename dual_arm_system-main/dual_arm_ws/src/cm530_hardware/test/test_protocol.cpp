#include <gtest/gtest.h>

#include "cm530_hardware/protocol.hpp"

using cm530_hardware::Arm;
using cm530_hardware::Command;
using cm530_hardware::RawPose;
using cm530_hardware::ReplyStatus;
using cm530_hardware::classify_reply;
using cm530_hardware::format_command;

namespace
{
const RawPose kPose{90, 567, 712, 825};
}

TEST(Protocol, FormatsExactBytes)
{
  EXPECT_EQ(format_command(Command::ping()), "PING\n");
  EXPECT_EQ(format_command(Command::version()), "VERSION\n");
  EXPECT_EQ(format_command(Command::get_home(Arm::kArm2)), "GET_HOME,arm2\n");
  EXPECT_EQ(format_command(Command::read(Arm::kArm1)), "READ,arm1\n");
  EXPECT_EQ(format_command(Command::hold(Arm::kArm2)), "HOLD,arm2\n");
  EXPECT_EQ(format_command(Command::ax(Arm::kArm1, kPose)), "AX,arm1,90,567,712,825\n");
  EXPECT_EQ(format_command(Command::torque_enable(Arm::kArm2, true)), "TORQUE,arm2,1\n");
  EXPECT_EQ(format_command(Command::torque_enable(Arm::kArm1, false)), "TORQUE,arm1,0\n");
}

TEST(Protocol, AcceptsExactReplies)
{
  EXPECT_EQ(classify_reply(Command::ping(), "PONG").status, ReplyStatus::kOk);
  EXPECT_EQ(classify_reply(Command::version(), "VERSION,5").status, ReplyStatus::kOk);
  EXPECT_EQ(classify_reply(Command::ax(Arm::kArm1, kPose), "OK,AX,arm1").status, ReplyStatus::kOk);
  EXPECT_EQ(classify_reply(Command::hold(Arm::kArm2), "OK,HOLD,arm2").status, ReplyStatus::kOk);
  EXPECT_EQ(classify_reply(Command::torque_enable(Arm::kArm1, true), "OK,TORQUE,arm1,1").status, ReplyStatus::kOk);
  const auto read = classify_reply(Command::read(Arm::kArm2), "POS,arm2,1,2,3,1023");
  ASSERT_EQ(read.status, ReplyStatus::kOk);
  EXPECT_EQ(read.pose, (RawPose{1, 2, 3, 1023}));
  const auto home = classify_reply(Command::get_home(Arm::kArm1), "HOME,arm1,90,567,712,825");
  ASSERT_EQ(home.status, ReplyStatus::kOk);
  EXPECT_EQ(home.pose, kPose);
}

TEST(Protocol, RejectsAnythingElse)
{
  const auto ax = Command::ax(Arm::kArm1, kPose);
  for (const char * line : {"OK,AX,arm2", "OK,AX", "OK,AX,arm1,extra", "OK", "ok,ax,arm1", "PONG", ""}) {
    EXPECT_EQ(classify_reply(ax, line).status, ReplyStatus::kMismatch) << line;
  }
  EXPECT_EQ(classify_reply(Command::version(), "VERSION,4").status, ReplyStatus::kMismatch);
  EXPECT_EQ(classify_reply(Command::torque_enable(Arm::kArm1, true), "OK,TORQUE,arm1,0").status, ReplyStatus::kMismatch);
  const auto read = Command::read(Arm::kArm1);
  for (const char * line : {
      "POS,arm2,1,2,3,4", "HOME,arm1,1,2,3,4", "POS,arm1,1,2,3", "POS,arm1,1,2,3,4,5",
      "POS,arm1,1024,2,3,4", "POS,arm1,-1,2,3,4", "POS,arm1,+1,2,3,4", "POS,arm1,1.5,2,3,4",
      "POS,arm1,,2,3,4", "POS,arm1,1x,2,3,4", "OK,READ,arm1"})
  {
    EXPECT_EQ(classify_reply(read, line).status, ReplyStatus::kMismatch) << line;
  }
}

TEST(Protocol, SeparatesErrorsAndRestarts)
{
  const auto read = Command::read(Arm::kArm1);
  EXPECT_EQ(classify_reply(read, "ERR,DXL_TIMEOUT,arm1,3").status, ReplyStatus::kDeviceError);
  EXPECT_EQ(classify_reply(read, "READY,5").status, ReplyStatus::kRestarted);
  EXPECT_EQ(classify_reply(read, "READY").status, ReplyStatus::kRestarted);
}
