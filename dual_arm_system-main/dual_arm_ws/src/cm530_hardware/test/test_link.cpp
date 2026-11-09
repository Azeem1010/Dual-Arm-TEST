// Every test here talks to the real v17 parser and AX-12 driver code.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "cm530_hardware/link.hpp"
#include "loop_transport.hpp"

using namespace std::chrono_literals;
using cm530_hardware::Arm;
using cm530_hardware::Command;
using cm530_hardware::Link;
using cm530_hardware::LinkStatus;
using cm530_hardware::RawPose;
using cm530_test::FirmwareSim;
using cm530_test::LoopTransport;
using cm530_test::MotorFault;

class LinkTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    boot_output = sim.reset_and_boot();
    auto transport = std::make_unique<LoopTransport>(sim);
    loop = transport.get();
    link = std::make_unique<Link>(std::move(transport), 20ms);
  }

  FirmwareSim & sim{FirmwareSim::instance()};
  LoopTransport * loop{nullptr};
  std::unique_ptr<Link> link;
  std::string boot_output;
};

TEST_F(LinkTest, BootsWithTorqueOffAndReady)
{
  EXPECT_EQ(boot_output, "READY,5\r\n");
  EXPECT_FALSE(sim.motor(17).torque);
}

TEST_F(LinkTest, HandshakesWithOrWithoutReady)
{
  loop->inject(boot_output);
  EXPECT_TRUE(link->handshake(20ms).ok());
  EXPECT_TRUE(link->handshake(2ms).ok());
}

TEST_F(LinkTest, RejectsInitFailureAtHandshake)
{
  loop->inject("ERR,INIT_FAILED\r\n");
  EXPECT_FALSE(link->handshake(20ms).ok());
  EXPECT_FALSE(link->healthy());
}

TEST_F(LinkTest, ReadsFirmwareHomePresets)
{
  const auto arm1 = link->transact(Command::get_home(Arm::kArm1));
  ASSERT_TRUE(arm1.ok());
  EXPECT_EQ(arm1.pose, (RawPose{90, 567, 712, 825}));
  const auto arm2 = link->transact(Command::get_home(Arm::kArm2));
  ASSERT_TRUE(arm2.ok());
  EXPECT_EQ(arm2.pose, (RawPose{590, 567, 712, 825}));
  EXPECT_EQ(sim.goal_writes(), 0);
}

TEST_F(LinkTest, MapsJointOrderToMotorIds)
{
  ASSERT_TRUE(link->transact(Command::ax(Arm::kArm2, RawPose{100, 200, 300, 400})).ok());
  EXPECT_EQ(sim.motor(12).goal, 100);
  EXPECT_EQ(sim.motor(1).goal, 200);
  EXPECT_EQ(sim.motor(8).goal, 300);
  EXPECT_EQ(sim.motor(16).goal, 400);
  EXPECT_EQ(sim.motor(17).goal, 512);

  sim.set_present(17, 11);
  sim.set_present(3, 22);
  sim.set_present(2, 33);
  sim.set_present(15, 44);
  const auto read = link->transact(Command::read(Arm::kArm1));
  ASSERT_TRUE(read.ok());
  EXPECT_EQ(read.pose, (RawPose{11, 22, 33, 44}));
}

TEST_F(LinkTest, FirmwareRequiresTargetBeforeTorque)
{
  const auto refused = link->transact(Command::torque_enable(Arm::kArm1, true));
  EXPECT_EQ(refused.status, LinkStatus::kDeviceError);
  EXPECT_EQ(refused.detail, "ERR,NO_TARGET,arm1");
  EXPECT_TRUE(link->healthy());
  EXPECT_FALSE(sim.motor(17).torque);

  ASSERT_TRUE(link->transact(Command::ax(Arm::kArm1, RawPose{512, 512, 512, 512})).ok());
  ASSERT_TRUE(link->transact(Command::torque_enable(Arm::kArm1, true)).ok());
  EXPECT_TRUE(sim.motor(17).torque);
  EXPECT_FALSE(sim.motor(12).torque);
}

TEST_F(LinkTest, ReportsMotorFaultsWithoutPartialPose)
{
  sim.set_fault(3, MotorFault::kNoReply);
  const auto timeout = link->transact(Command::read(Arm::kArm1));
  EXPECT_EQ(timeout.status, LinkStatus::kDeviceError);
  EXPECT_EQ(timeout.detail, "ERR,DXL_TIMEOUT,arm1,3");
  EXPECT_FALSE(timeout.pose);
  EXPECT_TRUE(link->healthy());

  sim.set_fault(3, MotorFault::kNone);
  sim.set_fault(15, MotorFault::kOverheat);
  const auto overheat = link->transact(Command::read(Arm::kArm1));
  EXPECT_EQ(overheat.detail, "ERR,DXL_MOTOR,arm1,15,4");
  EXPECT_TRUE(link->transact(Command::read(Arm::kArm2)).ok());
}

TEST_F(LinkTest, TimeoutLatchesAndNeverResends)
{
  loop->drop_next_reply();
  EXPECT_EQ(link->transact(Command::ax(Arm::kArm1, RawPose{1, 2, 3, 4})).status, LinkStatus::kTimeout);
  EXPECT_FALSE(link->healthy());
  const int writes = loop->writes();
  EXPECT_EQ(link->transact(Command::read(Arm::kArm1)).status, LinkStatus::kDown);
  EXPECT_EQ(loop->writes(), writes);
}

TEST_F(LinkTest, WrongOrRestartRepliesLatch)
{
  loop->override_next_reply("OK,AX,arm2\r\n");
  EXPECT_EQ(link->transact(Command::ax(Arm::kArm1, RawPose{1, 2, 3, 4})).status, LinkStatus::kMismatch);
  EXPECT_FALSE(link->healthy());

  SetUp();
  loop->override_next_reply("READY,5\r\n");
  EXPECT_EQ(link->transact(Command::read(Arm::kArm1)).status, LinkStatus::kRestarted);
  EXPECT_FALSE(link->healthy());

  SetUp();
  loop->override_next_reply("ERR,OVERFLOW\r\n");
  EXPECT_EQ(link->transact(Command::read(Arm::kArm1)).status, LinkStatus::kDeviceError);
  EXPECT_FALSE(link->healthy());
}

TEST_F(LinkTest, StaleInputBlocksTheNextRequest)
{
  loop->inject("OK,AX,arm1\r\n");
  const int writes = loop->writes();
  EXPECT_EQ(link->transact(Command::read(Arm::kArm1)).status, LinkStatus::kMismatch);
  EXPECT_EQ(loop->writes(), writes);
}

TEST_F(LinkTest, ToleratesLeftoverLineEndingsAndFragments)
{
  loop->inject("\n");
  loop->set_chunk(1);
  const auto read = link->transact(Command::read(Arm::kArm1));
  ASSERT_TRUE(read.ok());
  EXPECT_EQ(read.pose, (RawPose{512, 512, 512, 512}));
  EXPECT_TRUE(link->transact(Command::ping()).ok());
}

TEST_F(LinkTest, TransportFailureLatches)
{
  loop->fail_io();
  EXPECT_EQ(link->transact(Command::ping()).status, LinkStatus::kIoError);
  EXPECT_FALSE(link->healthy());
}
