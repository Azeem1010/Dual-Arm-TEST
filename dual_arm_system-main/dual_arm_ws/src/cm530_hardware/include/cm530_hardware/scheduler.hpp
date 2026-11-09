#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "cm530_hardware/link.hpp"
#include "cm530_hardware/protocol.hpp"

namespace cm530_hardware
{

struct ArmSample
{
  RawPose raw{};
  std::chrono::steady_clock::time_point stamp{};
};

using ArmSamples = std::array<ArmSample, kArmCount>;

struct SchedulerOptions
{
  bool enable_motion{false};
  int max_step_raw{0};
  std::chrono::milliseconds min_cycle{10};
};

// Owns the serial link while running, keeping blocking I/O off the control
// thread. Any error latches a fault: nothing further is sent, and no command is
// retried. Faults are final for this instance.
class Scheduler
{
public:
  Scheduler(Link & link, SchedulerOptions options, const std::array<RawPose, kArmCount> & initial);
  ~Scheduler();
  Scheduler(const Scheduler &) = delete;
  Scheduler & operator=(const Scheduler &) = delete;

  void start();
  void stop();

  // Non-blocking, for the control thread. A false return means the lock was
  // busy and the caller should retry on its next cycle.
  bool submit(Arm arm, const RawPose & target) noexcept;
  bool snapshot(ArmSamples & samples) const noexcept;

  bool faulted() const noexcept { return faulted_.load(std::memory_order_acquire); }
  std::string fault() const;

private:
  void run(std::stop_token stop);
  bool cycle_arm(Arm arm);
  void latch(std::string reason);

  Link & link_;
  SchedulerOptions options_;
  std::array<RawPose, kArmCount> last_sent_;
  mutable std::mutex mutex_;
  std::array<std::optional<RawPose>, kArmCount> targets_{};
  ArmSamples samples_{};
  std::string fault_;
  std::atomic<bool> faulted_{false};
  std::jthread worker_;
};

}  // namespace cm530_hardware
