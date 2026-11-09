#include "cm530_hardware/scheduler.hpp"

#include <cstdlib>
#include <format>
#include <utility>

namespace cm530_hardware
{

Scheduler::Scheduler(
  Link & link, SchedulerOptions options, const std::array<RawPose, kArmCount> & initial)
: link_(link), options_(options), last_sent_(initial)
{
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t arm = 0; arm < kArmCount; ++arm) {
    samples_[arm] = ArmSample{initial[arm], now};
  }
}

Scheduler::~Scheduler()
{
  stop();
}

void Scheduler::start()
{
  worker_ = std::jthread([this](std::stop_token stop) {run(std::move(stop));});
}

void Scheduler::stop()
{
  if (worker_.joinable()) {
    worker_.request_stop();
    worker_.join();
  }
}

bool Scheduler::submit(Arm arm, const RawPose & target) noexcept
{
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  targets_[static_cast<std::size_t>(arm)] = target;
  return true;
}

bool Scheduler::snapshot(ArmSamples & samples) const noexcept
{
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  samples = samples_;
  return true;
}

std::string Scheduler::fault() const
{
  const std::scoped_lock lock(mutex_);
  return fault_;
}

void Scheduler::latch(std::string reason)
{
  {
    const std::scoped_lock lock(mutex_);
    fault_ = std::move(reason);
  }
  faulted_.store(true, std::memory_order_release);
}

bool Scheduler::cycle_arm(Arm arm)
{
  const auto index = static_cast<std::size_t>(arm);
  if (options_.enable_motion) {
    std::optional<RawPose> target;
    {
      const std::scoped_lock lock(mutex_);
      target = targets_[index];
    }
    if (target && *target != last_sent_[index]) {
      for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
        const int step = std::abs(int{(*target)[joint]} - int{last_sent_[index][joint]});
        if (step > options_.max_step_raw) {
          latch(std::format(
              "{} joint {} step {} exceeds max_step_raw {}", arm_name(arm), joint + 1, step,
              options_.max_step_raw));
          return false;
        }
      }
      const LinkResult sent = link_.transact(Command::ax(arm, *target));
      if (!sent.ok()) {
        latch(std::format("AX {} failed: {}", arm_name(arm), sent.detail));
        return false;
      }
      last_sent_[index] = *target;
    }
  }
  const LinkResult read = link_.transact(Command::read(arm));
  if (!read.ok() || !read.pose) {
    latch(std::format("READ {} failed: {}", arm_name(arm), read.detail));
    return false;
  }
  const std::scoped_lock lock(mutex_);
  samples_[index] = ArmSample{*read.pose, std::chrono::steady_clock::now()};
  return true;
}

void Scheduler::run(std::stop_token stop)
{
  while (!stop.stop_requested()) {
    const auto started = std::chrono::steady_clock::now();
    for (const Arm arm : kArms) {
      if (stop.stop_requested()) {
        return;
      }
      if (!cycle_arm(arm)) {
        return;
      }
    }
    std::this_thread::sleep_until(started + options_.min_cycle);
  }
}

}  // namespace cm530_hardware
