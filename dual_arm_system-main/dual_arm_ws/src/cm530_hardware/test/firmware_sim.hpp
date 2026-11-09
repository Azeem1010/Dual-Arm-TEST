#pragma once

// Runs the unmodified v17 bridge.c, ax12.c and dynamixel.c against a simulated
// AX-12A bus. Only the physical HAL is simulated. The firmware uses globals, so
// there is exactly one instance per process.

#include <string>
#include <string_view>

namespace cm530_test
{

enum class MotorFault { kNone, kNoReply, kOverheat };

struct Motor
{
  double present{512.0};
  int goal{512};
  bool torque{false};
  MotorFault fault{MotorFault::kNone};
};

class FirmwareSim
{
public:
  static FirmwareSim & instance();

  // Restores eight motors to raw 512 with torque off, then boots the firmware.
  std::string reset_and_boot();
  // Feeds host bytes to the firmware parser and returns its output.
  std::string feed(std::string_view bytes);

  Motor motor(int id) const;
  void set_present(int id, int raw);
  void set_fault(int id, MotorFault fault);
  // Counts per second toward the goal; zero moves instantly.
  void set_motor_rate(double counts_per_second);
  int goal_writes() const;
  int torque_writes() const;

private:
  FirmwareSim() = default;
};

}  // namespace cm530_test
