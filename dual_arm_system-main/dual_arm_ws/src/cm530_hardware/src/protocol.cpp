#include "cm530_hardware/protocol.hpp"

#include <charconv>
#include <format>
#include <vector>

namespace cm530_hardware
{
namespace
{

std::vector<std::string_view> split_fields(std::string_view line)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string_view::npos) {
      fields.push_back(line.substr(start));
      return fields;
    }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
}

// Unsigned parsing rejects signs; the full-consumption check rejects decimals.
std::optional<std::uint16_t> parse_raw(std::string_view text)
{
  unsigned value = 0;
  const char * const end = text.data() + text.size();
  const auto [ptr, error] = std::from_chars(text.data(), end, value);
  if (text.empty() || error != std::errc{} || ptr != end || value > static_cast<unsigned>(kMaxRaw)) {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>(value);
}

Reply match_pose(std::string_view line, std::string_view tag, Arm arm)
{
  const auto fields = split_fields(line);
  if (fields.size() != 2 + kJointsPerArm || fields[0] != tag || fields[1] != arm_name(arm)) {
    return {ReplyStatus::kMismatch, std::nullopt};
  }
  RawPose pose{};
  for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
    const auto value = parse_raw(fields[joint + 2]);
    if (!value) {
      return {ReplyStatus::kMismatch, std::nullopt};
    }
    pose[joint] = *value;
  }
  return {ReplyStatus::kOk, pose};
}

Reply match_exact(std::string_view line, std::string_view expected)
{
  return {line == expected ? ReplyStatus::kOk : ReplyStatus::kMismatch, std::nullopt};
}

}  // namespace

std::string_view arm_name(Arm arm) noexcept
{
  return arm == Arm::kArm1 ? "arm1" : "arm2";
}

std::string format_command(const Command & command)
{
  const std::string_view arm = arm_name(command.arm);
  switch (command.kind) {
    case CommandKind::kPing:
      return "PING\n";
    case CommandKind::kVersion:
      return "VERSION\n";
    case CommandKind::kGetHome:
      return std::format("GET_HOME,{}\n", arm);
    case CommandKind::kRead:
      return std::format("READ,{}\n", arm);
    case CommandKind::kHold:
      return std::format("HOLD,{}\n", arm);
    case CommandKind::kAx:
      return std::format(
        "AX,{},{},{},{},{}\n", arm, command.pose[0], command.pose[1], command.pose[2],
        command.pose[3]);
    case CommandKind::kTorque:
      return std::format("TORQUE,{},{}\n", arm, command.torque ? 1 : 0);
  }
  return {};
}

Reply classify_reply(const Command & command, std::string_view line)
{
  if (line == "READY" || line.starts_with("READY,")) {
    return {ReplyStatus::kRestarted, std::nullopt};
  }
  if (line.starts_with("ERR,")) {
    return {ReplyStatus::kDeviceError, std::nullopt};
  }
  const std::string_view arm = arm_name(command.arm);
  switch (command.kind) {
    case CommandKind::kPing:
      return match_exact(line, "PONG");
    case CommandKind::kVersion:
      return match_exact(line, "VERSION,5");
    case CommandKind::kGetHome:
      return match_pose(line, "HOME", command.arm);
    case CommandKind::kRead:
      return match_pose(line, "POS", command.arm);
    case CommandKind::kHold:
      return match_exact(line, std::format("OK,HOLD,{}", arm));
    case CommandKind::kAx:
      return match_exact(line, std::format("OK,AX,{}", arm));
    case CommandKind::kTorque:
      return match_exact(line, std::format("OK,TORQUE,{},{}", arm, command.torque ? 1 : 0));
  }
  return {ReplyStatus::kMismatch, std::nullopt};
}

}  // namespace cm530_hardware
