#include "cm530_hardware/link.hpp"

#include <array>
#include <exception>
#include <format>
#include <utility>

namespace cm530_hardware
{
namespace
{

constexpr std::size_t kMaxBuffered = 256;

bool is_eol(char value) noexcept
{
  return value == '\r' || value == '\n';
}

}  // namespace

Link::Link(std::unique_ptr<Transport> transport, std::chrono::milliseconds command_timeout)
: transport_(std::move(transport)), command_timeout_(command_timeout)
{
}

LinkResult Link::fail(LinkStatus status, std::string detail)
{
  down_ = true;
  return {status, std::nullopt, std::move(detail)};
}

bool Link::fill(std::chrono::steady_clock::time_point deadline)
{
  const auto remaining =
    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
  if (remaining.count() < 0) {
    return false;
  }
  std::array<char, 128> chunk{};
  const std::size_t count = transport_->read_some(chunk, remaining);
  buffer_.append(chunk.data(), count);
  return true;
}

std::optional<std::string> Link::pop_line()
{
  std::size_t start = 0;
  while (start < buffer_.size() && is_eol(buffer_[start])) {
    ++start;
  }
  std::size_t end = start;
  while (end < buffer_.size() && !is_eol(buffer_[end])) {
    ++end;
  }
  if (end == buffer_.size()) {
    buffer_.erase(0, start);
    return std::nullopt;
  }
  std::string line = buffer_.substr(start, end - start);
  buffer_.erase(0, end + 1);
  return line;
}

// Anything but line endings queued before a request is an unsolicited or stale reply.
std::optional<std::string> Link::pending_input()
{
  std::array<char, 128> chunk{};
  for (int reads = 0; reads < 8; ++reads) {
    const std::size_t count = transport_->read_some(chunk, std::chrono::milliseconds{0});
    if (count == 0) {
      break;
    }
    buffer_.append(chunk.data(), count);
  }
  for (const char value : buffer_) {
    if (!is_eol(value)) {
      return buffer_;
    }
  }
  buffer_.clear();
  return std::nullopt;
}

LinkResult Link::handshake(std::chrono::milliseconds listen)
{
  if (down_) {
    return {LinkStatus::kDown, std::nullopt, "link is down"};
  }
  // READY is only sent at board boot, so its absence is not a failure.
  try {
    const auto deadline = std::chrono::steady_clock::now() + listen;
    while (true) {
      if (auto line = pop_line()) {
        if (*line == "READY,5") {
          break;
        }
        return fail(LinkStatus::kMismatch, std::format("unexpected startup line: {}", *line));
      }
      if (std::chrono::steady_clock::now() >= deadline || !fill(deadline)) {
        break;
      }
    }
  } catch (const std::exception & error) {
    return fail(LinkStatus::kIoError, error.what());
  }
  if (LinkResult version = transact(Command::version()); !version.ok()) {
    return version;
  }
  return transact(Command::ping());
}

LinkResult Link::transact(const Command & command)
{
  if (down_) {
    return {LinkStatus::kDown, std::nullopt, "link is down"};
  }
  try {
    if (auto stale = pending_input()) {
      return fail(LinkStatus::kMismatch, std::format("unsolicited input before request: {}", *stale));
    }
    transport_->write_all(format_command(command), command_timeout_);
    const auto deadline = std::chrono::steady_clock::now() + command_timeout_;
    while (true) {
      if (auto line = pop_line()) {
        Reply reply = classify_reply(command, *line);
        switch (reply.status) {
          case ReplyStatus::kOk:
            return {LinkStatus::kOk, reply.pose, std::move(*line)};
          case ReplyStatus::kDeviceError:
            if (line->starts_with("ERR,OVERFLOW")) {
              return fail(LinkStatus::kDeviceError, std::move(*line));
            }
            return {LinkStatus::kDeviceError, std::nullopt, std::move(*line)};
          case ReplyStatus::kRestarted:
            return fail(LinkStatus::kRestarted, std::move(*line));
          case ReplyStatus::kMismatch:
            return fail(LinkStatus::kMismatch, std::format("unexpected reply: {}", *line));
        }
      }
      if (buffer_.size() > kMaxBuffered) {
        return fail(LinkStatus::kMismatch, "response line too long");
      }
      if (std::chrono::steady_clock::now() >= deadline || !fill(deadline)) {
        return fail(LinkStatus::kTimeout, "no reply before the deadline; result unknown");
      }
    }
  } catch (const std::exception & error) {
    return fail(LinkStatus::kIoError, error.what());
  }
}

}  // namespace cm530_hardware
