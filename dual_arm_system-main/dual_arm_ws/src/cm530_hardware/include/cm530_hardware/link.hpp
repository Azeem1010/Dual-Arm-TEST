#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "cm530_hardware/protocol.hpp"
#include "cm530_hardware/transport.hpp"

namespace cm530_hardware
{

enum class LinkStatus { kOk, kDeviceError, kRestarted, kMismatch, kTimeout, kIoError, kDown };

struct LinkResult
{
  LinkStatus status{LinkStatus::kDown};
  std::optional<RawPose> pose{};
  std::string detail{};

  bool ok() const noexcept { return status == LinkStatus::kOk; }
};

// One request at a time over one port. Not thread-safe: exactly one thread may
// use a Link at any moment. The protocol has no sequence numbers, so after a
// timeout, mismatch or restart a late reply cannot be told from a new one; the
// link therefore latches down and never resends. Recovery means a new Link.
class Link
{
public:
  Link(std::unique_ptr<Transport> transport, std::chrono::milliseconds command_timeout);

  // Optional READY listen followed by the mandatory VERSION and PING.
  LinkResult handshake(std::chrono::milliseconds listen);
  LinkResult transact(const Command & command);
  bool healthy() const noexcept { return !down_; }

private:
  LinkResult fail(LinkStatus status, std::string detail);
  bool fill(std::chrono::steady_clock::time_point deadline);
  std::optional<std::string> pop_line();
  std::optional<std::string> pending_input();

  std::unique_ptr<Transport> transport_;
  std::chrono::milliseconds command_timeout_;
  std::string buffer_;
  bool down_{false};
};

}  // namespace cm530_hardware
