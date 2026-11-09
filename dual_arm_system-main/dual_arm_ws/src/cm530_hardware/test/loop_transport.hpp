#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "cm530_hardware/transport.hpp"
#include "firmware_sim.hpp"

namespace cm530_test
{

// In-process link to the real firmware parser, with injectable link faults.
class LoopTransport final : public cm530_hardware::Transport
{
public:
  explicit LoopTransport(FirmwareSim & sim)
  : sim_(sim) {}

  void write_all(std::string_view data, std::chrono::milliseconds) override
  {
    const std::scoped_lock lock(mutex_);
    if (fail_io_) {
      throw cm530_hardware::TransportError("injected I/O failure");
    }
    ++writes_;
    last_written_ = data;
    const std::string reply = sim_.feed(data);
    if (drop_next_reply_) {
      drop_next_reply_ = false;
      return;
    }
    if (override_reply_) {
      rx_ += *override_reply_;
      override_reply_.reset();
      return;
    }
    rx_ += reply;
  }

  std::size_t read_some(std::span<char> buffer, std::chrono::milliseconds timeout) override
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      {
        const std::scoped_lock lock(mutex_);
        if (!rx_.empty()) {
          const std::size_t count = std::min({buffer.size(), rx_.size(), chunk_});
          std::copy_n(rx_.begin(), count, buffer.begin());
          rx_.erase(0, count);
          return count;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::microseconds{200});
    }
  }

  void inject(const std::string & text) { const std::scoped_lock lock(mutex_); rx_ += text; }
  void drop_next_reply() { const std::scoped_lock lock(mutex_); drop_next_reply_ = true; }
  void override_next_reply(const std::string & text) { const std::scoped_lock lock(mutex_); override_reply_ = text; }
  void set_chunk(std::size_t size) { const std::scoped_lock lock(mutex_); chunk_ = size; }
  void fail_io() { const std::scoped_lock lock(mutex_); fail_io_ = true; }
  int writes() const { const std::scoped_lock lock(mutex_); return writes_; }
  std::string last_written() const { const std::scoped_lock lock(mutex_); return last_written_; }

private:
  FirmwareSim & sim_;
  mutable std::mutex mutex_;
  std::string rx_;
  std::string last_written_;
  std::optional<std::string> override_reply_;
  std::size_t chunk_{1024};
  int writes_{0};
  bool drop_next_reply_{false};
  bool fail_io_{false};
};

}  // namespace cm530_test
