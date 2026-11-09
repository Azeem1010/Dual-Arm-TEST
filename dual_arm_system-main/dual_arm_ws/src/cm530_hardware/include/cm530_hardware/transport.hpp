#pragma once

#include <chrono>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string_view>

namespace cm530_hardware
{

struct TransportError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

class Transport
{
public:
  virtual ~Transport() = default;
  // Both throw TransportError on I/O failure.
  virtual void write_all(std::string_view data, std::chrono::milliseconds timeout) = 0;
  // Returns zero when nothing arrives within the timeout.
  virtual std::size_t read_some(std::span<char> buffer, std::chrono::milliseconds timeout) = 0;
};

}  // namespace cm530_hardware
