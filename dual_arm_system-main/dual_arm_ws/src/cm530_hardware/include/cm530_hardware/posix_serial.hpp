#pragma once

#include <string>

#include "cm530_hardware/transport.hpp"

namespace cm530_hardware
{

// 57600 8N1 raw port, held exclusively so only one process can own the CM-530.
class PosixSerial final : public Transport
{
public:
  explicit PosixSerial(const std::string & path);
  ~PosixSerial() override;
  PosixSerial(const PosixSerial &) = delete;
  PosixSerial & operator=(const PosixSerial &) = delete;

  void write_all(std::string_view data, std::chrono::milliseconds timeout) override;
  std::size_t read_some(std::span<char> buffer, std::chrono::milliseconds timeout) override;

private:
  int fd_{-1};
  std::string path_;
};

// True when the path resolves to a pseudo-terminal under /dev/pts.
bool is_pseudo_terminal(const std::string & path);

}  // namespace cm530_hardware
