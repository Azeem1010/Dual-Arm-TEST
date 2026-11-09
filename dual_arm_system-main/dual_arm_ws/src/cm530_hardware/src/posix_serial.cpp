#include "cm530_hardware/posix_serial.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <format>

namespace cm530_hardware
{
namespace
{

std::string system_message(std::string_view action, const std::string & path)
{
  return std::format("{} {}: {}", action, path, std::strerror(errno));
}

int to_poll_timeout(std::chrono::milliseconds timeout)
{
  return static_cast<int>(std::clamp<std::chrono::milliseconds::rep>(timeout.count(), 0, 60'000));
}

}  // namespace

PosixSerial::PosixSerial(const std::string & path)
: path_(path)
{
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw TransportError(system_message("cannot open", path));
  }
  const auto fail = [this](std::string_view action) {
    const std::string message = system_message(action, path_);
    ::close(fd_);
    fd_ = -1;
    throw TransportError(message);
  };
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    fail("port already owned:");
  }
  if (::ioctl(fd_, TIOCEXCL) != 0) {
    fail("cannot lock");
  }
  termios settings{};
  if (::tcgetattr(fd_, &settings) != 0) {
    fail("cannot read settings of");
  }
  ::cfmakeraw(&settings);
  settings.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CSIZE | CRTSCTS | HUPCL);
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::cfsetispeed(&settings, B57600) != 0 || ::cfsetospeed(&settings, B57600) != 0 ||
    ::tcsetattr(fd_, TCSANOW, &settings) != 0)
  {
    fail("cannot configure");
  }
  // The manual terminal keeps DTR/RTS low; pseudo-terminals do not support this.
  const int lines = TIOCM_DTR | TIOCM_RTS;
  if (::ioctl(fd_, TIOCMBIC, &lines) != 0 && errno != ENOTTY && errno != EINVAL) {
    fail("cannot clear modem lines on");
  }
  ::tcflush(fd_, TCIOFLUSH);
}

PosixSerial::~PosixSerial()
{
  if (fd_ >= 0) {
    ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
  }
}

void PosixSerial::write_all(std::string_view data, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!data.empty()) {
    const ssize_t written = ::write(fd_, data.data(), data.size());
    if (written > 0) {
      data.remove_prefix(static_cast<std::size_t>(written));
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EINTR) {
      throw TransportError(system_message("write failed on", path_));
    }
    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      throw TransportError(std::format("write timed out on {}", path_));
    }
    pollfd descriptor{fd_, POLLOUT, 0};
    if (::poll(&descriptor, 1, to_poll_timeout(remaining)) < 0 && errno != EINTR) {
      throw TransportError(system_message("poll failed on", path_));
    }
  }
}

std::size_t PosixSerial::read_some(std::span<char> buffer, std::chrono::milliseconds timeout)
{
  pollfd descriptor{fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, to_poll_timeout(timeout));
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;
    }
    throw TransportError(system_message("poll failed on", path_));
  }
  if (ready == 0) {
    return 0;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (descriptor.revents & POLLIN) == 0) {
    throw TransportError(std::format("port {} disconnected", path_));
  }
  const ssize_t count = ::read(fd_, buffer.data(), buffer.size());
  if (count < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return 0;
    }
    throw TransportError(system_message("read failed on", path_));
  }
  return static_cast<std::size_t>(count);
}

bool is_pseudo_terminal(const std::string & path)
{
  std::error_code error;
  const auto resolved = std::filesystem::canonical(path, error);
  return !error && resolved.string().starts_with("/dev/pts/");
}

}  // namespace cm530_hardware
