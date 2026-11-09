// Exposes the firmware-in-the-loop simulator on a pseudo-terminal, so the
// ros2_control plugin connects through the same PosixSerial path as real hardware.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

#include "firmware_sim.hpp"

namespace
{

volatile std::sig_atomic_t g_running = 1;

void handle_signal(int)
{
  g_running = 0;
}

bool write_all(int fd, std::string_view data)
{
  while (!data.empty()) {
    const ssize_t written = ::write(fd, data.data(), data.size());
    if (written <= 0) {
      return false;
    }
    data.remove_prefix(static_cast<std::size_t>(written));
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string link_path;
  double rate = 600.0;
  for (int index = 1; index + 1 < argc; index += 2) {
    const std::string_view name = argv[index];
    if (name == "--link") {
      link_path = argv[index + 1];
    } else if (name == "--rate") {
      rate = std::strtod(argv[index + 1], nullptr);
    }
  }
  if (link_path.empty()) {
    std::fputs("usage: cm530_mock_serial --link PATH [--rate COUNTS_PER_SECOND]\n", stderr);
    return 2;
  }

  const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0) {
    std::perror("cannot create pseudo-terminal");
    return 1;
  }
  const char * const slave_name = ::ptsname(master);
  // Holding the slave open keeps the master readable between client connections.
  const int slave = slave_name != nullptr ? ::open(slave_name, O_RDWR | O_NOCTTY) : -1;
  if (slave < 0) {
    std::perror("cannot open pseudo-terminal");
    return 1;
  }
  termios settings{};
  ::tcgetattr(slave, &settings);
  ::cfmakeraw(&settings);
  ::tcsetattr(slave, TCSANOW, &settings);

  ::unlink(link_path.c_str());
  if (::symlink(slave_name, link_path.c_str()) != 0) {
    std::perror("cannot create link");
    return 1;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  auto & sim = cm530_test::FirmwareSim::instance();
  const std::string boot = sim.reset_and_boot();
  sim.set_motor_rate(rate);
  if (!write_all(master, boot)) {
    return 1;
  }
  std::printf("CM-530 mock serial ready on %s -> %s\n", link_path.c_str(), slave_name);
  std::fflush(stdout);

  std::array<char, 256> buffer{};
  while (g_running != 0) {
    pollfd descriptor{master, POLLIN, 0};
    if (::poll(&descriptor, 1, 100) <= 0) {
      continue;
    }
    const ssize_t count = ::read(master, buffer.data(), buffer.size());
    if (count <= 0) {
      continue;
    }
    const std::string reply = sim.feed({buffer.data(), static_cast<std::size_t>(count)});
    if (!write_all(master, reply)) {
      break;
    }
  }
  ::unlink(link_path.c_str());
  ::close(slave);
  ::close(master);
  return 0;
}
