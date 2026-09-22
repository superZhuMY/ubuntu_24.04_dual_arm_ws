// Copyright 2026 zmy

#include "double_arm_end_effector/serial_bus.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "double_arm_end_effector/serial_bus_servo_protocol.hpp"

namespace double_arm_end_effector
{

namespace
{
speed_t baud_constant(int baud_rate)
{
  if (baud_rate == 115200) {
    return B115200;
  }
  throw std::invalid_argument("only the servo manual's 115200 baud rate is supported");
}

std::runtime_error io_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}
}  // namespace

SerialBus::~SerialBus()
{
  close_port();
}

void SerialBus::open_port(const std::string & device, int baud_rate)
{
  close_port();
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw io_error("cannot open " + device);
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    const auto error = io_error("tcgetattr failed for " + device);
    ::close(fd);
    throw error;
  }
  cfmakeraw(&tty);
  const speed_t speed = baud_constant(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    const auto error = io_error("tcsetattr failed for " + device);
    ::close(fd);
    throw error;
  }
  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
}

void SerialBus::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialBus::is_open() const
{
  return fd_ >= 0;
}

void SerialBus::write_command(const std::string & command)
{
  if (!is_open()) {
    throw std::runtime_error("serial bus is not open");
  }
  std::size_t sent = 0;
  while (sent < command.size()) {
    const ssize_t count = ::write(fd_, command.data() + sent, command.size() - sent);
    if (count > 0) {
      sent += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EINTR || errno == EAGAIN)) {
      continue;
    }
    throw io_error("servo serial write failed");
  }
  if (tcdrain(fd_) != 0) {
    throw io_error("servo serial tcdrain failed");
  }
}

std::optional<int> SerialBus::read_position(
  int servo_id, std::chrono::milliseconds timeout)
{
  if (!is_open()) {
    throw std::runtime_error("serial bus is not open");
  }
  tcflush(fd_, TCIFLUSH);
  write_command(read_position_command(servo_id));

  std::string receive_buffer;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    pollfd descriptor{fd_, POLLIN, 0};
    const int poll_result = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw io_error("servo serial poll failed");
    }
    if (poll_result == 0) {
      break;
    }
    char chunk[64];
    const ssize_t count = ::read(fd_, chunk, sizeof(chunk));
    if (count > 0) {
      receive_buffer.append(chunk, static_cast<std::size_t>(count));
      if (const auto pulse = parse_position_response(receive_buffer, servo_id)) {
        return pulse;
      }
    } else if (count < 0 && errno != EAGAIN && errno != EINTR) {
      throw io_error("servo serial read failed");
    }
  }
  return std::nullopt;
}

}  // namespace double_arm_end_effector
