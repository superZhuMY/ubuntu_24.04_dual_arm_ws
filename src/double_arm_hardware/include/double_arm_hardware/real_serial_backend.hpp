#ifndef DOUBLE_ARM_HARDWARE__REAL_SERIAL_BACKEND_HPP_
#define DOUBLE_ARM_HARDWARE__REAL_SERIAL_BACKEND_HPP_

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "double_arm_hardware/serial_backend.hpp"

namespace double_arm_hardware
{

/// Real serial backend using POSIX termios.
/// 115200 baud, 8N1, configurable timeout.
class RealSerialBackend : public ISerialBackend
{
public:
  RealSerialBackend() = default;
  ~RealSerialBackend() override { close(); }

  bool open(const std::string & device, int baudrate, int timeout_ms) override;
  void close() override;
  void flush_input() override;
  int  write_raw(const std::vector<uint8_t> & data) override;
  std::vector<uint8_t> read_raw(size_t size, int timeout_ms) override;
  bool is_open() const override;

private:
  static speed_t baud_to_speed(int baudrate);

  int fd_ = -1;
  mutable std::mutex mtx_;
};

// ── inline impls ──────────────────────────────────────────────────

inline bool RealSerialBackend::open(
  const std::string & device, int baudrate, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (fd_ >= 0) return true;

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  struct termios tty{};
  if (tcgetattr(fd_, &tty) < 0) { ::close(fd_); fd_ = -1; return false; }

  cfmakeraw(&tty);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  cfsetispeed(&tty, baud_to_speed(baudrate));
  cfsetospeed(&tty, baud_to_speed(baudrate));

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>((timeout_ms + 99) / 100);

  if (tcsetattr(fd_, TCSANOW, &tty) < 0) { ::close(fd_); fd_ = -1; return false; }
  return true;
}

inline void RealSerialBackend::close()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

inline void RealSerialBackend::flush_input()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

inline int RealSerialBackend::write_raw(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (fd_ < 0) return -1;

  size_t total = 0;
  const uint8_t * ptr = data.data();
  size_t remaining = data.size();

  while (remaining > 0) {
    ssize_t n = ::write(fd_, ptr + total, remaining);
    if (n < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    total += static_cast<size_t>(n);
    remaining -= static_cast<size_t>(n);
  }
  return static_cast<int>(total);
}

inline std::vector<uint8_t> RealSerialBackend::read_raw(
  size_t size, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (fd_ < 0) return {};

  std::vector<uint8_t> buf;
  buf.reserve(size);

  auto deadline = std::chrono::steady_clock::now()
    + std::chrono::milliseconds(timeout_ms);

  while (buf.size() < size) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;
    auto remain_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - now).count();
    if (remain_ms <= 0) break;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv{static_cast<time_t>(remain_ms / 1000),
                      static_cast<suseconds_t>((remain_ms % 1000) * 1000)};

    int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (ret == 0) break; // timeout

    uint8_t tmp[64];
    ssize_t n = ::read(fd_, tmp, std::min(sizeof(tmp), size - buf.size()));
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) break;
    buf.insert(buf.end(), tmp, tmp + n);
  }
  return buf;
}

inline bool RealSerialBackend::is_open() const { return fd_ >= 0; }

inline speed_t RealSerialBackend::baud_to_speed(int baudrate)
{
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return B115200;
  }
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__REAL_SERIAL_BACKEND_HPP_
