#ifndef DOUBLE_ARM_HARDWARE__SERIAL_BACKEND_HPP_
#define DOUBLE_ARM_HARDWARE__SERIAL_BACKEND_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace double_arm_hardware
{

/// Abstract serial port backend for J4-J6 motor communication.
///
/// RealSerialBackend  — termios-based, only instantiated in REAL mode.
/// MockSerialBackend — recording fake, used in tests.
class ISerialBackend
{
public:
  virtual ~ISerialBackend() = default;

  /// Open serial port at given baudrate with timeout.
  /// Baudrate 115200, 8N1 per original firmware.
  /// @return true on success.
  virtual bool open(const std::string & device, int baudrate, int timeout_ms) = 0;

  /// Close port and release resources. Idempotent.
  virtual void close() = 0;

  /// Flush input buffer (replicates serial_client.flushInput()).
  virtual void flush_input() = 0;

  /// Write raw bytes to port.
  /// @return number of bytes written, or -1 on error.
  virtual int write_raw(const std::vector<uint8_t> & data) = 0;

  /// Read up to `size` bytes with timeout.
  /// Returns whatever is available (may be less than `size`).
  virtual std::vector<uint8_t> read_raw(size_t size, int timeout_ms) = 0;

  /// Whether the port is currently open.
  virtual bool is_open() const = 0;
};

// ═══════════════════════════════════════════════════════════════════
//  MockSerialBackend — records calls, returns canned responses
// ═══════════════════════════════════════════════════════════════════

class MockSerialBackend : public ISerialBackend
{
public:
  bool open(const std::string & device, int baudrate, int timeout_ms) override;
  void close() override;
  void flush_input() override;
  int write_raw(const std::vector<uint8_t> & data) override;
  std::vector<uint8_t> read_raw(size_t size, int timeout_ms) override;
  bool is_open() const override;

  // ── Test helpers ──────────────────────────────────────────────

  /// Set canned response for the next read_raw() call.
  void set_next_response(const std::vector<uint8_t> & resp)
  { next_response_ = resp; }

  /// Enable/disable simulated failures.
  void set_fail_open(bool v)    { fail_open_ = v; }
  void set_fail_write(bool v)   { fail_write_ = v; }
  void set_fail_read(bool v)    { fail_read_ = v; }
  void set_short_read(bool v)   { short_read_ = v; }

  // State inspection
  const std::string & last_device()   const { return last_device_; }
  int last_baudrate()                 const { return last_baudrate_; }
  int last_timeout_ms()               const { return last_timeout_ms_; }
  const std::vector<uint8_t> & last_write_data() const { return last_write_; }
  size_t open_count()                 const { return open_count_; }
  size_t close_count()                const { return close_count_; }
  size_t flush_count()                const { return flush_count_; }
  size_t write_count()                const { return write_count_; }
  size_t read_count()                 const { return read_count_; }
  bool was_closed()                   const { return was_closed_; }

private:
  bool open_ = false;
  std::string last_device_;
  int last_baudrate_ = 0;
  int last_timeout_ms_ = 0;
  std::vector<uint8_t> last_write_;
  std::vector<uint8_t> next_response_;
  size_t open_count_ = 0, close_count_ = 0, flush_count_ = 0;
  size_t write_count_ = 0, read_count_ = 0;
  bool was_closed_ = false;
  bool fail_open_ = false, fail_write_ = false;
  bool fail_read_ = false, short_read_ = false;
};

// ── inline impls ────────────────────────────────────────────────

inline bool MockSerialBackend::open(const std::string & device,
  int baudrate, int timeout_ms)
{
  last_device_ = device;
  last_baudrate_ = baudrate;
  last_timeout_ms_ = timeout_ms;
  open_count_++;
  if (fail_open_) return false;
  open_ = true;
  return true;
}

inline void MockSerialBackend::close()
{
  was_closed_ = true;
  open_ = false;
  close_count_++;
}

inline void MockSerialBackend::flush_input()
{
  flush_count_++;
}

inline int MockSerialBackend::write_raw(const std::vector<uint8_t> & data)
{
  last_write_ = data;
  write_count_++;
  if (fail_write_) return -1;
  return static_cast<int>(data.size());
}

inline std::vector<uint8_t> MockSerialBackend::read_raw(size_t /*size*/, int /*timeout_ms*/)
{
  read_count_++;
  if (fail_read_) return {};
  if (short_read_ && !next_response_.empty())
    return {next_response_.begin(),
            next_response_.begin() + std::min(size_t(5), next_response_.size())};
  return next_response_;
}

inline bool MockSerialBackend::is_open() const { return open_; }

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__SERIAL_BACKEND_HPP_
