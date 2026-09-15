#ifndef DOUBLE_ARM_HARDWARE__STM32_MOCK_SERIAL_HPP_
#define DOUBLE_ARM_HARDWARE__STM32_MOCK_SERIAL_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "double_arm_hardware/serial_backend.hpp"
#include "double_arm_hardware/stm32_protocol.hpp"

namespace double_arm_hardware
{

/// Mock serial backend that behaves like the STM32 V1.3.1 firmware:
/// parses each written frame and queues the proper response bytes
/// (ACK for HELLO/ENABLE/STOP/DISABLE/TARGET, STATE for GET_STATE).
/// Zero /dev access — used by the offline Stm32Backend tests.
class MockStm32Serial : public ISerialBackend
{
public:
  bool open(const std::string & device, int baudrate, int timeout_ms) override;
  void close() override;
  void flush_input() override;
  int write_raw(const std::vector<uint8_t> & data) override;
  std::vector<uint8_t> read_raw(size_t size, int timeout_ms) override;
  bool is_open() const override;

  // ── Scenario configuration ────────────────────────────────────

  /// Override the ACK result for EVERY request (default ACK_OK).
  void set_ack_result(uint8_t result) { ack_result_ = result; }

  /// Override the ACK result for ONE command code (takes precedence over
  /// set_ack_result). Used to make e.g. STOP fail while ENABLE succeeds.
  void set_ack_result_for(uint8_t cmd, uint8_t result)
  {
    per_cmd_ack_[cmd] = result;
  }

  /// When true, GET_STATE requests are NOT answered (state timeout
  /// scenarios) while all other commands still reply.
  void set_no_state_response(bool v) { no_state_response_ = v; }

  /// When true, ACK-command requests (HELLO/ENABLE/STOP/DISABLE/TARGET) are
  /// NOT answered while GET_STATE still replies. Used to make ENABLE time
  /// out without starving the STATE polls (startup sync keeps working).
  void set_no_ack_response(bool v) { no_ack_response_ = v; }

  /// Answer GET_STATE normally for the first `n` requests, then go silent
  /// on GET_STATE (deterministic state-timeout injection).
  void set_state_ok_count(int n) { state_ok_count_ = n; }

  /// After an ENABLE command is written, subsequent STATE responses switch
  /// to these values (deterministic post-ENABLE validation scenarios).
  void set_after_enable_state(uint16_t valid, uint8_t control,
                              uint16_t enabled = 0x0FFFU, uint8_t fault = 0)
  {
    after_enable_armed_ = true;
    after_enable_valid_ = valid;
    after_enable_control_ = control;
    after_enable_enabled_ = enabled;
    after_enable_fault_ = fault;
  }

  /// When >= 0, the ACK payload echoes this SEQ instead of the request's
  /// SEQ — used to test cross-SEQ mismatch rejection on the host side.
  void set_ack_seq_override(int16_t seq) { ack_seq_override_ = seq; }

  /// When >= 0, the ACK payload echoes this CMD instead of the request's
  /// CMD — used to test cross-CMD mismatch rejection on the host side.
  void set_ack_cmd_override(int16_t cmd) { ack_cmd_override_ = cmd; }

  /// Inject raw bytes that will be returned by the next read_raw() calls,
  /// BEFORE the mock's own response. Used to test mismatch scenarios such
  /// as a STATE frame whose header and payload SEQ disagree.
  void inject_raw(const std::vector<uint8_t> & bytes)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (uint8_t b : bytes) rx_queue_.push_back(b);
  }

  /// State-frame values used when a GET_STATE is answered.
  void set_state_values(const std::array<int32_t, 12> & pos,
                        uint16_t valid,
                        uint8_t control_state,
                        uint16_t enabled = 0x0FFFU,
                        uint8_t fault = 0,
                        uint32_t axis_status = 0);

  /// When true, no response is queued at all (timeout scenarios).
  void set_no_response(bool v) { no_response_ = v; }

  /// When true, read_raw returns at most 1 byte per call (split packets).
  void set_split_reads(bool v) { split_reads_ = v; }

  /// Simulated serial delay before the queued response becomes readable.
  void set_response_delay_ms(int ms) { response_delay_ms_ = ms; }

  // ── Failure injection ─────────────────────────────────────────
  void set_fail_open(bool v)  { fail_open_ = v; }
  void set_fail_write(bool v) { fail_write_ = v; }
  void set_fail_read(bool v)  { fail_read_ = v; }

  // ── Inspection ────────────────────────────────────────────────
  const std::string & last_device() const { return last_device_; }
  int last_baudrate() const { return last_baudrate_; }
  size_t open_count() const { return open_count_; }
  size_t close_count() const { return close_count_; }
  size_t write_count() const { return write_count_; }
  size_t read_count() const { return read_count_; }

  /// Every request frame that was written, as raw bytes.
  const std::vector<std::vector<uint8_t>> & sent_frames() const
  { return sent_frames_; }

  /// Command codes of every written frame, in order.
  std::vector<uint8_t> sent_commands() const;

  /// Decoded TARGET payload of the last written TARGET frame.
  Stm32Protocol::Target last_target() const;

  /// SEQ of the last written frame.
  uint16_t last_seq() const;

private:
  void queue_response(const std::vector<uint8_t> & bytes);

  bool open_ = false;
  std::string last_device_;
  int last_baudrate_ = 0;

  std::array<int32_t, 12> state_pos_{};
  uint16_t state_valid_ = 0;
  uint8_t  state_control_ = Stm32Protocol::CTRL_DISABLED;
  uint16_t state_enabled_ = 0x0FFFU;
  uint8_t  state_fault_ = 0;
  uint32_t state_axis_status_ = 0;

  uint8_t ack_result_ = Stm32Protocol::ACK_OK;
  int16_t ack_seq_override_ = -1;
  int16_t ack_cmd_override_ = -1;
  bool no_response_ = false;
  bool no_state_response_ = false;
  bool no_ack_response_ = false;
  int state_ok_count_ = -1;
  bool after_enable_armed_ = false;
  uint16_t after_enable_valid_ = 0;
  uint8_t  after_enable_control_ = Stm32Protocol::CTRL_DISABLED;
  uint16_t after_enable_enabled_ = 0x0FFFU;
  uint8_t  after_enable_fault_ = 0;
  bool enable_seen_ = false;
  std::map<uint8_t, uint8_t> per_cmd_ack_;
  bool split_reads_ = false;
  int response_delay_ms_ = 0;

  bool fail_open_ = false;
  bool fail_write_ = false;
  bool fail_read_ = false;

  size_t open_count_ = 0, close_count_ = 0;
  size_t write_count_ = 0, read_count_ = 0;
  std::vector<std::vector<uint8_t>> sent_frames_;
  std::deque<uint8_t> rx_queue_;
  std::chrono::steady_clock::time_point delayed_ready_{};
  std::deque<uint8_t> delayed_queue_;
  mutable std::mutex mtx_;
};

// ── inline impls ────────────────────────────────────────────────────

inline bool MockStm32Serial::open(const std::string & device,
                                  int baudrate, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  last_device_ = device;
  last_baudrate_ = baudrate;
  (void)timeout_ms;
  open_count_++;
  if (fail_open_) return false;
  open_ = true;
  return true;
}

inline void MockStm32Serial::close()
{
  std::lock_guard<std::mutex> lock(mtx_);
  close_count_++;
  open_ = false;
  rx_queue_.clear();
}

inline void MockStm32Serial::flush_input()
{
  std::lock_guard<std::mutex> lock(mtx_);
  rx_queue_.clear();
}

inline void MockStm32Serial::set_state_values(
  const std::array<int32_t, 12> & pos, uint16_t valid,
  uint8_t control_state, uint16_t enabled, uint8_t fault,
  uint32_t axis_status)
{
  std::lock_guard<std::mutex> lock(mtx_);
  state_pos_ = pos;
  state_valid_ = valid;
  state_control_ = control_state;
  state_enabled_ = enabled;
  state_fault_ = fault;
  state_axis_status_ = axis_status;
}

inline void MockStm32Serial::queue_response(const std::vector<uint8_t> & bytes)
{
  if (response_delay_ms_ > 0) {
    // Simulated processing delay: park the bytes until the delay elapses.
    delayed_queue_.clear();
    for (uint8_t b : bytes) delayed_queue_.push_back(b);
    delayed_ready_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(response_delay_ms_);
  } else {
    for (uint8_t b : bytes) rx_queue_.push_back(b);
  }
}

inline int MockStm32Serial::write_raw(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(mtx_);
  sent_frames_.push_back(data);
  write_count_++;
  if (fail_write_) return -1;

  if (no_response_ || data.size() < 8U) {
    return static_cast<int>(data.size());
  }

  const uint8_t cmd = data[3];
  const uint16_t seq = static_cast<uint16_t>(data[4]) |
                       (static_cast<uint16_t>(data[5]) << 8);

  if (cmd == Stm32Protocol::CMD_GET_STATE) {
    if (no_state_response_) {
      return static_cast<int>(data.size());  // never answer GET_STATE
    }
    if (state_ok_count_ >= 0) {
      if (state_ok_count_ == 0) {
        return static_cast<int>(data.size());  // budget exhausted
      }
      --state_ok_count_;
    }
    // After an ENABLE the firmware state may differ — deterministic switch.
    if (enable_seen_ && after_enable_armed_) {
      state_valid_ = after_enable_valid_;
      state_control_ = after_enable_control_;
      state_enabled_ = after_enable_enabled_;
      state_fault_ = after_enable_fault_;
      enable_seen_ = false;  // apply once
    }
    // STATE frame: seq(2) + control(1) + fault(1) + valid(2) + enabled(2)
    // + axis_status(4) + 12×int32(LE) = 60 payload bytes.
    std::vector<uint8_t> payload(Stm32Protocol::STATE_PAYLOAD_LEN, 0);
    payload[0] = static_cast<uint8_t>(seq & 0xFF);
    payload[1] = static_cast<uint8_t>(seq >> 8);
    payload[2] = state_control_;
    payload[3] = state_fault_;
    payload[4] = static_cast<uint8_t>(state_valid_ & 0xFF);
    payload[5] = static_cast<uint8_t>(state_valid_ >> 8);
    payload[6] = static_cast<uint8_t>(state_enabled_ & 0xFF);
    payload[7] = static_cast<uint8_t>(state_enabled_ >> 8);
    for (int b = 0; b < 4; ++b) {
      payload[8 + b] = static_cast<uint8_t>((state_axis_status_ >> (8 * b)) & 0xFF);
    }
    for (int axis = 0; axis < 12; ++axis) {
      const uint32_t v = static_cast<uint32_t>(state_pos_[axis]);
      for (int b = 0; b < 4; ++b) {
        payload[12 + axis * 4 + b] = static_cast<uint8_t>((v >> (8 * b)) & 0xFF);
      }
    }
    queue_response(Stm32Protocol::build_frame(
      Stm32Protocol::RESP_STATE, seq, payload));
  } else {
    if (no_ack_response_) {
      return static_cast<int>(data.size());  // ACK commands never answered
    }
    if (cmd == Stm32Protocol::CMD_ENABLE) {
      enable_seen_ = true;  // post-ENABLE STATE switch armed
    }
    // ACK frame for HELLO / ENABLE / STOP / DISABLE / TARGET.
    auto it = per_cmd_ack_.find(cmd);
    const uint8_t result = (it != per_cmd_ack_.end()) ? it->second : ack_result_;
    const uint16_t ack_seq = (ack_seq_override_ >= 0)
      ? static_cast<uint16_t>(ack_seq_override_) : seq;
    const uint8_t ack_cmd = (ack_cmd_override_ >= 0)
      ? static_cast<uint8_t>(ack_cmd_override_) : cmd;
    std::vector<uint8_t> payload(4, 0);
    payload[0] = static_cast<uint8_t>(ack_seq & 0xFF);
    payload[1] = static_cast<uint8_t>(ack_seq >> 8);
    payload[2] = ack_cmd;
    payload[3] = result;
    queue_response(Stm32Protocol::build_frame(
      Stm32Protocol::RESP_ACK, ack_seq, payload));
  }
  return static_cast<int>(data.size());
}

inline std::vector<uint8_t> MockStm32Serial::read_raw(size_t size, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  read_count_++;
  (void)timeout_ms;
  if (fail_read_) return {};

  // Promote a delayed response to the live queue once its delay elapsed.
  if (response_delay_ms_ > 0 && !delayed_queue_.empty() &&
      std::chrono::steady_clock::now() >= delayed_ready_) {
    rx_queue_ = delayed_queue_;
    delayed_queue_.clear();
  }

  if (rx_queue_.empty()) return {};
  const size_t n = split_reads_ ? 1U : std::min(size, rx_queue_.size());
  std::vector<uint8_t> out(rx_queue_.begin(), rx_queue_.begin() + n);
  rx_queue_.erase(rx_queue_.begin(), rx_queue_.begin() + n);
  return out;
}

inline bool MockStm32Serial::is_open() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return open_;
}

inline std::vector<uint8_t> MockStm32Serial::sent_commands() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::vector<uint8_t> cmds;
  for (const auto & f : sent_frames_) {
    if (f.size() >= 4U) cmds.push_back(f[3]);
  }
  return cmds;
}

inline Stm32Protocol::Target MockStm32Serial::last_target() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  Stm32Protocol::Target t;
  if (sent_frames_.empty()) return t;
  const auto & f = sent_frames_.back();
  if (f.size() < 8U + Stm32Protocol::TARGET_PAYLOAD_LEN) return t;
  Stm32Protocol::decode_target(
    &f[8], Stm32Protocol::TARGET_PAYLOAD_LEN, t);
  return t;
}

inline uint16_t MockStm32Serial::last_seq() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (sent_frames_.empty()) return 0;
  const auto & f = sent_frames_.back();
  if (f.size() < 6U) return 0;
  return static_cast<uint16_t>(f[4]) | (static_cast<uint16_t>(f[5]) << 8);
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__STM32_MOCK_SERIAL_HPP_
