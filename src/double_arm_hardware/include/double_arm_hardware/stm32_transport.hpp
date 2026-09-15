#ifndef DOUBLE_ARM_HARDWARE__STM32_TRANSPORT_HPP_
#define DOUBLE_ARM_HARDWARE__STM32_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "double_arm_hardware/serial_backend.hpp"
#include "double_arm_hardware/stm32_protocol.hpp"
#include "double_arm_hardware/stm32_stream_parser.hpp"

namespace double_arm_hardware
{

/// Serial transport for the STM32 whole-robot binary protocol.
///
/// The transport is the only place that talks to the serial line. It
/// guarantees that at most ONE request is in flight at any time: a new
/// request replaces any unsent previous one (latest-wins, matching the
/// firmware's coalescing) and a request that has already been sent is
/// completed or timed out before the next one is sent.
///
/// Response matching (per review task §7):
///   - ACK is accepted only when ALL of these hold:
///       frame.cmd == RESP_ACK
///       frame.header_seq == expected_seq
///       ack.payload_seq == expected_seq
///       ack.cmd == expected_cmd
///       CRC and length valid
///   - STATE is accepted only when ALL of these hold:
///       frame.cmd == RESP_STATE
///       frame.header_seq == expected_seq
///       state.payload_seq == expected_seq
///       CRC valid and LEN == 60
///   Any other legal frame is a stale/mismatch (counted, never completes
///   the current request).
///
/// The serial backend is injected (RealSerialBackend or a mock), so the
/// transport itself never touches /dev. Serial config follows the firmware:
/// 115200, 8 data bits, no parity, 1 stop bit.
class Stm32Transport
{
public:
  /// Injected serial backend (real or mock).
  explicit Stm32Transport(std::unique_ptr<ISerialBackend> serial)
    : serial_(std::move(serial))
  {}

  ~Stm32Transport() { close(); }

  /// Configure device path and timeouts. Does NOT open the device.
  /// The control-ACK default is 10000 ms: ENABLE / STOP / DISABLE are
  /// whole-robot asynchronous sequences whose ACK is only returned when
  /// the sequence finishes (real firmware ≈ 8 s) — 3 s would misjudge a
  /// normal sequence as a timeout.
  bool configure(
    const std::string & device,
    int baud_rate = 115200,
    int target_ack_timeout_ms = 500,
    int state_timeout_ms = 500,
    int control_ack_timeout_ms = 10000);

  /// Open the serial device (idempotent). Never called automatically.
  bool open();

  /// Close the device (idempotent, safe to call repeatedly).
  void close();

  enum class PendingKind : uint8_t { ACK, STATE };

  /// Send a request expecting an ACK back (HELLO/ENABLE/STOP/DISABLE/TARGET).
  /// Blocks up to timeout_ms waiting for an ACK that matches BOTH the
  /// request SEQ and the original command CMD. Returns:
  ///   - ACK_OK on a clean ACK_OK;
  ///   - the ACK result code on any other matching ACK (0x01..0x0B);
  ///   - -1 on timeout / wrong response / transport error.
  /// The payload is built by the caller; cmd/seq are put in the frame here.
  int send_ack_request(
    uint8_t cmd, uint16_t seq, const std::vector<uint8_t> & payload,
    int timeout_ms);

  /// Send GET_STATE expecting a STATE frame back (never an ACK).
  /// Blocks up to timeout_ms waiting for a STATE whose header and payload
  /// SEQ both equal `seq`. Returns:
  ///   - 0 with `st` filled in on success;
  ///   - -1 on timeout / wrong response / transport error.
  int send_get_state(uint16_t seq, int timeout_ms, Stm32Protocol::State & st);

  /// True when the device is open.
  bool is_open() const;

  /// Total serial write failures + protocol-level failures.
  int error_count() const { return error_count_; }

  // ── Response-mismatch diagnostics (§12.3) ──────────────────────
  int ack_seq_mismatch_count() const;
  int ack_cmd_mismatch_count() const;
  int state_seq_mismatch_count() const;

  /// Expose the underlying backend for testing.
  ISerialBackend * serial() { return serial_.get(); }

  /// Current sequence counter (incremented on every sent request).
  uint16_t next_seq() { return ++seq_; }

private:
  /// Read whatever bytes are available and feed the parser. Returns false
  /// on a serial read failure.
  bool pump();

  /// Wait until a frame of the wanted kind that fully matches (header seq,
  /// payload seq, and for ACK also the original CMD) arrives or `deadline`
  /// passes. Consumes all frames seen; returns the matched frame.
  bool wait_for(
    PendingKind kind, uint16_t seq, uint8_t cmd,
    std::chrono::steady_clock::time_point deadline, int & result);

  std::unique_ptr<ISerialBackend> serial_;
  std::string device_;
  int baud_rate_ = 115200;
  int target_ack_timeout_ms_ = 500;
  int state_timeout_ms_ = 500;
  int control_ack_timeout_ms_ = 10000;

  Stm32StreamParser parser_;
  Stm32Protocol::State last_state_{};
  bool configured_ = false;
  bool opened_ = false;
  uint16_t seq_ = 0;
  int error_count_ = 0;
  int ack_seq_mismatch_ = 0;
  int ack_cmd_mismatch_ = 0;
  int state_seq_mismatch_ = 0;
  mutable std::mutex mtx_;
};

// ──────────── inline implementations ───────────────────────────────

inline bool Stm32Transport::configure(
  const std::string & device,
  int baud_rate,
  int target_ack_timeout_ms,
  int state_timeout_ms,
  int control_ack_timeout_ms)
{
  if (device.empty()) return false;
  if (baud_rate <= 0) return false;
  if (target_ack_timeout_ms <= 0 || state_timeout_ms <= 0 ||
      control_ack_timeout_ms <= 0) {
    return false;
  }
  device_ = device;
  baud_rate_ = baud_rate;
  target_ack_timeout_ms_ = target_ack_timeout_ms;
  state_timeout_ms_ = state_timeout_ms;
  control_ack_timeout_ms_ = control_ack_timeout_ms;
  configured_ = true;
  return true;
}

inline bool Stm32Transport::open()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!configured_) return false;
  if (opened_) return true;
  if (!serial_->open(device_, baud_rate_, 50)) return false;
  parser_.reset();
  error_count_ = 0;
  ack_seq_mismatch_ = 0;
  ack_cmd_mismatch_ = 0;
  state_seq_mismatch_ = 0;
  opened_ = true;
  return true;
}

inline void Stm32Transport::close()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!opened_) return;
  serial_->close();
  opened_ = false;
}

inline bool Stm32Transport::is_open() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return opened_;
}

inline int Stm32Transport::ack_seq_mismatch_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ack_seq_mismatch_;
}

inline int Stm32Transport::ack_cmd_mismatch_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ack_cmd_mismatch_;
}

inline int Stm32Transport::state_seq_mismatch_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return state_seq_mismatch_;
}

inline bool Stm32Transport::pump()
{
  // Read up to 64 bytes per call; keep going until the serial buffer is
  // drained (empty read) or a read error occurs.
  for (int round = 0; round < 16; ++round) {
    auto bytes = serial_->read_raw(64, 5);
    if (bytes.empty()) break;
    parser_.append(bytes);
  }
  return true;
}

inline int Stm32Transport::send_ack_request(
  uint8_t cmd, uint16_t seq, const std::vector<uint8_t> & payload,
  int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!opened_) return -1;

  const auto frame = Stm32Protocol::build_frame(cmd, seq, payload);
  if (serial_->write_raw(frame) < 0) {
    ++error_count_;
    return -1;
  }

  int result = -1;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  (void)wait_for(PendingKind::ACK, seq, cmd, deadline, result);
  return result;
}

inline int Stm32Transport::send_get_state(
  uint16_t seq, int timeout_ms, Stm32Protocol::State & st)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!opened_) return -1;

  const auto frame = Stm32Protocol::build_frame(
    Stm32Protocol::CMD_GET_STATE, seq, std::vector<uint8_t>{});
  if (serial_->write_raw(frame) < 0) {
    ++error_count_;
    return -1;
  }

  int result = -1;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  if (!wait_for(PendingKind::STATE, seq, Stm32Protocol::CMD_GET_STATE,
                deadline, result)) {
    return -1;
  }
  st = last_state_;
  return 0;
}

inline bool Stm32Transport::wait_for(
  PendingKind kind, uint16_t seq, uint8_t cmd,
  std::chrono::steady_clock::time_point deadline, int & result)
{
  result = -1;
  last_state_ = Stm32Protocol::State{};

  while (std::chrono::steady_clock::now() < deadline) {
    pump();

    for (auto & f : parser_.poll()) {
      if (!f.ok) {
        ++error_count_;  // CRC / version failure on a received frame
        continue;
      }
      if (f.cmd == Stm32Protocol::RESP_ACK && kind == PendingKind::ACK) {
        Stm32Protocol::Ack ack;
        if (!Stm32Protocol::decode_ack(
              f.payload.data(), static_cast<uint16_t>(f.payload.size()), ack)) {
          ++error_count_;
          continue;
        }
        // Full match: header seq + payload seq + original command CMD.
        if (f.seq != seq || ack.seq != seq) {
          ++ack_seq_mismatch_;
          continue;  // stale / mismatched — never completes this request
        }
        if (ack.cmd != cmd) {
          ++ack_cmd_mismatch_;
          continue;
        }
        result = ack.result;
        return true;
      } else if (f.cmd == Stm32Protocol::RESP_STATE && kind == PendingKind::STATE) {
        Stm32Protocol::State st;
        if (!Stm32Protocol::decode_state(
              f.payload.data(), static_cast<uint16_t>(f.payload.size()), st)) {
          ++error_count_;
          continue;
        }
        // Full match: header seq + payload seq (STATE carries no CMD).
        if (f.seq != seq || st.seq != seq) {
          ++state_seq_mismatch_;
          continue;
        }
        last_state_ = st;
        result = 0;
        return true;
      }
      // Any other frame while waiting is dropped (latest-wins semantics).
    }

    // Small sleep so the mock / real backend has time to produce bytes.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (kind == PendingKind::ACK) {
    ++error_count_;  // timed out waiting for the ACK
  }
  return false;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__STM32_TRANSPORT_HPP_
