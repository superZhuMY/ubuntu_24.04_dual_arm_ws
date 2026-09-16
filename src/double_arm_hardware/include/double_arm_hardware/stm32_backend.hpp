#ifndef DOUBLE_ARM_HARDWARE__STM32_BACKEND_HPP_
#define DOUBLE_ARM_HARDWARE__STM32_BACKEND_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "double_arm_hardware/robot_backend.hpp"
#include "double_arm_hardware/stm32_protocol.hpp"
#include "double_arm_hardware/stm32_transport.hpp"

namespace double_arm_hardware
{

/// Whole-robot backend for the AIMotor_F407 STM32 (V1.3.1 firmware).
///
/// One STM32 behind one serial port:
///   - a TARGET frame addresses one arm (6 joints) — µm for J1..J3,
///     µrad for J4..J6;
///   - a STATE frame returns all 12 axes (L_J1..L_J6, R_J1..R_J6);
///   - ENABLE / STOP / DISABLE are whole-robot control sequences whose
///     ACK is returned only when the sequence finishes (delayed ACK).
///
/// Non-blocking for ros2_control:
///   - write_targets() overwrites the latest-command cache and returns
///     immediately (latest-wins, never an unbounded queue);
///   - read_state() copies the latest state snapshot and returns
///     immediately;
///   - a dedicated I/O thread (or an explicit service_step() in tests)
///     sends the latest left TARGET, then the latest right TARGET, then
///     GET_STATE — only the freshest target per arm is ever transmitted;
///     an in-flight transaction uses an immutable snapshot while new
///     targets only replace the next-to-send one.
///
/// Run modes (review task §3):
///   READ_ONLY      — only GET_STATE is ever sent; targets are refused even
///                    if write_targets() is called by mistake.
///   ACTIVE_CONTROL — the full send cycle, gated by the target gate AND
///                    link freshness AND control state / enabled bitmap /
///                    fault / target-error escalation.
///
/// Health layering (review task §8): a single boolean is not enough. The
/// backend derives a BackendHealth from the latest STATE plus the run mode
/// and error escalation, and refuses new targets unless every motion
/// precondition holds.
///
/// SI conversion is the ONLY conversion done here (no pulse / gear-ratio /
/// differential / sign correction — those live inside the STM32):
///   write: wire_value = (zero_offset + direction * ros_value) * 1e6
///   read:  ros_value  = direction * (wire_value * 1e-6 - zero_offset)
/// where direction is exactly +1 or -1.  Pulse, gear-ratio and wrist
/// coupling conversions remain exclusively in the STM32 firmware.
class Stm32Backend : public IRobotBackend
{
public:
  /// Run mode of the I/O loop (review task §3.4).
  enum class IoMode : uint8_t { READ_ONLY, ACTIVE_CONTROL };

  /// Layered health (review task §8.2).
  enum class Health : uint8_t
  {
    OK,                      ///< motion mode, everything fresh and enabled
    READ_ONLY,               ///< read-only mode — nominal, but targets blocked
    STATE_STALE,             ///< no full-valid STATE within the stale window
    CONTROL_NOT_ENABLED,     ///< control_state != ENABLED (motion mode)
    ENABLE_BITMAP_INCOMPLETE,///< enabled_bitmap != 0x0FFF (motion mode)
    FIRMWARE_FAULT,          ///< STATE fault byte != 0
    TARGET_REJECTED,         ///< TARGET errors escalated → gate closed
    TRANSPORT_ERROR          ///< serial/protocol errors escalated
  };

  /// Timeouts in ms; poll period derived from state_poll_hz.
  struct Timeouts
  {
    int target_ack_ms = 200;
    int state_ms = 200;
    /// ENABLE / STOP / DISABLE are whole-robot asynchronous sequences whose
    /// ACK is only sent when the sequence completes — on the real firmware
    /// this can take ~8 s (see stm32_motor_cli.py CONTROL_ACK_TIMEOUT_S).
    /// 3 s would misjudge a normal sequence as a timeout, so default 10 s.
    int control_ack_ms = 10000;
    double state_poll_hz = 20.0;
    /// Whole-robot feedback staleness threshold: if no fully-valid STATE
    /// (valid_bitmap == 0x0FFF) arrives within this window, the link is
    /// considered unhealthy and new targets are refused.
    int state_stale_ms = 1000;
  };

  /// @param transport  Serial transport (real or mock backend inside).
  /// @param gate_passed True once the REAL gate (hardware_mode=real +
  ///                    enable_hardware + allow_hardware_io) has passed;
  ///                    connect() refuses to open devices otherwise.
  /// @param timeouts   Per-command timeouts.
  Stm32Backend(std::shared_ptr<Stm32Transport> transport,
               bool gate_passed,
               const Timeouts & timeouts);

  ~Stm32Backend() override { close(); }

  // ── IRobotBackend ─────────────────────────────────────────────
  bool configure(const HardwareConfig & cfg) override;
  bool connect() override;
  bool read_state(RobotState & out) override;
  bool write_targets(const std::array<double, 12> & targets) override;
  bool enable() override;
  bool stop() override;
  bool disable() override;
  void close() noexcept override;

  // ── Run mode / gates ──────────────────────────────────────────
  /// Select the I/O loop run mode. READ_ONLY sends only GET_STATE.
  void set_io_mode(IoMode mode);
  IoMode io_mode() const;

  /// Permit/forbid target transmission. Kept false until the activate
  /// sequence has completed ENABLE and the operator gate — even a stray
  /// write_targets() call is dropped, never transmitted.
  void set_targets_allowed(bool allowed);
  bool targets_allowed() const;

  /// True when the target gate has been closed by error escalation
  /// (TARGET rejected / transport errors). The reason is in stats().
  bool target_escalated() const;

  /// Reset to a clean post-construction state (used by on_cleanup).
  void reset();

  // ── State accessors (lock-free snapshots under the cache lock) ─
  uint8_t control_state() const;
  uint16_t enabled() const;
  uint8_t fault() const;

  /// Derived layered health (see enum). Lock-protected.
  Health health() const;

  /// True when a fully-valid STATE arrived within state_stale_ms.
  bool link_healthy() const;

  // ── Startup synchronisation primitives ────────────────────────
  /// HELLO handshake + GET_STATE until valid_bitmap == 0x0FFF.
  /// Rate-limited to state_poll_hz so it never hammers the serial line.
  /// @return true when all 12 axes report fresh feedback within `timeout_ms`.
  bool wait_all_valid(int timeout_ms);

  /// Single read-only GET_STATE (no enable, no target). Updates the cache.
  bool refresh_state(int timeout_ms);

  /// Copy the latest 12-axis feedback (SI) into `out` (same as read_state).
  bool current_feedback(std::array<double, 12> & out);

  /// Start the I/O thread with the given run mode. Idempotent; no-op when
  /// already running. Uses deadline scheduling: if a round takes longer
  /// than the poll period it is NOT followed by an extra sleep (overrun
  /// counted) — the loop never adds latency on top of slow communication.
  void start_io(IoMode mode);

  /// Stop the I/O thread and join it. Idempotent.
  void stop_io();

  /// Run one full I/O round according to the current mode.
  /// Exposed for offline tests.
  void service_step();

  // ── Diagnostics / statistics (§12.3) ──────────────────────────
  struct Stats
  {
    uint64_t target_ack_ok[2] = {0, 0};      // per arm
    uint64_t target_timeout[2] = {0, 0};
    uint64_t target_superseded[2] = {0, 0};
    uint64_t target_rejected[2] = {0, 0};    // validation failures
    uint64_t target_ctrl_busy[2] = {0, 0};
    std::array<uint64_t, 16> target_result{};  // aggregate by ACK code 0..15
    uint64_t ack_seq_mismatch = 0;           // from transport
    uint64_t ack_cmd_mismatch = 0;
    uint64_t state_seq_mismatch = 0;
    uint64_t state_received = 0;
    uint64_t state_timeout = 0;
    uint64_t crc_errors = 0;                 // from transport error_count delta
    uint64_t transport_errors = 0;
    uint64_t io_overruns = 0;                // rounds exceeding the period
    uint64_t total_loops = 0;
    double last_loop_period_us = 0.0;
    double max_loop_period_us = 0.0;
    double target_hz[2] = {0.0, 0.0};        // measured over the last window
    double state_hz = 0.0;
    std::string gate_close_reason;           // last reason the gate closed
  };

  Stats stats() const;

  /// True when connect() succeeded and the device is open.
  bool is_connected() const;

  /// True when the I/O thread is running.
  bool is_running() const;

private:
  // ── axis map ──────────────────────────────────────────────────
  struct AxisSlot
  {
    bool configured = false;
    std::string name;
    double zero_offset = 0.0;
    double direction = 1.0;
  };

  bool configured_ = false;
  std::array<AxisSlot, 12> axis_map_;
  bool gate_passed_ = false;
  bool connected_ = false;
  Timeouts timeouts_;

  std::shared_ptr<Stm32Transport> transport_;
  std::atomic<bool> running_{false};
  std::thread io_thread_;
  IoMode io_mode_ = IoMode::READ_ONLY;

  // ── caches ────────────────────────────────────────────────────
  mutable std::mutex cache_mtx_;
  std::array<double, 12> latest_command_{};   // SI units, always valid
  std::array<bool, 2> arm_dirty_{false, false};  // left / right new target
  // Last requested wire values.  ros2_control calls write() continuously;
  // compare after SI->integer conversion so an unchanged target does not
  // repeatedly restart a firmware point-to-point transaction.
  std::array<int32_t, 12> latest_wire_command_{};
  std::array<bool, 2> arm_command_initialized_{false, false};
  std::array<double, 12> latest_state_{};     // SI units, last known value
  std::array<bool, 12> state_valid_{};
  uint8_t control_state_ = Stm32Protocol::CTRL_DISABLED;
  uint16_t enabled_ = 0;
  uint8_t fault_ = 0;
  bool got_state_ = false;
  bool targets_allowed_ = false;
  bool escalated_ = false;
  std::string gate_close_reason_;
  std::chrono::steady_clock::time_point last_state_update_{};
  std::chrono::steady_clock::time_point last_all_valid_time_{};

  // ── target error escalation ───────────────────────────────────
  std::array<int, 2> target_err_count_{0, 0};

  // ── stats ─────────────────────────────────────────────────────
  Stats stats_;
  // Rate measurement baseline (counters + time when last sampled).
  struct RateBase
  {
    std::chrono::steady_clock::time_point t{};
    uint64_t target[2] = {0, 0};
    uint64_t state = 0;
  };
  mutable RateBase rate_base_;

  // ── helpers ───────────────────────────────────────────────────
  bool send_arm_target(uint8_t arm);
  bool send_get_state_request();

  /// Merge a freshly received STATE frame into the caches. Caller holds
  /// cache_mtx_. Axes whose valid bit is 0 keep their last known value but
  /// are marked not-fresh — the placeholder 0 is never published.
  void apply_state_locked(const Stm32Protocol::State & st);

  /// Start sequence: HELLO handshake. Requires an open transport.
  bool handshake();

  /// Run a control sequence command (ENABLE/STOP/DISABLE) to completion.
  bool run_control(uint8_t cmd);

  /// All motion preconditions for sending a TARGET, evaluated under the
  /// cache lock: gate open, mode active, fresh link, control_state ENABLED,
  /// enabled bitmap full, fault clear, no escalation.
  bool motion_ready_locked() const;

  /// Record a TARGET ACK result (arm, res) under the cache lock and
  /// escalate when the consecutive error count crosses the threshold.
  void record_target_result_locked(uint8_t arm, int res);

  /// Close the target gate with a reason (under the cache lock).
  void close_gate_locked(const std::string & reason);
};

// ──────────── inline implementations ───────────────────────────────

inline Stm32Backend::Stm32Backend(
  std::shared_ptr<Stm32Transport> transport,
  bool gate_passed,
  const Timeouts & timeouts)
  : gate_passed_(gate_passed)
  , timeouts_(timeouts)
  , transport_(std::move(transport))
{}

inline bool Stm32Backend::configure(const HardwareConfig & cfg)
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  if (cfg.axes.size() != 12U) return false;
  for (int i = 0; i < 12; ++i) {
    AxisSlot & s = axis_map_[i];
    s.configured = true;
    s.name = cfg.axes[i].name;
    s.zero_offset = cfg.axes[i].zero_offset;
    s.direction = cfg.axes[i].direction;
    if (s.direction != 1.0 && s.direction != -1.0) return false;
  }
  configured_ = true;
  return true;
}

inline bool Stm32Backend::connect()
{
  if (!configured_) return false;
  if (!gate_passed_) return false;  // REAL gate incomplete — no device access
  if (connected_) return true;

  if (!transport_->open()) return false;

  // HELLO handshake — the first command after opening the port.
  if (!handshake()) {
    transport_->close();
    return false;
  }
  connected_ = true;
  return true;
}

inline bool Stm32Backend::handshake()
{
  const uint16_t seq = transport_->next_seq();
  const int res = transport_->send_ack_request(
    Stm32Protocol::CMD_HELLO, seq, std::vector<uint8_t>{},
    timeouts_.target_ack_ms);
  if (res != Stm32Protocol::ACK_OK) {
    ++stats_.transport_errors;
    return false;
  }
  return true;
}

inline void Stm32Backend::set_io_mode(IoMode mode)
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  io_mode_ = mode;
  if (mode == IoMode::READ_ONLY) {
    targets_allowed_ = false;  // read-only never transmits targets
  }
}

inline Stm32Backend::IoMode Stm32Backend::io_mode() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  return io_mode_;
}

inline void Stm32Backend::set_targets_allowed(bool allowed)
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  targets_allowed_ = allowed && io_mode_ == IoMode::ACTIVE_CONTROL;
}

inline bool Stm32Backend::targets_allowed() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  return targets_allowed_;
}

inline bool Stm32Backend::target_escalated() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  return escalated_;
}

inline uint8_t Stm32Backend::control_state() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  return control_state_;
}

inline uint16_t Stm32Backend::enabled() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  return enabled_;
}

inline uint8_t Stm32Backend::fault() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  return fault_;
}

inline void Stm32Backend::close_gate_locked(const std::string & reason)
{
  if (targets_allowed_) {
    targets_allowed_ = false;
    gate_close_reason_ = reason;
    escalated_ = true;
  }
}

inline bool Stm32Backend::motion_ready_locked() const
{
  if (!targets_allowed_) return false;
  if (io_mode_ != IoMode::ACTIVE_CONTROL) return false;
  if (escalated_) return false;
  if (!got_state_) return false;
  const auto now = std::chrono::steady_clock::now();
  if ((now - last_all_valid_time_) >
      std::chrono::milliseconds(timeouts_.state_stale_ms)) {
    return false;
  }
  if (control_state_ != Stm32Protocol::CTRL_ENABLED) return false;
  if (enabled_ != 0x0FFFU) return false;
  if (fault_ != 0U) return false;
  return true;
}

inline bool Stm32Backend::link_healthy() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  if (!got_state_) return false;
  const auto now = std::chrono::steady_clock::now();
  return (now - last_all_valid_time_) <=
    std::chrono::milliseconds(timeouts_.state_stale_ms);
}

inline Stm32Backend::Health Stm32Backend::health() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  if (escalated_) {
    // Distinguish the escalation source via the close reason.
    return (gate_close_reason_.find("transport") != std::string::npos ||
            gate_close_reason_.find("timeout") != std::string::npos)
      ? Health::TRANSPORT_ERROR : Health::TARGET_REJECTED;
  }
  if (!got_state_) return Health::STATE_STALE;
  const auto now = std::chrono::steady_clock::now();
  if ((now - last_all_valid_time_) >
      std::chrono::milliseconds(timeouts_.state_stale_ms)) {
    return Health::STATE_STALE;
  }
  if (io_mode_ == IoMode::READ_ONLY) return Health::READ_ONLY;
  if (control_state_ != Stm32Protocol::CTRL_ENABLED) {
    return Health::CONTROL_NOT_ENABLED;
  }
  if (enabled_ != 0x0FFFU) return Health::ENABLE_BITMAP_INCOMPLETE;
  if (fault_ != 0U) return Health::FIRMWARE_FAULT;
  return Health::OK;
}

inline bool Stm32Backend::read_state(RobotState & out)
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  out.position = latest_state_;
  out.valid = state_valid_;
  out.control_state = control_state_;
  out.enabled = enabled_;
  out.fault = fault_;
  out.got_state = got_state_;
  out.last_update = last_state_update_;
  const auto now = std::chrono::steady_clock::now();
  out.link_healthy = got_state_ &&
    (now - last_all_valid_time_) <=
      std::chrono::milliseconds(timeouts_.state_stale_ms);
  return got_state_;
}

inline bool Stm32Backend::current_feedback(std::array<double, 12> & out)
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  out = latest_state_;
  return got_state_;
}

inline bool Stm32Backend::write_targets(const std::array<double, 12> & targets)
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  // Hard gates: mode, target gate, link freshness and motion preconditions.
  if (!motion_ready_locked()) return false;

  std::array<bool, 2> changed{false, false};
  for (int axis = 0; axis < 12; ++axis) {
    const AxisSlot & slot = axis_map_[axis];
    const double wire_si = slot.zero_offset + slot.direction * targets[axis];
    const double wire = wire_si * 1e6;
    const int arm = axis / 6;
    // Leave final validity/range reporting to send_arm_target(), but make
    // malformed values dirty so they cannot be silently suppressed here.
    if (!std::isfinite(wire) || wire < -2147483648.0 || wire > 2147483647.0) {
      changed[arm] = true;
      continue;
    }
    const int32_t rounded = static_cast<int32_t>(std::llround(wire));
    if (!arm_command_initialized_[arm] || latest_wire_command_[axis] != rounded) {
      changed[arm] = true;
    }
    latest_wire_command_[axis] = rounded;
  }
  latest_command_ = targets;
  for (int arm = 0; arm < 2; ++arm) {
    arm_dirty_[arm] = arm_dirty_[arm] || changed[arm];
    arm_command_initialized_[arm] = true;
  }
  return true;
}

inline bool Stm32Backend::run_control(uint8_t cmd)
{
  if (!connected_) return false;
  const uint16_t seq = transport_->next_seq();
  // Control sequences (ENABLE/STOP/DISABLE) are delayed-ACK commands: the
  // firmware runs the whole 12-axis sequence before replying, so the
  // timeout must be much larger than the TARGET / STATE timeouts.
  int timeout = timeouts_.control_ack_ms;
  std::vector<uint8_t> payload;
  if (cmd == Stm32Protocol::CMD_STOP) {
    payload = std::vector<uint8_t>{0};  // arm byte is ignored by V1.3.1
  }
  const int res = transport_->send_ack_request(cmd, seq, payload, timeout);
  if (res != Stm32Protocol::ACK_OK) {
    std::lock_guard<std::mutex> lock(cache_mtx_);
    ++stats_.transport_errors;
    return false;
  }
  return true;
}

inline bool Stm32Backend::enable() { return run_control(Stm32Protocol::CMD_ENABLE); }
inline bool Stm32Backend::stop()   { return run_control(Stm32Protocol::CMD_STOP); }
inline bool Stm32Backend::disable(){ return run_control(Stm32Protocol::CMD_DISABLE); }

inline void Stm32Backend::reset()
{
  stop_io();
  std::lock_guard<std::mutex> lock(cache_mtx_);
  latest_command_ = {};
  arm_dirty_ = {false, false};
  latest_wire_command_ = {};
  arm_command_initialized_ = {false, false};
  latest_state_ = {};
  state_valid_ = {};
  control_state_ = Stm32Protocol::CTRL_DISABLED;
  enabled_ = 0;
  fault_ = 0;
  got_state_ = false;
  targets_allowed_ = false;
  escalated_ = false;
  gate_close_reason_.clear();
  target_err_count_ = {0, 0};
  stats_ = Stats{};
  rate_base_ = RateBase{};
  io_mode_ = IoMode::READ_ONLY;
  connected_ = false;
  last_all_valid_time_ = {};
}

inline void Stm32Backend::close() noexcept
{
  stop_io();
  std::lock_guard<std::mutex> lock(cache_mtx_);
  if (transport_) {
    transport_->close();
  }
  connected_ = false;
}

inline bool Stm32Backend::wait_all_valid(int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  // Rate-limit the startup polling to state_poll_hz so a fast mock/device
  // is never hammered (review task §9.3).
  const double period_s = (timeouts_.state_poll_hz > 0.0)
    ? (1.0 / timeouts_.state_poll_hz) : 0.05;
  auto next = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < deadline) {
    Stm32Protocol::State st;
    const uint16_t seq = transport_->next_seq();
    const int res = transport_->send_get_state(seq, timeouts_.state_ms, st);
    if (res == 0) {
      std::lock_guard<std::mutex> lock(cache_mtx_);
      apply_state_locked(st);
      if (st.valid == 0x0FFFU) return true;
    }
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period_s));
    const auto now = std::chrono::steady_clock::now();
    if (now < next) {
      std::this_thread::sleep_until(next);
    } else {
      next = now;  // overran the poll period — don't sleep again
    }
  }
  return false;
}

inline bool Stm32Backend::refresh_state(int timeout_ms)
{
  if (!connected_) return false;
  Stm32Protocol::State st;
  const uint16_t seq = transport_->next_seq();
  const int res = transport_->send_get_state(seq, timeout_ms, st);
  if (res != 0) return false;
  std::lock_guard<std::mutex> lock(cache_mtx_);
  apply_state_locked(st);
  return true;
}

inline bool Stm32Backend::is_connected() const { return connected_; }
inline bool Stm32Backend::is_running() const { return running_.load(); }

inline void Stm32Backend::start_io(IoMode mode)
{
  {
    std::lock_guard<std::mutex> lock(cache_mtx_);
    io_mode_ = mode;
    if (mode == IoMode::READ_ONLY) targets_allowed_ = false;
  }
  if (running_.exchange(true)) return;

  io_thread_ = std::thread([this]() {
    // Deadline scheduling: the loop rate is state_poll_hz, but a round that
    // takes longer than the period is NOT followed by an extra sleep — the
    // communication itself caps the rate, not an added fixed delay
    // (review task §9).
    const double period_s = (timeouts_.state_poll_hz > 0.0)
      ? (1.0 / timeouts_.state_poll_hz) : 0.05;
    auto next = std::chrono::steady_clock::now();
    while (running_.load()) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period_s));
      const auto start = std::chrono::steady_clock::now();
      service_step();
      const auto done = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lk(cache_mtx_);
        const double loop_us = std::chrono::duration_cast<
          std::chrono::microseconds>(done - start).count();
        stats_.last_loop_period_us = loop_us;
        stats_.max_loop_period_us = std::max(stats_.max_loop_period_us, loop_us);
        ++stats_.total_loops;
      }
      if (done < next) {
        std::this_thread::sleep_until(next);
      } else {
        std::lock_guard<std::mutex> lk(cache_mtx_);
        ++stats_.io_overruns;
        next = done;  // no extra sleep on overrun
      }
    }
  });
}

inline void Stm32Backend::stop_io()
{
  if (!running_.exchange(false)) return;
  if (io_thread_.joinable()) io_thread_.join();
}

inline void Stm32Backend::record_target_result_locked(uint8_t arm, int res)
{
  if (res == Stm32Protocol::ACK_OK) {
    ++stats_.target_ack_ok[arm];
    target_err_count_[arm] = 0;
  } else if (res == Stm32Protocol::ACK_SUPERSEDED) {
    ++stats_.target_superseded[arm];
    target_err_count_[arm] = 0;  // intentional drop — not a fault
  } else if (res == Stm32Protocol::ACK_CTRL_BUSY) {
    ++stats_.target_ctrl_busy[arm];
    target_err_count_[arm] = 0;  // retried naturally on the next round
    // (latest-wins: the freshest target is simply sent again)
  } else if (res < 0) {
    ++stats_.target_timeout[arm];
    ++stats_.transport_errors;
    if (++target_err_count_[arm] >= 3) {
      close_gate_locked("target timeout (arm " + std::to_string(arm) + ")");
    }
  } else {
    // STATE_DENIED / OUT_OF_RANGE / CTRL_FAILED / CRC / FORMAT / BAD_LEN /
    // BAD_ARM / COMM_TIMEOUT …
    ++stats_.target_result[res & 0x0F];
    ++target_err_count_[arm];
    if (res == Stm32Protocol::ACK_STATE_DENIED) {
      close_gate_locked("STATE_DENIED (arm " + std::to_string(arm) + ")");
    } else if (res == Stm32Protocol::ACK_OUT_OF_RANGE) {
      // command error — do NOT resend the same target; report only
      if (++target_err_count_[arm] >= 3) {
        close_gate_locked("OUT_OF_RANGE (arm " + std::to_string(arm) + ")");
      }
    } else if (res == Stm32Protocol::ACK_CTRL_FAILED) {
      close_gate_locked("CTRL_FAILED (arm " + std::to_string(arm) + ")");
    } else {
      // protocol / program errors (CRC, FORMAT, BAD_LEN, BAD_ARM, …)
      close_gate_locked("protocol error 0x" +
                        std::to_string(res) + " (arm " +
                        std::to_string(arm) + ")");
    }
  }
}

inline bool Stm32Backend::send_arm_target(uint8_t arm)
{
  std::array<double, 6> si;
  {
    std::lock_guard<std::mutex> lock(cache_mtx_);
    if (io_mode_ == IoMode::READ_ONLY) {
      // Read-only mode: drop any pending targets — never transmit. This is
      // the hard gate that keeps F.2 read-only safe even if write_targets()
      // is called by mistake.
      arm_dirty_[0] = false;
      arm_dirty_[1] = false;
      return true;
    }
    if (!motion_ready_locked()) {
      // Preconditions not met (gate closed / stale / not enabled / …).
      // Keep the dirty flag so a later valid state can still transmit —
      // EXCEPT when escalated, where the gate must stay closed.
      if (escalated_) {
        arm_dirty_[0] = false;
        arm_dirty_[1] = false;
      }
      return true;
    }
    if (!arm_dirty_[arm]) return true;  // nothing new for this arm
    for (int i = 0; i < 6; ++i) {
      si[i] = latest_command_[arm * 6 + i];
    }
    arm_dirty_[arm] = false;
  }

  // ── Atomic per-arm validation (review task §11.1) ─────────────
  // Any invalid axis rejects the WHOLE arm frame — never clamp-and-send.
  Stm32Protocol::Target t;
  t.arm = arm;
  for (int i = 0; i < 6; ++i) {
    const AxisSlot & slot = axis_map_[arm * 6 + i];
    const double v = slot.zero_offset + slot.direction * si[i];
    if (!std::isfinite(v)) {
      std::lock_guard<std::mutex> lock(cache_mtx_);
      ++stats_.target_rejected[arm];
      if (++target_err_count_[arm] >= 3) {
        close_gate_locked("non-finite target (arm " + std::to_string(arm) + ")");
      }
      return false;  // do NOT re-mark dirty: same invalid target won't resend
    }
    const double w = v * 1e6;          // m/rad → µm/µrad
    if (w < -2147483648.0 || w > 2147483647.0) {
      std::lock_guard<std::mutex> lock(cache_mtx_);
      ++stats_.target_rejected[arm];
      if (++target_err_count_[arm] >= 3) {
        close_gate_locked("int32 out-of-range target (arm " +
                          std::to_string(arm) + ")");
      }
      return false;  // out of range — no clamp, no resend of the same value
    }
    t.joint[i] = static_cast<int32_t>(std::llround(w));
  }

  const uint16_t seq = transport_->next_seq();
  const int res = transport_->send_ack_request(
    Stm32Protocol::CMD_TARGET, seq,
    Stm32Protocol::encode_target(t), timeouts_.target_ack_ms);

  std::lock_guard<std::mutex> lock(cache_mtx_);
  record_target_result_locked(arm, res);
  return res == Stm32Protocol::ACK_OK || res == Stm32Protocol::ACK_SUPERSEDED;
}

inline bool Stm32Backend::send_get_state_request()
{
  Stm32Protocol::State st;
  const uint16_t seq = transport_->next_seq();
  const int res = transport_->send_get_state(seq, timeouts_.state_ms, st);
  if (res != 0) {
    std::lock_guard<std::mutex> lock(cache_mtx_);
    ++stats_.state_timeout;
    return false;
  }
  std::lock_guard<std::mutex> lock(cache_mtx_);
  apply_state_locked(st);
  ++stats_.state_received;
  return true;
}

inline void Stm32Backend::service_step()
{
  if (!connected_) return;
  if (io_mode() == IoMode::READ_ONLY) {
    // Read-only mode: GET_STATE only — never any control or target frame.
    send_get_state_request();
    return;
  }
  // Active-control mode: latest-wins per arm; if both arms were updated
  // while the previous round was in flight, only the newest snapshot per
  // arm is transmitted (old ones dropped).
  send_arm_target(0);
  send_arm_target(1);
  send_get_state_request();
}

inline void Stm32Backend::apply_state_locked(const Stm32Protocol::State & st)
{
  control_state_ = st.control_state;
  fault_ = st.fault;
  enabled_ = st.enabled;
  got_state_ = true;
  const auto now = std::chrono::steady_clock::now();
  last_state_update_ = now;
  // A fully-valid STATE keeps the link healthy; once the last one ages
  // beyond state_stale_ms, new targets are refused.
  if (st.valid == 0x0FFFU) {
    last_all_valid_time_ = now;
  }

  for (int axis = 0; axis < 12; ++axis) {
    if (Stm32Protocol::axis_valid(st, axis)) {
      // Wire (µm/µrad) → SI (m/rad) with the configured coordinate
      // direction and zero offset.
      const AxisSlot & slot = axis_map_[axis];
      latest_state_[axis] = slot.direction *
        (static_cast<double>(st.position[axis]) * 1e-6 - slot.zero_offset);
      state_valid_[axis] = true;
    } else {
      // No fresh read-back this frame: keep the last known value, mark
      // the axis not-fresh so the caller does not publish a stale value.
      state_valid_[axis] = false;
    }
  }
}

inline Stm32Backend::Stats Stm32Backend::stats() const
{
  std::lock_guard<std::mutex> lock(cache_mtx_);
  Stats s = stats_;
  // Fold in transport mismatch / error counters.
  s.ack_seq_mismatch = static_cast<uint64_t>(transport_->ack_seq_mismatch_count());
  s.ack_cmd_mismatch = static_cast<uint64_t>(transport_->ack_cmd_mismatch_count());
  s.state_seq_mismatch = static_cast<uint64_t>(transport_->state_seq_mismatch_count());
  s.crc_errors = static_cast<uint64_t>(transport_->error_count());

  // Measured rates over the last full window (≥ 1 s of traffic).
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_s = std::chrono::duration<double>(
    now - rate_base_.t).count();
  if (rate_base_.t.time_since_epoch().count() == 0) {
    rate_base_.t = now;
    rate_base_.target[0] = s.target_ack_ok[0] + s.target_timeout[0] +
                           s.target_rejected[0] + s.target_ctrl_busy[0];
    rate_base_.target[1] = s.target_ack_ok[1] + s.target_timeout[1] +
                           s.target_rejected[1] + s.target_ctrl_busy[1];
    rate_base_.state = s.state_received;
  } else if (elapsed_s >= 1.0) {
    const uint64_t t0 = s.target_ack_ok[0] + s.target_timeout[0] +
                        s.target_rejected[0] + s.target_ctrl_busy[0];
    const uint64_t t1 = s.target_ack_ok[1] + s.target_timeout[1] +
                        s.target_rejected[1] + s.target_ctrl_busy[1];
    s.target_hz[0] = static_cast<double>(t0 - rate_base_.target[0]) / elapsed_s;
    s.target_hz[1] = static_cast<double>(t1 - rate_base_.target[1]) / elapsed_s;
    s.state_hz = static_cast<double>(s.state_received - rate_base_.state) / elapsed_s;
    rate_base_.t = now;
    rate_base_.target[0] = t0;
    rate_base_.target[1] = t1;
    rate_base_.state = s.state_received;
  }
  return s;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__STM32_BACKEND_HPP_
