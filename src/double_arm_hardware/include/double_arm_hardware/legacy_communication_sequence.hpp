#ifndef DOUBLE_ARM_HARDWARE__LEGACY_COMMUNICATION_SEQUENCE_HPP_
#define DOUBLE_ARM_HARDWARE__LEGACY_COMMUNICATION_SEQUENCE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "double_arm_hardware/transport_interface.hpp"

namespace double_arm_hardware
{

/// Replicates the exact communication sequence from the original
/// ROS1 jaka_send_read_node_zong_{L,R}.cpp firmware.
///
/// Joint write order:  J5 → J6 → J4 → J3 → J2 → J1
/// Joint read order:   J1 → J2 → J3 → J4 → J5 → J6
///
/// Serial J4-J6:  flush → A3 write → sleep ~1ms → 92 read → sleep ~1ms → read 14B
/// Modbus J1-J3:  0x0305=0 → sleep ~1ms → 0x110C write (2 regs, low first) →
///                sleep ~1ms → 0x0305=1 → read 0x0B07 (2 regs)
///
/// All hardware access goes through ITransport — never calls open/close/connect directly.
class LegacyCommunicationSequence
{
public:
  /// @param transport          Already-configured ITransport
  /// @param arm_label          "L" or "R" (for logging)
  /// @param serial_timeout_ms  L=20, R=30 (from original firmware)
  /// @param serial_motor_ids   e.g. {4,5,6} — used instead of hardcoded IDs
  /// @param modbus_slaves      e.g. {1,2,3} — used instead of hardcoded IDs
  LegacyCommunicationSequence(
    std::shared_ptr<ITransport> transport,
    const std::string & arm_label,
    int serial_timeout_ms,
    const std::vector<int> & serial_motor_ids = {4, 5, 6},
    const std::vector<int> & modbus_slaves = {1, 2, 3});

  // ── Write path (called by ros2_control write()) ────────────────

  /// Write all 6 joints in original order: J5→J6→J4→J3→J2→J1.
  /// Each write also reads back and updates the raw feedback buffer.
  /// @param motor_commands  [J1..J6] raw motor values
  /// @return true if ALL writes succeeded
  bool write_all_joints(const std::array<double, 6> & motor_commands);

  // ── Read path (called by ros2_control read()) ──────────────────

  /// Read all 6 joints in original order: J1→J2→J3→J4→J5→J6.
  /// Updates the internal raw feedback buffer.
  /// @return true if ALL reads succeeded
  bool read_all_joints();

  /// Get the last-read raw feedback values.
  std::array<double, 6> raw_feedback() const { return raw_feedback_; }

  // ── Lifecycle ──────────────────────────────────────────────────

  /// Open transport connections ONLY — no 0x0303 writes.
  /// Used by the E.3 safe-hold flow so that real positions can be
  /// read and validated before any power-on write.
  bool open();

  /// Original power-on step: read 0x0303 per Modbus slave and write
  /// 0x0001 only when the current status is 0.  Assumes transport open.
  /// Replicates the original initialize_modbus() power-on part.
  bool power_on();

  /// Initialize transport (open connections, power on motors).
  /// Replicates the original initialize_modbus() + serial setup.
  /// @return true on success
  bool initialize();

  /// Close connections WITHOUT power-off writes.  Used when the
  /// E.3 safe-hold flow never powered the arm on (preview / failure).
  void close_only();

  /// Stop transport (power off motors, close connections).
  /// Replicates the original close_modbus_connections().
  void shutdown();

  // ── Parameters ─────────────────────────────────────────────────

  const std::string & arm_label() const { return arm_label_; }
  int serial_timeout_ms() const { return serial_timeout_ms_; }

  /// Expose the underlying transport (for RecordingTransport access in tests).
  std::shared_ptr<ITransport> get_transport() { return transport_; }

private:
  // ── Serial helpers (J4-J6) ────────────────────────────────────

  /// Single serial joint: flush → A3 write → sleep 1ms → 92 read → sleep 1ms → read 14B
  bool write_serial_and_read(int motor_id, double command, double & raw_out);

  /// Read-only: flush → 92 read → sleep 1ms → read 14B
  bool read_serial(int motor_id, double & raw_out);

  // ── Modbus helpers (J1-J3) ────────────────────────────────────

  /// Single modbus write+read cycle
  bool write_modbus_and_read(int slave_index, double command, double & raw_out);

  /// Read-only: read 0x0B07
  bool read_modbus(int slave_index, double & raw_out);

  std::shared_ptr<ITransport> transport_;
  std::string arm_label_;       // "L" or "R"
  int serial_timeout_ms_;       // 20 for L, 30 for R
  std::vector<int> serial_ids_;   // e.g. {4,5,6}
  std::vector<int> modbus_ids_;   // e.g. {1,2,3}

  std::array<double, 6> raw_feedback_{};
};

// ── inline implementations ──────────────────────────────────────────

inline LegacyCommunicationSequence::LegacyCommunicationSequence(
  std::shared_ptr<ITransport> transport,
  const std::string & arm_label,
  int serial_timeout_ms,
  const std::vector<int> & serial_motor_ids,
  const std::vector<int> & modbus_slaves)
  : transport_(std::move(transport))
  , arm_label_(arm_label)
  , serial_timeout_ms_(serial_timeout_ms)
  , serial_ids_(serial_motor_ids)
  , modbus_ids_(modbus_slaves)
{
  raw_feedback_.fill(0.0);
}

// ═══════════════════════════════════════════════════════════════════
//  Write all 6 joints: J5 → J6 → J4 → J3 → J2 → J1
// ═══════════════════════════════════════════════════════════════════

inline bool LegacyCommunicationSequence::write_all_joints(
  const std::array<double, 6> & motor_commands)
{
  bool ok = true;

  // ── Serial J5 → J6 → J4 (using configured motor IDs) ─────────
  ok = write_serial_and_read(serial_ids_[1], motor_commands[4], raw_feedback_[4]) && ok;  // J5
  ok = write_serial_and_read(serial_ids_[2], motor_commands[5], raw_feedback_[5]) && ok;  // J6
  ok = write_serial_and_read(serial_ids_[0], motor_commands[3], raw_feedback_[3]) && ok;  // J4

  // ── Modbus J3 → J2 → J1 (reversed: slave 2,1,0) ──────────────
  ok = write_modbus_and_read(2, motor_commands[2], raw_feedback_[2]) && ok;
  ok = write_modbus_and_read(1, motor_commands[1], raw_feedback_[1]) && ok;
  ok = write_modbus_and_read(0, motor_commands[0], raw_feedback_[0]) && ok;

  return ok;
}

// ═══════════════════════════════════════════════════════════════════
//  Read all 6 joints: J1 → J2 → J3 → J4 → J5 → J6
// ═══════════════════════════════════════════════════════════════════

inline bool LegacyCommunicationSequence::read_all_joints()
{
  bool ok = true;

  // ── Modbus J1 → J2 → J3 ──────────────────────────────────────
  ok = read_modbus(0, raw_feedback_[0]) && ok;
  ok = read_modbus(1, raw_feedback_[1]) && ok;
  ok = read_modbus(2, raw_feedback_[2]) && ok;

  // ── Serial J4 → J5 → J6 ──────────────────────────────────────
  ok = read_serial(serial_ids_[0], raw_feedback_[3]) && ok;
  ok = read_serial(serial_ids_[1], raw_feedback_[4]) && ok;
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ok = read_serial(serial_ids_[2], raw_feedback_[5]) && ok;

  return ok;
}

// ═══════════════════════════════════════════════════════════════════
//  Serial write+read: flush → A3 write → sleep 1ms → 92 read → sleep 1ms → read 14B
// ═══════════════════════════════════════════════════════════════════

inline bool LegacyCommunicationSequence::write_serial_and_read(
  int motor_id, double command, double & raw_out)
{
  transport_->flush_serial();

  if (!transport_->write_serial(motor_id, command))
    return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (!transport_->read_serial_position(motor_id, raw_out))
    return false;

  return true;
}

inline bool LegacyCommunicationSequence::read_serial(
  int motor_id, double & raw_out)
{
  transport_->flush_serial();

  if (!transport_->read_serial_position(motor_id, raw_out))
    return false;

  return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Modbus write+read: 0x0305=0 → 0x110C write → 0x0305=1 → 0x0B07 read
// ═══════════════════════════════════════════════════════════════════

inline bool LegacyCommunicationSequence::write_modbus_and_read(
  int slave_index, double command, double & raw_out)
{
  // 0x0305 = POWER_OFF
  if (!transport_->write_modbus_register(slave_index, 0x0305, 0x0000))
    return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // 0x110C write position (2 registers, low word first)
  if (!transport_->write_modbus_position(slave_index, 0x110C,
        static_cast<uint32_t>(command)))
    return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // 0x0305 = POWER_ON
  if (!transport_->write_modbus_register(slave_index, 0x0305, 0x0001))
    return false;

  // 0x0B07 read position feedback (2 registers)
  if (!transport_->read_modbus_position(slave_index, 0x0B07, raw_out))
    return false;

  return true;
}

inline bool LegacyCommunicationSequence::read_modbus(
  int slave_index, double & raw_out)
{
  return transport_->read_modbus_position(slave_index, 0x0B07, raw_out);
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════

inline bool LegacyCommunicationSequence::initialize()
{
  if (!open())
    return false;
  if (!power_on()) {
    transport_->close();
    return false;
  }
  return true;
}

inline bool LegacyCommunicationSequence::open()
{
  // Step 1: open transport (serial + modbus connections)
  return transport_->open();
}

inline bool LegacyCommunicationSequence::power_on()
{
  // Step 2: Original init — for each modbus slave, check 0x0303
  //          and power on if zero. Each step is checked.
  for (size_t i = 0; i < modbus_ids_.size(); ++i) {
    double status = 0.0;
    // read 0x0303 — must succeed, no default value fallback
    if (!transport_->read_modbus_register(static_cast<int>(i), 0x0303, status)) {
      return false;
    }
    // Original: if (buffer[0] == 0) → write on_power
    if (status == 0.0) {
      if (!transport_->write_modbus_register(static_cast<int>(i), 0x0303, 0x0001)) {
        return false;
      }
    }
  }

  return true;
}

inline void LegacyCommunicationSequence::close_only()
{
  transport_->close();
}

inline void LegacyCommunicationSequence::shutdown()
{
  // Original shutdown: write 0x0303=off twice with 110ms sleep per slave
  for (size_t i = 0; i < modbus_ids_.size(); ++i) {
    transport_->write_modbus_register(static_cast<int>(i), 0x0303, 0x0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    transport_->write_modbus_register(static_cast<int>(i), 0x0303, 0x0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
  }
  transport_->close();
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__LEGACY_COMMUNICATION_SEQUENCE_HPP_
