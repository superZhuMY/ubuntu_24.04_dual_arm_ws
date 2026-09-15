#ifndef DOUBLE_ARM_HARDWARE__RECORDING_TRANSPORT_HPP_
#define DOUBLE_ARM_HARDWARE__RECORDING_TRANSPORT_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "double_arm_hardware/transport_interface.hpp"

namespace double_arm_hardware
{

/// Wraps any ITransport and records every call with its arguments.
/// Used in tests to assert exact communication sequences.
class RecordingTransport : public ITransport
{
public:
  enum class Op : uint8_t
  {
    CONFIGURE, OPEN, CLOSE,
    FLUSH_SERIAL,
    WRITE_SERIAL,        READ_SERIAL,
    READ_SERIAL_POSITION,
    WRITE_MODBUS,        READ_MODBUS,
    WRITE_MODBUS_REG,    READ_MODBUS_REG,
    WRITE_MODBUS_POS,    READ_MODBUS_POS,
  };

  struct Record
  {
    Op op;
    int arg0 = 0;            // motor_id / slave_index
    int arg1 = 0;            // register address (for reg ops)
    double dval = 0.0;       // raw value
    uint32_t uval = 0;       // uint32 value
    static std::string op_name(Op o);
  };

  explicit RecordingTransport(std::shared_ptr<ITransport> inner)
    : inner_(std::move(inner)) {}

  ~RecordingTransport() override = default;

  // ── ITransport passthrough + record ───────────────────────────
  bool configure(const std::string & md, const std::string & sd,
    const std::vector<int> & ms, const std::vector<int> & sm,
    int cycle, int timeout) override;

  bool open() override;
  void close() override;

  bool write_modbus(int slave_index, double raw) override;
  bool read_modbus(int slave_index, double & raw) override;
  bool write_serial(int motor_id, double raw) override;
  bool read_serial(int motor_id, double & raw) override;

  void flush_serial() override;
  bool read_serial_position(int motor_id, double & raw) override;
  bool write_modbus_register(int slave_index, uint16_t addr, uint16_t val) override;
  bool read_modbus_register(int slave_index, uint16_t addr, double & raw) override;
  bool write_modbus_position(int slave_index, uint16_t addr, uint32_t val) override;
  bool read_modbus_position(int slave_index, uint16_t addr, double & raw) override;

  std::string mode_name() const override { return inner_->mode_name(); }
  bool is_real() const override { return inner_->is_real(); }
  int error_count() const override { return inner_->error_count(); }

  // ── Record access ─────────────────────────────────────────────
  const std::deque<Record> & records() const { return records_; }
  void clear() { records_.clear(); }

  /// Print all records as a human-readable sequence.
  std::string dump() const;

  /// Get inner transport (for inspecting state after operations).
  std::shared_ptr<ITransport> inner() { return inner_; }

private:
  void add(Op op, int a0 = 0, int a1 = 0, double d = 0.0, uint32_t u = 0)
  {
    records_.push_back({op, a0, a1, d, u});
  }

  std::shared_ptr<ITransport> inner_;
  std::deque<Record> records_;
};

// ── inline impls ────────────────────────────────────────────────────

inline std::string RecordingTransport::Record::op_name(Op o)
{
  switch (o) {
    case Op::CONFIGURE:            return "CONFIGURE";
    case Op::OPEN:                 return "OPEN";
    case Op::CLOSE:                return "CLOSE";
    case Op::FLUSH_SERIAL:         return "FLUSH_SERIAL";
    case Op::WRITE_SERIAL:         return "WRITE_SERIAL";
    case Op::READ_SERIAL:          return "READ_SERIAL";
    case Op::READ_SERIAL_POSITION: return "READ_SERIAL_POSITION";
    case Op::WRITE_MODBUS:         return "WRITE_MODBUS";
    case Op::READ_MODBUS:          return "READ_MODBUS";
    case Op::WRITE_MODBUS_REG:     return "WRITE_MODBUS_REG";
    case Op::READ_MODBUS_REG:      return "READ_MODBUS_REG";
    case Op::WRITE_MODBUS_POS:     return "WRITE_MODBUS_POS";
    case Op::READ_MODBUS_POS:      return "READ_MODBUS_POS";
  }
  return "?";
}

inline bool RecordingTransport::configure(
  const std::string & md, const std::string & sd,
  const std::vector<int> & ms, const std::vector<int> & sm,
  int cycle, int timeout)
{
  add(Op::CONFIGURE);
  return inner_->configure(md, sd, ms, sm, cycle, timeout);
}
inline bool RecordingTransport::open()
{
  add(Op::OPEN);
  return inner_->open();
}
inline void RecordingTransport::close()
{
  add(Op::CLOSE);
  inner_->close();
}

inline bool RecordingTransport::write_modbus(int si, double r)
{
  add(Op::WRITE_MODBUS, si, 0, r);
  return inner_->write_modbus(si, r);
}
inline bool RecordingTransport::read_modbus(int si, double & r)
{
  bool ok = inner_->read_modbus(si, r);
  add(Op::READ_MODBUS, si, 0, r);
  return ok;
}
inline bool RecordingTransport::write_serial(int mi, double r)
{
  add(Op::WRITE_SERIAL, mi, 0, r);
  return inner_->write_serial(mi, r);
}
inline bool RecordingTransport::read_serial(int mi, double & r)
{
  bool ok = inner_->read_serial(mi, r);
  add(Op::READ_SERIAL, mi, 0, r);
  return ok;
}

inline void RecordingTransport::flush_serial()
{
  add(Op::FLUSH_SERIAL);
  inner_->flush_serial();
}
inline bool RecordingTransport::read_serial_position(int mi, double & r)
{
  bool ok = inner_->read_serial_position(mi, r);
  add(Op::READ_SERIAL_POSITION, mi, 0, r);
  return ok;
}
inline bool RecordingTransport::write_modbus_register(int si, uint16_t addr, uint16_t val)
{
  add(Op::WRITE_MODBUS_REG, si, addr, 0.0, val);
  return inner_->write_modbus_register(si, addr, val);
}
inline bool RecordingTransport::read_modbus_register(int si, uint16_t addr, double & r)
{
  bool ok = inner_->read_modbus_register(si, addr, r);
  add(Op::READ_MODBUS_REG, si, addr, r);
  return ok;
}
inline bool RecordingTransport::write_modbus_position(int si, uint16_t addr, uint32_t val)
{
  add(Op::WRITE_MODBUS_POS, si, addr, 0.0, val);
  return inner_->write_modbus_position(si, addr, val);
}
inline bool RecordingTransport::read_modbus_position(int si, uint16_t addr, double & r)
{
  bool ok = inner_->read_modbus_position(si, addr, r);
  add(Op::READ_MODBUS_POS, si, addr, r);
  return ok;
}

inline std::string RecordingTransport::dump() const
{
  std::ostringstream oss;
  for (const auto & r : records_) {
    oss << Record::op_name(r.op) << "(" << r.arg0;
    if (r.arg1) oss << "," << r.arg1;
    oss << ")";
    if (r.op == Op::WRITE_SERIAL || r.op == Op::READ_SERIAL ||
        r.op == Op::READ_SERIAL_POSITION || r.op == Op::WRITE_MODBUS ||
        r.op == Op::READ_MODBUS || r.op == Op::READ_MODBUS_REG)
      oss << "=" << r.dval;
    if (r.op == Op::WRITE_MODBUS_REG || r.op == Op::WRITE_MODBUS_POS)
      oss << " val=" << r.uval;
    oss << "\n";
  }
  return oss.str();
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__RECORDING_TRANSPORT_HPP_
