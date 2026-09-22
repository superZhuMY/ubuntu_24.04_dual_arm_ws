// Copyright 2026 zmy

#include "double_arm_end_effector/serial_bus_servo_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace double_arm_end_effector
{

bool valid_servo_id(int id)
{
  return id >= kMinServoId && id <= kMaxServoId;
}

bool valid_pulse(int pulse)
{
  return pulse >= kMinPulse && pulse <= kMaxPulse;
}

int normalized_to_pulse(double position, int open_pulse, int closed_pulse)
{
  if (!std::isfinite(position) || position < 0.0 || position > 1.0) {
    throw std::invalid_argument("normalized servo position must lie in [0, 1]");
  }
  if (!valid_pulse(open_pulse) || !valid_pulse(closed_pulse) ||
    open_pulse == closed_pulse)
  {
    throw std::invalid_argument("open/closed pulses must be distinct values in [500, 2500]");
  }
  return static_cast<int>(std::lround(
    static_cast<double>(open_pulse) + position * (closed_pulse - open_pulse)));
}

double pulse_to_normalized(int pulse, int open_pulse, int closed_pulse)
{
  if (open_pulse == closed_pulse) {
    throw std::invalid_argument("open and closed pulses must be distinct");
  }
  const double value = static_cast<double>(pulse - open_pulse) /
    static_cast<double>(closed_pulse - open_pulse);
  return std::clamp(value, 0.0, 1.0);
}

namespace
{
std::string format_command(const char * format, int id, int first = 0, int second = 0)
{
  char buffer[32];
  const int written = std::snprintf(buffer, sizeof(buffer), format, id, first, second);
  if (written <= 0 || static_cast<std::size_t>(written) >= sizeof(buffer)) {
    throw std::runtime_error("failed to format serial-bus servo command");
  }
  return std::string(buffer, static_cast<std::size_t>(written));
}
}  // namespace

std::string move_command(int id, int pulse, int duration_ms)
{
  if (!valid_servo_id(id) || !valid_pulse(pulse) ||
    duration_ms < 0 || duration_ms > kMaxDurationMs)
  {
    throw std::invalid_argument("invalid servo id, pulse, or duration");
  }
  return format_command("#%03dP%04dT%04d!", id, pulse, duration_ms);
}

std::string read_position_command(int id)
{
  if (!valid_servo_id(id)) {
    throw std::invalid_argument("invalid servo id");
  }
  return format_command("#%03dPRAD!", id);
}

std::string stop_command(int id)
{
  if (!valid_servo_id(id)) {
    throw std::invalid_argument("invalid servo id");
  }
  return format_command("#%03dPDST!", id);
}

std::optional<int> parse_position_response(std::string_view data, int expected_id)
{
  if (!valid_servo_id(expected_id)) {
    return std::nullopt;
  }
  char prefix_buffer[8];
  std::snprintf(prefix_buffer, sizeof(prefix_buffer), "#%03dP", expected_id);
  const std::string_view prefix(prefix_buffer);
  std::size_t cursor = 0;
  while ((cursor = data.find(prefix, cursor)) != std::string_view::npos) {
    const std::size_t digit = cursor + prefix.size();
    if (digit + 5 <= data.size() && data[digit + 4] == '!') {
      int pulse = 0;
      bool digits = true;
      for (std::size_t i = 0; i < 4; ++i) {
        const char ch = data[digit + i];
        if (ch < '0' || ch > '9') {
          digits = false;
          break;
        }
        pulse = pulse * 10 + (ch - '0');
      }
      if (digits && valid_pulse(pulse)) {
        return pulse;
      }
    }
    cursor += prefix.size();
  }
  return std::nullopt;
}

}  // namespace double_arm_end_effector
