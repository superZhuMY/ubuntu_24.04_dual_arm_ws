// Copyright 2026 zmy
#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace double_arm_end_effector
{

constexpr int kMinServoId = 0;
constexpr int kMaxServoId = 254;
constexpr int kMinPulse = 500;
constexpr int kMaxPulse = 2500;
constexpr int kMaxDurationMs = 9999;

bool valid_servo_id(int id);
bool valid_pulse(int pulse);
int normalized_to_pulse(double position, int open_pulse, int closed_pulse);
double pulse_to_normalized(int pulse, int open_pulse, int closed_pulse);

std::string move_command(int id, int pulse, int duration_ms);
std::string read_position_command(int id);
std::string stop_command(int id);

/// Finds a #dddPdddd! response in an arbitrary receive buffer. Query echoes
/// such as #dddPRAD! are ignored.
std::optional<int> parse_position_response(std::string_view data, int expected_id);

}  // namespace double_arm_end_effector
