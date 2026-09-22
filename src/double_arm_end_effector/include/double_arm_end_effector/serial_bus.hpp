// Copyright 2026 zmy
#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace double_arm_end_effector
{

class SerialBus
{
public:
  SerialBus() = default;
  ~SerialBus();
  SerialBus(const SerialBus &) = delete;
  SerialBus & operator=(const SerialBus &) = delete;

  void open_port(const std::string & device, int baud_rate);
  void close_port();
  bool is_open() const;

  void write_command(const std::string & command);
  std::optional<int> read_position(int servo_id, std::chrono::milliseconds timeout);

private:
  int fd_{-1};
};

}  // namespace double_arm_end_effector
