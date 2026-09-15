#ifndef DOUBLE_ARM_HARDWARE__E4_SHARED_STOP_HPP_
#define DOUBLE_ARM_HARDWARE__E4_SHARED_STOP_HPP_

#include <atomic>

namespace double_arm_hardware
{

/// Process-wide E.4 shared stop flag.
///
/// Both ArmSystemHardware instances (left / right) live in the same
/// ros2_control process, so a single atomic flag lets either arm halt
/// BOTH arms' step-wise homing: once set, no arm generates new targets
/// and every arm holds its current feedback (no motion, no new commands).
class E4SharedStop
{
public:
  static bool active()
  {
    return flag_.load(std::memory_order_relaxed);
  }

  static void trigger()
  {
    flag_.store(true, std::memory_order_relaxed);
  }

  /// Test-only: reset between offline scenarios.
  static void clear()
  {
    flag_.store(false, std::memory_order_relaxed);
  }

private:
  static std::atomic<bool> flag_;
};

/// Process-wide E.4 "arm finished" flag.
///
/// Used to sequence the two arms (right → left): an arm with
/// e4_arm_order == 1 marks this flag once it has homed its whole joint
/// sequence; an arm with e4_arm_order == 2 waits for the flag before it
/// leaves its static hold and starts homing.
class E4SharedDone
{
public:
  static bool done()
  {
    return flag_.load(std::memory_order_relaxed);
  }

  static void mark()
  {
    flag_.store(true, std::memory_order_relaxed);
  }

  /// Test-only: reset between offline scenarios.
  static void clear()
  {
    flag_.store(false, std::memory_order_relaxed);
  }

private:
  static std::atomic<bool> flag_;
};

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__E4_SHARED_STOP_HPP_
