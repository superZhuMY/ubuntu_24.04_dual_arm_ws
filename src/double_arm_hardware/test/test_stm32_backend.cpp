/// Offline tests for Stm32Backend (stm32_backend.hpp) against a mock serial
/// backend that emulates the STM32 V1.3.1 firmware behaviour.
///
/// Covers review-document §10 items 10–16: startup sync gate, first-target
/// equals feedback, valid-bitmap retention, no unbounded queue, pending vs
/// active snapshot, per-arm latest targets, SUPERSEDED handling and
/// different timeouts per command class.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "double_arm_hardware/robot_backend.hpp"
#include "double_arm_hardware/stm32_backend.hpp"
#include "double_arm_hardware/stm32_mock_serial.hpp"
#include "double_arm_hardware/stm32_transport.hpp"

using namespace double_arm_hardware;
using namespace std::chrono_literals;

namespace
{

struct BF
{
  MockStm32Serial * mock = nullptr;
  std::shared_ptr<Stm32Transport> tr;
  std::shared_ptr<Stm32Backend> be;

  explicit BF(bool gate = true, Stm32Backend::Timeouts t = Stm32Backend::Timeouts{})
  {
    auto s = std::make_unique<MockStm32Serial>();
    mock = s.get();
    tr = std::make_shared<Stm32Transport>(std::move(s));
    tr->configure("/dev/ttyUSB0", 115200, t.target_ack_ms, t.state_ms,
                  t.control_ack_ms);
    be = std::make_shared<Stm32Backend>(tr, gate, t);
    IRobotBackend::HardwareConfig cfg;
    for (int i = 0; i < 12; ++i) {
      cfg.axes.push_back({Stm32Protocol::axis_name(i), 0.0});
    }
    if (!be->configure(cfg)) {
      throw std::runtime_error("BF: configure failed");
    }
  }

  void connect_ok()
  {
    ASSERT_TRUE(be->connect());
    ASSERT_TRUE(be->is_connected());
  }

  void full_valid(const std::array<int32_t, 12> & pos,
                  uint8_t ctrl = Stm32Protocol::CTRL_ENABLED)
  {
    mock->set_state_values(pos, 0x0FFFu, ctrl);
  }

  /// Open the active-control target gate (motion preconditions must also be
  /// met by a fresh full-valid STATE before targets actually flow).
  void open_gate()
  {
    be->set_io_mode(Stm32Backend::IoMode::ACTIVE_CONTROL);
    be->set_targets_allowed(true);
  }
};

bool contains(const std::vector<uint8_t> & cmds, uint8_t cmd)
{
  for (uint8_t c : cmds) if (c == cmd) return true;
  return false;
}

/// Decode every TARGET frame among the sent frames, in order.
std::vector<Stm32Protocol::Target> collect_targets(
  const std::vector<std::vector<uint8_t>> & frames)
{
  std::vector<Stm32Protocol::Target> out;
  for (const auto & fr : frames) {
    if (fr.size() >= 8U + Stm32Protocol::TARGET_PAYLOAD_LEN &&
        fr[3] == Stm32Protocol::CMD_TARGET) {
      Stm32Protocol::Target t;
      if (Stm32Protocol::decode_target(
            &fr[8], Stm32Protocol::TARGET_PAYLOAD_LEN, t)) {
        out.push_back(t);
      }
    }
  }
  return out;
}

}  // namespace

// ══════════════════════════════════════════════════════════════════
//  Construction / REAL gate
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendGate, ConstructionDoesNotOpenDevice)
{
  BF f;
  EXPECT_EQ(f.mock->open_count(), 0u);
  EXPECT_FALSE(f.be->is_connected());
}

TEST(Stm32BackendGate, IncompleteGateRefusesConnect)
{
  BF f(false);  // REAL gate not passed
  EXPECT_FALSE(f.be->connect());
  EXPECT_EQ(f.mock->open_count(), 0u) << "No device access without the gate";
}

TEST(Stm32BackendGate, UnconfiguredRefusesConnect)
{
  auto s = std::make_unique<MockStm32Serial>();
  auto tr = std::make_shared<Stm32Transport>(std::move(s));
  tr->configure("/dev/ttyUSB0", 115200, 200, 200, 3000);
  Stm32Backend::Timeouts t;
  auto be = std::make_shared<Stm32Backend>(tr, true, t);
  EXPECT_FALSE(be->connect());  // configure() never called
}

// ══════════════════════════════════════════════════════════════════
//  Connect / HELLO handshake
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendConnect, HelloFirstAndOnlyHandshake)
{
  BF f;
  f.connect_ok();
  const auto cmds = f.mock->sent_commands();
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0], Stm32Protocol::CMD_HELLO)
    << "HELLO must be the very first frame — no TARGET/ENABLE at connect";
}

TEST(Stm32BackendConnect, HelloRejectedFailsConnect)
{
  BF f;
  f.mock->set_ack_result(Stm32Protocol::ACK_STATE_DENIED);
  EXPECT_FALSE(f.be->connect());
  EXPECT_FALSE(f.be->is_connected());
  EXPECT_FALSE(f.mock->is_open()) << "Device must be closed after failed handshake";
}

// ══════════════════════════════════════════════════════════════════
//  Startup synchronisation: valid_bitmap == 0x0FFF gate
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendStartup, AllValidPasses)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  pos[3] = 100000;  // L_J4 = 100000 µrad
  f.full_valid(pos);
  EXPECT_TRUE(f.be->wait_all_valid(2000));
  std::array<double, 12> fb;
  ASSERT_TRUE(f.be->current_feedback(fb));
  EXPECT_DOUBLE_EQ(fb[3], 0.1);  // 100000 µrad = 0.1 rad
}

TEST(Stm32BackendStartup, IncompleteValidNeverActivates)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.mock->set_state_values(pos, 0x0FFEu, Stm32Protocol::CTRL_DISABLED);
  // L_J1 (bit0) never becomes valid — startup must not complete.
  EXPECT_FALSE(f.be->wait_all_valid(400));
}

// ══════════════════════════════════════════════════════════════════
//  First target equals current feedback
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendStartup, FirstTargetEqualsFeedback)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  pos[0] = 1000000;    // L_J1 = 1 m
  pos[3] = -2000000;   // L_J4 = -2 rad (µrad)
  pos[6] = 3000000;    // R_J1 = 3 m
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));

  std::array<double, 12> fb;
  ASSERT_TRUE(f.be->current_feedback(fb));
  f.open_gate();  // simulate the activate gate
  f.be->write_targets(fb);   // command = state, per the F.2/F.3 sequence
  f.be->service_step();

  // Left arm target: joints[0] must round-trip back to the µm/µrad value.
  // sent_frames()[0]=HELLO, [1]=GET_STATE(wait_all_valid), then per-round
  // frames [2]=L TARGET, [3]=R TARGET, [4]=GET_STATE(service_step).
  const auto frames = f.mock->sent_frames();
  ASSERT_GE(frames.size(), 5u);
  EXPECT_EQ(frames[2][3], Stm32Protocol::CMD_TARGET);
  EXPECT_EQ(frames[2][8], 0);   // arm byte = left
  Stm32Protocol::Target tl;
  ASSERT_TRUE(Stm32Protocol::decode_target(
    &frames[2][8], Stm32Protocol::TARGET_PAYLOAD_LEN, tl));
  EXPECT_EQ(tl.arm, 0);
  EXPECT_EQ(tl.joint[0], 1000000);
  EXPECT_EQ(tl.joint[3], -2000000);
  // Right arm target.
  EXPECT_EQ(frames[3][3], Stm32Protocol::CMD_TARGET);
  EXPECT_EQ(frames[3][8], 1);   // arm byte = right
  Stm32Protocol::Target trg;
  ASSERT_TRUE(Stm32Protocol::decode_target(
    &frames[3][8], Stm32Protocol::TARGET_PAYLOAD_LEN, trg));
  EXPECT_EQ(trg.arm, 1);
  EXPECT_EQ(trg.joint[0], 3000000);
}

// ══════════════════════════════════════════════════════════════════
//  valid-bitmap retention: never publish placeholder 0
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendState, InvalidKeepsLastKnownValue)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  pos[0] = 1000000;
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.be->service_step();

  // Next state: ALL axes invalid (positions zeroed by the firmware).
  std::array<int32_t, 12> zero{};
  f.mock->set_state_values(zero, 0x0000u, Stm32Protocol::CTRL_ENABLED);
  f.be->service_step();

  IRobotBackend::RobotState st;
  ASSERT_TRUE(f.be->read_state(st));
  // Last known value is kept…
  EXPECT_DOUBLE_EQ(st.position[0], 1.0);
  // …but marked not-fresh so it is never published as real feedback.
  EXPECT_FALSE(st.valid[0]);
  EXPECT_FALSE(st.valid[11]);
}

// ══════════════════════════════════════════════════════════════════
//  No unbounded queue under repeated 20 Hz writes
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendWrite, NoUnboundedQueue)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));   // seed the healthy link
  f.open_gate();

  // Simulate many control cycles between I/O rounds: only the latest
  // target per arm may ever be transmitted.
  std::array<double, 12> t{};
  for (int cycle = 0; cycle < 10000; ++cycle) {
    t[0] = 0.001 * static_cast<double>(cycle);  // millimetre steps, 0..9.999 m
    f.be->write_targets(t);
  }
  f.be->service_step();

  const auto targets = collect_targets(f.mock->sent_frames());
  ASSERT_EQ(targets.size(), 2u) << "Exactly one TARGET per arm, not 10000";
  EXPECT_EQ(targets[0].arm, 0);
  EXPECT_EQ(targets[0].joint[0], 9999 * 1000)
    << "Latest value only (9.999 m → 9,999,000 µm)";
}

// ══════════════════════════════════════════════════════════════════
//  Pending replaces pending; in-flight snapshot stays immutable
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendWrite, InFlightSnapshotNotMutated)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));   // seed the healthy link
  f.open_gate();

  // Round 1: mark a target and keep the response in flight (250 ms delay)
  // while a NEW target arrives — the in-flight frame must keep its own
  // snapshot; only the next round picks up the new value.
  std::array<double, 12> a{};
  a[0] = 1.0;   // L_J1 = 1 m → 1,000,000 µm
  f.be->write_targets(a);

  f.mock->set_response_delay_ms(250);
  std::thread t1([&]() { f.be->service_step(); });

  std::this_thread::sleep_for(100ms);
  std::array<double, 12> b{};
  b[0] = 2.0;   // L_J1 = 2 m → 2,000,000 µm
  f.be->write_targets(b);

  t1.join();
  f.mock->set_response_delay_ms(0);
  f.be->service_step();   // round 2

  // TARGET frames in order: round-1 L (snapshot A) and R, then round-2 L
  // (the newest target B). R is not re-sent in round 2: its dirty flag was
  // consumed by round 1 and no new R target arrived (latest-wins).
  const auto targets = collect_targets(f.mock->sent_frames());
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_EQ(targets[0].arm, 0);
  EXPECT_EQ(targets[0].joint[0], 1000000)
    << "Round-1 in-flight frame keeps snapshot A";
  EXPECT_EQ(targets[1].arm, 1);
  EXPECT_EQ(targets[2].arm, 0);
  EXPECT_EQ(targets[2].joint[0], 2000000)
    << "Round-2 sends the newest target B";
}

// ══════════════════════════════════════════════════════════════════
//  Per-arm latest targets
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendWrite, LeftAndRightLatestTargets)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));   // seed the healthy link
  f.open_gate();

  std::array<double, 12> t{};
  t[0] = 0.5;    // L_J1 = 0.5 m
  t[6] = -0.25;  // R_J1 = -0.25 m
  t[11] = 0.75;  // R_J6 = 0.75 rad
  f.be->write_targets(t);
  f.be->service_step();

  const auto targets = collect_targets(f.mock->sent_frames());
  ASSERT_EQ(targets.size(), 2u);
  EXPECT_EQ(targets[0].arm, 0);
  EXPECT_EQ(targets[0].joint[0], 500000);
  EXPECT_EQ(targets[1].arm, 1);
  EXPECT_EQ(targets[1].joint[0], -250000);
  EXPECT_EQ(targets[1].joint[5], 750000);
}

// ══════════════════════════════════════════════════════════════════
//  0x0B SUPERSEDED is not a hardware fault
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendAck, SupersededNotTreatedAsFault)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));   // seed the healthy link
  f.open_gate();
  f.mock->set_ack_result(Stm32Protocol::ACK_SUPERSEDED);
  std::array<double, 12> t{};
  t[0] = 1.0;
  f.be->write_targets(t);
  f.be->service_step();

  auto st = f.be->stats();
  EXPECT_EQ(st.target_superseded[0], 1u);
  EXPECT_EQ(st.target_ack_ok[0], 0u);
  EXPECT_EQ(st.target_timeout[0], 0u) << "SUPERSEDED is intentional, not a timeout";
}

// ══════════════════════════════════════════════════════════════════
//  Control sequences: ENABLE / STOP / DISABLE (delayed ACK)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendControl, EnableStopDisableOrder)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));

  EXPECT_TRUE(f.be->enable());
  EXPECT_TRUE(f.be->stop());
  EXPECT_TRUE(f.be->disable());

  // HELLO, GET_STATE (wait_all_valid), then the three control sequences.
  const auto cmds = f.mock->sent_commands();
  ASSERT_GE(cmds.size(), 5u);
  EXPECT_EQ(cmds[0], Stm32Protocol::CMD_HELLO);
  EXPECT_EQ(cmds[1], Stm32Protocol::CMD_GET_STATE);
  EXPECT_EQ(cmds[2], Stm32Protocol::CMD_ENABLE);
  EXPECT_EQ(cmds[3], Stm32Protocol::CMD_STOP);
  EXPECT_EQ(cmds[4], Stm32Protocol::CMD_DISABLE);
}

TEST(Stm32BackendControl, EnableFailurePropagates)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.mock->set_ack_result(Stm32Protocol::ACK_CTRL_FAILED);
  EXPECT_FALSE(f.be->enable());
}

TEST(Stm32BackendControl, ControlNeedsConnection)
{
  BF f;
  EXPECT_FALSE(f.be->enable());
  EXPECT_FALSE(f.be->stop());
  EXPECT_FALSE(f.be->disable());
}

// ══════════════════════════════════════════════════════════════════
//  Different timeouts per command class
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendTimeout, ControlLongerThanTargetAndState)
{
  // target/state timeouts short (80 ms), control sequence timeout long (400 ms).
  Stm32Backend::Timeouts t;
  t.target_ack_ms = 80;
  t.state_ms = 80;
  t.control_ack_ms = 400;
  BF f(true, t);
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  // Seed a healthy link first (gates need fresh feedback), then go silent.
  ASSERT_TRUE(f.be->wait_all_valid(500));
  f.open_gate();
  f.mock->set_no_response(true);  // everything times out

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(f.be->enable());  // control ACK timeout
  const auto d_enable = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(d_enable, 320ms) << "ENABLE waits for the long control timeout";

  const auto t1 = std::chrono::steady_clock::now();
  std::array<double, 12> cmd{};
  cmd[0] = 1.0;
  f.be->write_targets(cmd);
  f.be->service_step();  // L(80) + R(80) + state(80) ≈ 240 ms
  const auto d_target = std::chrono::steady_clock::now() - t1;
  EXPECT_LE(d_target, 300ms) << "TARGET/STATE use the short timeouts";

  auto st = f.be->stats();
  EXPECT_EQ(st.target_timeout[0], 1u);
  EXPECT_EQ(st.target_timeout[1], 1u);
  EXPECT_EQ(st.state_timeout, 1u);
}

// ══════════════════════════════════════════════════════════════════
//  read_state before any STATE
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendState, ReadBeforeAnyState)
{
  BF f;
  f.connect_ok();
  IRobotBackend::RobotState st;
  EXPECT_FALSE(f.be->read_state(st));
  EXPECT_FALSE(st.got_state);
}

// ══════════════════════════════════════════════════════════════════
//  Control ACK timeout — delayed ACK must not be misjudged
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendControl, ControlAckDefaultIs10s)
{
  Stm32Backend::Timeouts t;
  EXPECT_EQ(t.control_ack_ms, 10000)
    << "The real firmware returns the ENABLE/STOP/DISABLE ACK only after the "
       "whole-robot sequence completes (~8 s) — 3 s would misjudge it";
}

TEST(Stm32BackendControl, DelayedAckHonoredWithDefaultTimeout)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  // Simulate the firmware's delayed control ACK (real value ≈ 8 s; use a
  // proportionally short delay so the test stays fast while exercising the
  // "wait for the sequence to finish" path).
  f.mock->set_response_delay_ms(250);
  EXPECT_TRUE(f.be->enable());
  EXPECT_TRUE(f.be->stop());
  EXPECT_TRUE(f.be->disable());
}

TEST(Stm32BackendControl, TooShortControlTimeoutMisjudgesDelayedAck)
{
  // A short control timeout (mirroring the 3 s misconfiguration, scaled
  // down) must FAIL on a normally-delayed ACK — proving the contradiction.
  Stm32Backend::Timeouts t;
  t.control_ack_ms = 100;
  BF f(true, t);
  f.connect_ok();
  f.mock->set_response_delay_ms(250);
  EXPECT_FALSE(f.be->enable());
}

// ══════════════════════════════════════════════════════════════════
//  Target gate: read-only connect vs activate
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendGate, TargetsBlockedBeforeActivate)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));

  // Pre-activate: write_targets() is refused and nothing is transmitted.
  std::array<double, 12> t{};
  t[0] = 1.0;
  EXPECT_FALSE(f.be->write_targets(t));
  f.be->service_step();
  EXPECT_FALSE(contains(f.mock->sent_commands(), Stm32Protocol::CMD_TARGET))
    << "no TARGET may be sent before the activate gate opens";

  // After the gate opens, targets flow.
  f.open_gate();
  EXPECT_TRUE(f.be->write_targets(t));
  f.be->service_step();
  EXPECT_TRUE(contains(f.mock->sent_commands(), Stm32Protocol::CMD_TARGET));
}

// ══════════════════════════════════════════════════════════════════
//  Sustained feedback loss: stop commanding from stale data
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendState, StaleFeedbackRefusesNewTargets)
{
  Stm32Backend::Timeouts t;
  t.state_stale_ms = 100;
  BF f(true, t);
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.open_gate();

  std::array<double, 12> cmd{};
  cmd[0] = 1.0;
  EXPECT_TRUE(f.be->write_targets(cmd));   // healthy link — accepted
  f.be->service_step();

  // The link loses all-valid feedback; once the stale window passes the
  // backend must refuse new targets instead of commanding from old data.
  f.mock->set_state_values({}, 0x0000u, Stm32Protocol::CTRL_ENABLED);
  std::this_thread::sleep_for(150ms);
  f.be->service_step();   // ingest the invalid STATE

  EXPECT_FALSE(f.be->link_healthy());
  EXPECT_FALSE(f.be->write_targets(cmd)) << "stale feedback — no new targets";
}

// ══════════════════════════════════════════════════════════════════
//  STATE poll rate is capped by state_poll_hz
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendIo, PollRateCappedByStatePollHz)
{
  Stm32Backend::Timeouts t;
  t.state_poll_hz = 10.0;   // 100 ms period
  t.state_ms = 50;
  BF f(true, t);
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);

  f.be->start_io(Stm32Backend::IoMode::ACTIVE_CONTROL);
  std::this_thread::sleep_for(450ms);
  f.be->stop_io();

  size_t states = 0;
  for (uint8_t c : f.mock->sent_commands()) {
    if (c == Stm32Protocol::CMD_GET_STATE) ++states;
  }
  // ~450 ms at a 100 ms period → ≈5 GET_STATE rounds. Without the rate cap
  // the loop would spin (hundreds of GET_STATE) since responses are instant.
  EXPECT_GE(states, 2u);
  EXPECT_LE(states, 8u) << "state_poll_hz must throttle the GET_STATE rate";
}

// ══════════════════════════════════════════════════════════════════
//  Read-only mode: GET_STATE only, never any control/target frame
//  (review task §3.5 / §8.4)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendReadOnly, DefaultModeIsReadOnlyAndBlocksTargets)
{
  BF f;
  f.connect_ok();
  EXPECT_EQ(f.be->io_mode(), Stm32Backend::IoMode::READ_ONLY);
  EXPECT_FALSE(f.be->targets_allowed());
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));

  std::array<double, 12> t{};
  t[0] = 1.0;
  EXPECT_FALSE(f.be->write_targets(t)) << "read-only: targets refused";
  f.be->service_step();
  // HELLO is the only non-GET_STATE frame (from connect()); after that,
  // read-only rounds must never send ENABLE/TARGET.
  const auto cmds = f.mock->sent_commands();
  size_t idx = 0;
  for (; idx < cmds.size(); ++idx) {
    if (cmds[idx] != Stm32Protocol::CMD_HELLO) break;
  }
  for (; idx < cmds.size(); ++idx) {
    EXPECT_EQ(cmds[idx], Stm32Protocol::CMD_GET_STATE)
      << "read-only must only send GET_STATE after HELLO";
  }
  EXPECT_FALSE(contains(cmds, Stm32Protocol::CMD_ENABLE));
  EXPECT_FALSE(contains(cmds, Stm32Protocol::CMD_TARGET));
}

TEST(Stm32BackendReadOnly, ServiceStepSendsOnlyGetState)
{
  BF f;
  f.connect_ok();
  f.be->set_io_mode(Stm32Backend::IoMode::READ_ONLY);
  std::array<double, 12> t{};
  t[0] = 1.0;
  f.be->write_targets(t);  // must be dropped
  f.be->service_step();

  const auto cmds = f.mock->sent_commands();
  size_t idx = 0;
  for (; idx < cmds.size(); ++idx) {
    if (cmds[idx] != Stm32Protocol::CMD_HELLO) break;
  }
  for (; idx < cmds.size(); ++idx) {
    EXPECT_EQ(cmds[idx], Stm32Protocol::CMD_GET_STATE)
      << "READ_ONLY rounds must only ever send GET_STATE (got 0x" << std::hex
      << int(cmds[idx]) << std::dec << ")";
  }
}

TEST(Stm32BackendReadOnly, SettingTargetsAllowedInReadOnlyHasNoEffect)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));

  f.be->set_targets_allowed(true);   // mode is still READ_ONLY
  EXPECT_FALSE(f.be->targets_allowed());
  std::array<double, 12> t{};
  t[0] = 1.0;
  EXPECT_FALSE(f.be->write_targets(t));
}

TEST(Stm32BackendReadOnly, ReadOnlyDisableStateNotAHealthFailure)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  // Read-only mode with control_state DISABLED must NOT be flagged as a
  // link/health failure — it is the nominal F.2 state.
  EXPECT_EQ(f.be->health(), Stm32Backend::Health::READ_ONLY);
}

// ══════════════════════════════════════════════════════════════════
//  TARGET result escalation (review task §8.3)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendTarget, StateDeniedClosesGateImmediately)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.open_gate();

  f.mock->set_ack_result(Stm32Protocol::ACK_STATE_DENIED);
  std::array<double, 12> t{};
  t[0] = 1.0;
  f.be->write_targets(t);
  f.be->service_step();

  EXPECT_TRUE(f.be->target_escalated());
  EXPECT_FALSE(f.be->targets_allowed());
  EXPECT_EQ(f.be->health(), Stm32Backend::Health::TARGET_REJECTED);
}

TEST(Stm32BackendTarget, CtrlFailedClosesGate)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.open_gate();

  f.mock->set_ack_result(Stm32Protocol::ACK_CTRL_FAILED);
  std::array<double, 12> t{};
  t[0] = 1.0;
  f.be->write_targets(t);
  f.be->service_step();

  EXPECT_TRUE(f.be->target_escalated());
  EXPECT_FALSE(f.be->targets_allowed());
}

TEST(Stm32BackendTarget, OutOfRangeDoesNotResendAndEscalatesAfterThreshold)
{
  BF f;
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.open_gate();

  // A target that overflows int32 (e.g. L_J1 = 1e9 m) is accepted into the
  // cache by write_targets() but MUST be rejected atomically when the frame
  // is built — the whole LEFT arm frame is refused, nothing is clamped/sent.
  f.mock->set_ack_result(Stm32Protocol::ACK_OUT_OF_RANGE);
  std::array<double, 12> t{};
  t[0] = 1e9;  // 1e15 µm — far beyond int32
  EXPECT_TRUE(f.be->write_targets(t));   // cached (gates are about freshness)
  f.be->service_step();
  const auto targets = collect_targets(f.mock->sent_frames());
  for (const auto & tg : targets) {
    EXPECT_NE(tg.arm, 0) << "no LEFT-arm TARGET may be sent for an "
                            "out-of-range arm";
  }
  auto st = f.be->stats();
  EXPECT_GE(st.target_rejected[0], 1u) << "rejection must be counted";
}

// ══════════════════════════════════════════════════════════════════
//  Deadline scheduling (review task §9)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32BackendIo, NoExtraSleepWhenRoundExceedsPeriod)
{
  // A slow round (250 ms delay) with a 100 ms period must NOT sleep again
  // after it completes — the total round time is bounded by the comms, and
  // io_overruns is counted.
  Stm32Backend::Timeouts t;
  t.state_poll_hz = 10.0;   // 100 ms period
  t.state_ms = 50;
  t.target_ack_ms = 50;
  BF f(true, t);
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  f.full_valid(pos);
  ASSERT_TRUE(f.be->wait_all_valid(2000));
  f.open_gate();

  // Keep a target pending so every round transmits both arms + STATE.
  std::array<double, 12> cmd{};
  cmd[0] = 1.0;
  f.be->write_targets(cmd);

  f.mock->set_response_delay_ms(250);
  f.be->start_io(Stm32Backend::IoMode::ACTIVE_CONTROL);
  std::this_thread::sleep_for(1200ms);
  f.be->stop_io();
  f.mock->set_response_delay_ms(0);

  auto st = f.be->stats();
  EXPECT_GT(st.io_overruns, 0u) << "Slow rounds must be counted as overruns";
  EXPECT_GE(st.max_loop_period_us, 150000.0)
    << "The measured loop must reflect the real (slow) round time";
}

TEST(Stm32BackendIo, WaitAllValidIsRateLimited)
{
  Stm32Backend::Timeouts t;
  t.state_poll_hz = 50.0;   // 20 ms — fast poll, still bounded
  t.state_ms = 20;
  BF f(true, t);
  f.connect_ok();
  std::array<int32_t, 12> pos{};
  // L_J1 (bit0) NEVER becomes valid → wait_all_valid loops until timeout,
  // which is exactly what must be rate-limited.
  f.mock->set_state_values(pos, 0x0FFEu, Stm32Protocol::CTRL_DISABLED);
  EXPECT_FALSE(f.be->wait_all_valid(500));

  // Over a 500 ms budget at 50 Hz the GET_STATE count must be bounded
  // (~25 expected); an unthrottled loop would be many times more.
  size_t states = 0;
  for (uint8_t c : f.mock->sent_commands()) {
    if (c == Stm32Protocol::CMD_GET_STATE) ++states;
  }
  EXPECT_GE(states, 5u);
  EXPECT_LE(states, 60u) << "wait_all_valid must be rate-limited";
}

TEST(Stm32BackendLifecycle, CloseStopsThreadAndIsIdempotent)
{
  BF f;
  f.connect_ok();
  f.be->start_io(Stm32Backend::IoMode::ACTIVE_CONTROL);
  EXPECT_TRUE(f.be->is_running());
  f.be->close();
  EXPECT_FALSE(f.be->is_running());
  EXPECT_FALSE(f.be->is_connected());
  f.be->close();  // idempotent
  SUCCEED();
}
