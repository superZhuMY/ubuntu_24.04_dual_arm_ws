/// Offline tests for Stm32Transport (stm32_transport.hpp) using a mock
/// serial backend that emulates the STM32 V1.3.1 firmware.
///
/// Key semantics verified:
///   - configure()/open()/close() lifecycle, idempotency, no auto-open;
///   - HELLO/ENABLE/STOP/DISABLE/TARGET answered by an ACK with matching SEQ;
///   - GET_STATE answered by a STATE frame, never by a plain ACK;
///   - SEQ matching: a stale ACK for another SEQ is not accepted;
///   - split/coalesced bytes are reassembled by the internal parser;
///   - response timeouts and failure paths.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "double_arm_hardware/serial_backend.hpp"
#include "double_arm_hardware/stm32_mock_serial.hpp"
#include "double_arm_hardware/stm32_transport.hpp"

using namespace double_arm_hardware;

namespace
{

struct Fixture
{
  MockStm32Serial * mock = nullptr;
  std::shared_ptr<Stm32Transport> t;

  Fixture()
  {
    auto s = std::make_unique<MockStm32Serial>();
    mock = s.get();
    t = std::make_shared<Stm32Transport>(std::move(s));
  }

  void cfg_and_open()
  {
    ASSERT_TRUE(t->configure("/dev/ttyUSB0", 115200, 200, 200, 3000));
    ASSERT_TRUE(t->open());
  }
};

}  // namespace

// ══════════════════════════════════════════════════════════════════
//  Lifecycle
// ══════════════════════════════════════════════════════════════════

TEST(Stm32TransportLifecycle, ConfigureDoesNotOpen)
{
  Fixture f;
  ASSERT_TRUE(f.t->configure("/dev/ttyUSB0", 115200, 200, 200, 3000));
  EXPECT_FALSE(f.t->is_open());
  EXPECT_FALSE(f.mock->is_open());
}

TEST(Stm32TransportLifecycle, OpenAfterConfigure)
{
  Fixture f;
  f.cfg_and_open();
  EXPECT_TRUE(f.t->is_open());
  EXPECT_TRUE(f.mock->is_open());
}

TEST(Stm32TransportLifecycle, OpenWithoutConfigureFails)
{
  Fixture f;
  EXPECT_FALSE(f.t->open());
}

TEST(Stm32TransportLifecycle, CloseIsIdempotent)
{
  Fixture f;
  f.cfg_and_open();
  f.t->close();
  const size_t n = f.mock->close_count();
  f.t->close();
  EXPECT_EQ(f.mock->close_count(), n);
  EXPECT_FALSE(f.t->is_open());
}

TEST(Stm32TransportLifecycle, ConfigureRejectsBadParams)
{
  Fixture f;
  EXPECT_FALSE(f.t->configure("", 115200, 200, 200, 3000));
  EXPECT_FALSE(f.t->configure("/dev/ttyUSB0", 0, 200, 200, 3000));
  EXPECT_FALSE(f.t->configure("/dev/ttyUSB0", 115200, 0, 200, 3000));
}

TEST(Stm32TransportLifecycle, SerialParamsPassed)
{
  Fixture f;
  f.cfg_and_open();
  EXPECT_EQ(f.mock->last_device(), "/dev/ttyUSB0");
  EXPECT_EQ(f.mock->last_baudrate(), 115200);
}

TEST(Stm32TransportLifecycle, OpenFailureDoesNotOpen)
{
  Fixture f;
  ASSERT_TRUE(f.t->configure("/dev/ttyUSB0", 115200, 200, 200, 3000));
  f.mock->set_fail_open(true);
  EXPECT_FALSE(f.t->open());
  EXPECT_FALSE(f.t->is_open());
}

// ══════════════════════════════════════════════════════════════════
//  ACK requests (HELLO / ENABLE / STOP / DISABLE / TARGET)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32TransportAck, HelloOk)
{
  Fixture f;
  f.cfg_and_open();
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 1, {}, 200);
  EXPECT_EQ(res, Stm32Protocol::ACK_OK);
  const auto cmds = f.mock->sent_commands();
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0], Stm32Protocol::CMD_HELLO);
  EXPECT_EQ(f.mock->last_seq(), 1);
}

TEST(Stm32TransportAck, AckResultPropagated)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_ack_result(Stm32Protocol::ACK_STATE_DENIED);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_TARGET, 2, {}, 200);
  EXPECT_EQ(res, Stm32Protocol::ACK_STATE_DENIED);
}

TEST(Stm32TransportAck, SupersededResult)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_ack_result(Stm32Protocol::ACK_SUPERSEDED);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_TARGET, 3, {}, 200);
  EXPECT_EQ(res, Stm32Protocol::ACK_SUPERSEDED);
}

TEST(Stm32TransportAck, TimeoutReturnsMinusOne)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_no_response(true);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 4, {}, 100);
  EXPECT_EQ(res, -1);
}

TEST(Stm32TransportAck, WriteFailure)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_fail_write(true);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 5, {}, 100);
  EXPECT_EQ(res, -1);
}

TEST(Stm32TransportAck, NotOpenFails)
{
  Fixture f;
  ASSERT_TRUE(f.t->configure("/dev/ttyUSB0", 115200, 200, 200, 3000));
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 6, {}, 100);
  EXPECT_EQ(res, -1);
}

// ══════════════════════════════════════════════════════════════════
//  SEQ matching
// ══════════════════════════════════════════════════════════════════

TEST(Stm32TransportSeq, StaleAckDifferentSeqNotAccepted)
{
  Fixture f;
  f.cfg_and_open();
  // Mock echoes ACK_OK but for a different SEQ (99) — the transport must
  // NOT accept it for our request (seq 7) and must time out instead.
  f.mock->set_ack_seq_override(99);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 7, {}, 100);
  EXPECT_EQ(res, -1);
}

// ══════════════════════════════════════════════════════════════════
//  GET_STATE
// ══════════════════════════════════════════════════════════════════

TEST(Stm32TransportState, GetStateReturnsState)
{
  Fixture f;
  f.cfg_and_open();
  std::array<int32_t, 12> pos{};
  pos[0] = 1234;
  pos[11] = -5678;
  f.mock->set_state_values(pos, 0x0FFFu, Stm32Protocol::CTRL_ENABLED);

  Stm32Protocol::State st;
  const int res = f.t->send_get_state(9, 200, st);
  EXPECT_EQ(res, 0);
  EXPECT_EQ(st.seq, 9);
  EXPECT_EQ(st.control_state, Stm32Protocol::CTRL_ENABLED);
  EXPECT_EQ(st.valid, 0x0FFFu);
  EXPECT_EQ(st.position[0], 1234);
  EXPECT_EQ(st.position[11], -5678);
  // GET_STATE must never be answered by a plain ACK — verify the mock
  // produced a STATE response by checking the sent command was GET_STATE.
  const auto cmds = f.mock->sent_commands();
  ASSERT_GE(cmds.size(), 1u);
  EXPECT_EQ(cmds.back(), Stm32Protocol::CMD_GET_STATE);
}

TEST(Stm32TransportState, GetStateTimeout)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_no_response(true);
  Stm32Protocol::State st;
  const int res = f.t->send_get_state(10, 100, st);
  EXPECT_EQ(res, -1);
}

TEST(Stm32TransportState, SplitResponseStillParsed)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_split_reads(true);
  std::array<int32_t, 12> pos{};
  pos[5] = 42;
  f.mock->set_state_values(pos, 0x0FFFu, Stm32Protocol::CTRL_STOPPED);
  Stm32Protocol::State st;
  const int res = f.t->send_get_state(11, 500, st);
  EXPECT_EQ(res, 0);
  EXPECT_EQ(st.position[5], 42);
  EXPECT_EQ(st.control_state, Stm32Protocol::CTRL_STOPPED);
}

// ══════════════════════════════════════════════════════════════════
//  SEQ + CMD double matching (review task §7)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32TransportSeq, MatchingSeqAccepted)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_ack_seq_override(7);  // same seq as the request — accepted
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 7, {}, 200);
  EXPECT_EQ(res, Stm32Protocol::ACK_OK);
}

TEST(Stm32TransportSeq, SeqCorrectCmdWrongRejected)
{
  Fixture f;
  f.cfg_and_open();
  // ACK carries the right SEQ but the WRONG command (TARGET instead of
  // HELLO) — must be rejected and end in a timeout.
  f.mock->set_ack_cmd_override(Stm32Protocol::CMD_TARGET);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 8, {}, 100);
  EXPECT_EQ(res, -1);
  EXPECT_GT(f.t->ack_cmd_mismatch_count(), 0);
}

TEST(Stm32TransportSeq, EnableRequestTargetAckRejected)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_ack_cmd_override(Stm32Protocol::CMD_TARGET);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_ENABLE, 9, {}, 100);
  EXPECT_EQ(res, -1) << "ENABLE must never accept a TARGET ACK";
}

TEST(Stm32TransportSeq, TargetRequestHelloAckRejected)
{
  Fixture f;
  f.cfg_and_open();
  f.mock->set_ack_cmd_override(Stm32Protocol::CMD_HELLO);
  const int res = f.t->send_ack_request(Stm32Protocol::CMD_TARGET, 10, {}, 100);
  EXPECT_EQ(res, -1) << "TARGET must never accept a HELLO ACK";
}

TEST(Stm32TransportSeq, MismatchThenCorrectStillMatches)
{
  Fixture f;
  f.cfg_and_open();
  // First response has the wrong CMD; the mock then answers the SAME
  // request correctly? No — the mock answers per request. Instead, verify
  // that a mismatched frame followed by a matching one (injected raw)
  // completes successfully.
  // Inject a stale ACK (seq 999, cmd HELLO) BEFORE the mock's own reply.
  Stm32Protocol::Ack stale_ack{999, Stm32Protocol::CMD_HELLO,
                               Stm32Protocol::ACK_OK};
  std::vector<uint8_t> stale_payload = {
    static_cast<uint8_t>(stale_ack.seq & 0xFF),
    static_cast<uint8_t>(stale_ack.seq >> 8),
    stale_ack.cmd, stale_ack.result};
  auto stale_frame = Stm32Protocol::build_frame(
    Stm32Protocol::RESP_ACK, 999, stale_payload);
  f.mock->inject_raw(stale_frame);

  const int res = f.t->send_ack_request(Stm32Protocol::CMD_HELLO, 11, {}, 200);
  EXPECT_EQ(res, Stm32Protocol::ACK_OK)
    << "A stale mismatched ACK must not block a later correct ACK";
  EXPECT_GT(f.t->ack_seq_mismatch_count(), 0);
}

TEST(Stm32TransportState, HeaderPayloadSeqMismatchRejected)
{
  Fixture f;
  f.cfg_and_open();
  // STATE frame whose header SEQ matches the request but whose payload SEQ
  // differs — must be rejected (injected before the mock's own STATE reply).
  std::vector<uint8_t> payload(60, 0);
  payload[0] = 77;  // payload SEQ = 77 ≠ request seq 12
  payload[1] = 0;
  payload[2] = Stm32Protocol::CTRL_ENABLED;
  auto bad = Stm32Protocol::build_frame(
    Stm32Protocol::RESP_STATE, 12, payload);  // header SEQ = 12
  f.mock->inject_raw(bad);

  Stm32Protocol::State st;
  const int res = f.t->send_get_state(12, 200, st);
  EXPECT_EQ(res, 0) << "The injected mismatch must be skipped, the mock's "
                       "correct STATE must complete the request";
  EXPECT_GT(f.t->state_seq_mismatch_count(), 0);
}

// ══════════════════════════════════════════════════════════════════
//  Target frame transport
// ══════════════════════════════════════════════════════════════════

TEST(Stm32TransportTarget, TargetFrameRoundTrip)
{
  Fixture f;
  f.cfg_and_open();
  Stm32Protocol::Target t;
  t.arm = 1;
  t.joint = {1000, 2000, 3000, 4000, 5000, 6000};
  const int res = f.t->send_ack_request(
    Stm32Protocol::CMD_TARGET, 12, Stm32Protocol::encode_target(t), 200);
  EXPECT_EQ(res, Stm32Protocol::ACK_OK);
  EXPECT_EQ(f.mock->last_target().arm, 1);
  EXPECT_EQ(f.mock->last_target().joint, t.joint);
}
