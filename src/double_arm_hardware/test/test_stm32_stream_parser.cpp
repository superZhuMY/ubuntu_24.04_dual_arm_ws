/// Offline tests for the STM32 stream parser (stm32_stream_parser.hpp).
///
/// Mirrors the firmware-side Aimotor_HostStreamPoll semantics: split /
/// coalesce / resync / CRC rejection / bad-LEN skipping / partial frames.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "double_arm_hardware/stm32_protocol.hpp"
#include "double_arm_hardware/stm32_stream_parser.hpp"

using namespace double_arm_hardware;

namespace
{

std::vector<uint8_t> hello_frame(uint16_t seq)
{
  return Stm32Protocol::build_frame(Stm32Protocol::CMD_HELLO, seq, {});
}

}  // namespace

// ══════════════════════════════════════════════════════════════════
//  Basic single-frame parsing
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, SingleFrame)
{
  Stm32StreamParser p;
  p.append(hello_frame(7));
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_TRUE(frames[0].ok);
  EXPECT_EQ(frames[0].cmd, Stm32Protocol::CMD_HELLO);
  EXPECT_EQ(frames[0].seq, 7);
  EXPECT_TRUE(frames[0].payload.empty());
}

// ══════════════════════════════════════════════════════════════════
//  Split packets (partial frame arrives over several appends)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, SplitPacketByteByByte)
{
  Stm32StreamParser p;
  const auto frame = hello_frame(3);
  std::vector<Stm32StreamParser::Frame> all;
  for (uint8_t b : frame) {
    p.append(&b, 1);
    auto frames = p.poll();
    all.insert(all.end(), frames.begin(), frames.end());
  }
  ASSERT_EQ(all.size(), 1u) << "Exactly one frame after all bytes arrive";
  EXPECT_TRUE(all[0].ok);
  EXPECT_EQ(all[0].seq, 3);
}

TEST(Stm32StreamParser, PartialFrameWaits)
{
  Stm32StreamParser p;
  const auto frame = hello_frame(9);
  p.append(std::vector<uint8_t>(frame.begin(), frame.begin() + 7));
  EXPECT_TRUE(p.poll().empty()) << "Header-only — not a complete frame";
  p.append(std::vector<uint8_t>(frame.begin() + 7, frame.end()));
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 9);
}

// ══════════════════════════════════════════════════════════════════
//  Coalesced frames (several frames in one buffer)
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, MultipleFramesOnePoll)
{
  Stm32StreamParser p;
  auto a = hello_frame(1);
  auto b = hello_frame(2);
  auto c = hello_frame(3);
  a.insert(a.end(), b.begin(), b.end());
  a.insert(a.end(), c.begin(), c.end());
  p.append(a);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0].seq, 1);
  EXPECT_EQ(frames[1].seq, 2);
  EXPECT_EQ(frames[2].seq, 3);
}

// ══════════════════════════════════════════════════════════════════
//  Noise / resynchronisation
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, NoiseBeforeFrameIsDropped)
{
  Stm32StreamParser p;
  std::vector<uint8_t> data = {0x00, 0x01, 0x55, 0xAA, 0x42};
  auto frame = hello_frame(4);
  data.insert(data.end(), frame.begin(), frame.end());
  p.append(data);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 4);
}

TEST(Stm32StreamParser, AaNotFollowedBy55Resyncs)
{
  Stm32StreamParser p;
  std::vector<uint8_t> data = {0xAA, 0x00, 0x01, 0x02};
  auto frame = hello_frame(5);
  data.insert(data.end(), frame.begin(), frame.end());
  p.append(data);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 5);
}

// ══════════════════════════════════════════════════════════════════
//  CRC errors
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, CrcErrorFrameReportedBad)
{
  Stm32StreamParser p;
  auto frame = hello_frame(6);
  frame[8] ^= 0xFF;  // corrupt CRC low byte
  p.append(frame);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_FALSE(frames[0].ok);
  EXPECT_EQ(frames[0].seq, 6);
  EXPECT_EQ(p.bad_frame_count(), 1u);
}

TEST(Stm32StreamParser, BadFrameThenGoodFrame)
{
  Stm32StreamParser p;
  auto bad = hello_frame(1);
  bad[8] ^= 0x01;
  auto good = hello_frame(2);
  bad.insert(bad.end(), good.begin(), good.end());
  p.append(bad);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_FALSE(frames[0].ok);
  EXPECT_TRUE(frames[1].ok);
  EXPECT_EQ(frames[1].seq, 2);
}

TEST(Stm32StreamParser, WrongVersionReportedBad)
{
  Stm32StreamParser p;
  auto frame = hello_frame(8);
  frame[2] = 0x02;  // VERSION mismatch
  p.append(frame);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_FALSE(frames[0].ok);
}

// ══════════════════════════════════════════════════════════════════
//  Bad LEN
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, BadLenSkippedAndResynced)
{
  Stm32StreamParser p;
  // Frame with LEN = 0xFF00 (> MAX_PAYLOAD): header byte skipped, then the
  // next valid frame must still be parsed.
  std::vector<uint8_t> data = {0xAA, 0x55, 0x01, 0x10,
                               0x00, 0x00, 0x00, 0xFF};
  auto good = hello_frame(11);
  data.insert(data.end(), good.begin(), good.end());
  p.append(data);
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_TRUE(frames[0].ok);
  EXPECT_EQ(frames[0].seq, 11);
  EXPECT_GE(p.bad_frame_count(), 1u);
}

// ══════════════════════════════════════════════════════════════════
//  Ring buffer behaviour
// ══════════════════════════════════════════════════════════════════

TEST(Stm32StreamParser, RingWrapKeepsParsing)
{
  Stm32StreamParser p;
  // Fill the ring with noise so a frame's bytes wrap around the end.
  std::vector<uint8_t> fill(Stm32StreamParser::BUFFER_SIZE - 10, 0x00);
  p.append(fill);
  p.append(hello_frame(12));
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 12);
}

TEST(Stm32StreamParser, OverflowDropsOldest)
{
  Stm32StreamParser p;
  std::vector<uint8_t> big(Stm32StreamParser::BUFFER_SIZE * 2, 0x00);
  p.append(big);
  EXPECT_GT(p.dropped_bytes(), 0u);
  // After dropping everything, appending a frame still parses cleanly.
  p.append(hello_frame(13));
  auto frames = p.poll();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 13);
}

TEST(Stm32StreamParser, ResetClearsState)
{
  Stm32StreamParser p;
  p.append(hello_frame(14));
  p.reset();
  EXPECT_EQ(p.buffered_bytes(), 0u);
  EXPECT_TRUE(p.poll().empty());
}
