#ifndef DOUBLE_ARM_HARDWARE__STM32_STREAM_PARSER_HPP_
#define DOUBLE_ARM_HARDWARE__STM32_STREAM_PARSER_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "double_arm_hardware/stm32_protocol.hpp"

namespace double_arm_hardware
{

/// Byte-stream frame parser for the STM32 binary protocol.
///
/// Mirrors the firmware-side `Aimotor_HostStreamPoll` behaviour:
///   - split / coalesce (one `poll()` may return several complete frames);
///   - resynchronise after noise (bytes not starting with AA 55 are skipped);
///   - reject bad LEN (> PROTOCOL_MAX_PAYLOAD) by skipping the frame-header
///     byte and resynchronising (counted as a bad frame);
///   - discard frames whose VERSION or CRC check fails (returned with
///     `ok == false` so the caller can count them — never delivered as data);
///   - wait for more data when only a partial frame is buffered.
///
/// Pure state machine, no I/O. Owned and called from a single thread
/// (the Stm32 I/O thread) so it needs no locking.
class Stm32StreamParser
{
public:
  /// Result of a poll: a parsed frame, or a discarded invalid frame.
  struct Frame
  {
    uint8_t cmd = 0;
    uint16_t seq = 0;
    std::vector<uint8_t> payload;
    bool ok = false;  ///< true = VERSION + CRC verified, valid frame
  };

  static constexpr size_t BUFFER_SIZE = 256U;  // PROTOCOL_RX_BUFFER_SIZE

  /// Append raw bytes (from the serial read). Returns false when the ring
  /// buffer overflowed (oldest bytes were dropped and counted).
  bool append(const uint8_t * data, size_t len);

  /// Append a byte vector.
  bool append(const std::vector<uint8_t> & data)
  {
    return append(data.data(), data.size());
  }

  /// Parse all complete frames currently buffered.
  /// @return frames in arrival order; invalid frames have `ok == false`.
  std::vector<Frame> poll();

  /// Number of bytes dropped because the ring buffer was full.
  size_t dropped_bytes() const { return dropped_bytes_; }

  /// Number of frames discarded for bad LEN / VERSION / CRC.
  size_t bad_frame_count() const { return bad_frame_count_; }

  /// Bytes still buffered (partial frame / noise).
  size_t buffered_bytes() const { return avail(); }

  /// Reset parser state (drop everything, keep counters? no — full reset).
  void reset();

private:
  size_t avail() const
  {
    return (head_ >= tail_) ? (head_ - tail_)
                            : (head_ + BUFFER_SIZE - tail_);
  }

  bool empty() const { return head_ == tail_; }

  uint8_t peek(size_t offset) const
  {
    return buf_[(tail_ + offset) % BUFFER_SIZE];
  }

  void consume(size_t n)
  {
    tail_ = (tail_ + n) % BUFFER_SIZE;
  }

  std::array<uint8_t, BUFFER_SIZE> buf_{};
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t dropped_bytes_ = 0;
  size_t bad_frame_count_ = 0;
};

// ──────────── inline implementations ───────────────────────────────

inline bool Stm32StreamParser::append(const uint8_t * data, size_t len)
{
  bool overflow = false;
  for (size_t i = 0; i < len; ++i) {
    const size_t next = (head_ + 1U) % BUFFER_SIZE;
    if (next == tail_) {
      // Ring full: drop the oldest byte, record overflow, keep parsing.
      tail_ = (tail_ + 1U) % BUFFER_SIZE;
      ++dropped_bytes_;
      overflow = true;
    }
    buf_[head_] = data[i];
    head_ = next;
  }
  return !overflow;
}

inline std::vector<Stm32StreamParser::Frame> Stm32StreamParser::poll()
{
  std::vector<Frame> out;

  // Each loop iteration either consumes at least one byte or returns;
  // capped like the firmware's loop guard so pathological input cannot
  // starve the caller.
  size_t iterations = 0;
  const size_t loop_guard = BUFFER_SIZE * 2U;

  while (!empty() && iterations++ < loop_guard) {
    if (peek(0) != Stm32Protocol::SOF0) {
      consume(1);  // noise byte
      continue;
    }
    if (avail() < 2U) return out;  // only AA so far — wait for more
    if (peek(1) != Stm32Protocol::SOF1) {
      consume(1);  // AA not followed by 55 — skip the AA and resync
      continue;
    }
    if (avail() < Stm32Protocol::HEADER_SIZE) return out;

    const uint16_t len = static_cast<uint16_t>(peek(6)) |
                         (static_cast<uint16_t>(peek(7)) << 8);
    if (len > Stm32Protocol::MAX_PAYLOAD) {
      // Malicious / corrupt LEN: skip the frame-header byte and resync.
      ++bad_frame_count_;
      consume(1);
      continue;
    }
    const uint16_t total = Stm32Protocol::total_len(len);
    if (avail() < total) return out;  // partial frame — wait for more

    // Linear copy of the whole frame so CRC can be computed safely even
    // when the ring wraps around.
    std::vector<uint8_t> raw(total);
    for (uint16_t i = 0; i < total; ++i) {
      raw[i] = peek(i);
    }

    Frame f;
    f.cmd = raw[3];
    f.seq = static_cast<uint16_t>(raw[4]) |
            (static_cast<uint16_t>(raw[5]) << 8);
    f.payload.assign(raw.begin() + 8, raw.begin() + 8 + len);

    const uint16_t got_crc = static_cast<uint16_t>(raw[8U + len]) |
                             (static_cast<uint16_t>(raw[9U + len]) << 8);
    // CRC covers VERSION..PAYLOAD end = 6+LEN bytes starting at byte 2.
    const uint16_t want_crc = Stm32Protocol::crc16(&raw[2], 6U + len);

    consume(total);  // consumed whether valid or not — no replay

    const bool version_ok = (raw[2] == Stm32Protocol::VERSION);
    f.ok = version_ok && (got_crc == want_crc);
    if (!f.ok) {
      ++bad_frame_count_;
    }
    out.push_back(f);
  }

  return out;
}

inline void Stm32StreamParser::reset()
{
  head_ = 0;
  tail_ = 0;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__STM32_STREAM_PARSER_HPP_
