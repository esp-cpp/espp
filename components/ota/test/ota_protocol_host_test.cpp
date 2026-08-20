// Host-buildable unit tests for the espp OTA stream protocol framing. Build &
// run with:
//   c++ -std=c++20 -Werror -I components/ota/include \
//       components/ota/test/ota_protocol_host_test.cpp -o test && ./test
//
// These tests exercise espp::detail::ota_stream directly so they need no
// ESP-IDF headers.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "detail/ota_stream_protocol.hpp"

namespace ota = espp::detail::ota_stream;
using ota::Frame;
using ota::MessageType;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static std::span<const uint8_t> as_bytes(std::string_view s) {
  return {reinterpret_cast<const uint8_t *>(s.data()), s.size()};
}

static void test_crc32() {
  std::printf("test_crc32\n");
  // The standard CRC-32 check value (zlib / IEEE 802.3).
  CHECK(ota::crc32(as_bytes("123456789")) == 0xCBF43926u);
  // Empty input yields the zlib initial value 0.
  CHECK(ota::crc32({}) == 0u);
  // A couple more golden vectors (values from zlib's crc32()).
  CHECK(ota::crc32(as_bytes("a")) == 0xE8B7BE43u);
  CHECK(ota::crc32(as_bytes("abc")) == 0x352441C2u);
  const uint8_t zeros[4] = {0, 0, 0, 0};
  CHECK(ota::crc32(zeros) == 0x2144DF1Cu);
  // Chaining across chunks matches the one-shot result.
  const uint32_t first = ota::crc32(as_bytes("12345"));
  CHECK(ota::crc32(as_bytes("6789"), first) == 0xCBF43926u);
}

static void test_frame_layout() {
  std::printf("test_frame_layout\n");
  // BEGIN with image_size 0x11223344; check the exact encoded bytes.
  const auto frame = ota::make_begin(0x11223344u);
  CHECK(frame.size() == ota::kHeaderSize + 4 + ota::kCrcSize);
  CHECK(frame[0] == 0x54); // 'T' — low byte of the LE magic 0x4F54
  CHECK(frame[1] == 0x4F); // 'O' — high byte
  CHECK(frame[2] == 0x01); // type BEGIN
  // len u32 LE = 4
  CHECK(frame[3] == 0x04 && frame[4] == 0x00 && frame[5] == 0x00 && frame[6] == 0x00);
  // payload u32 LE = image_size
  CHECK(frame[7] == 0x44 && frame[8] == 0x33 && frame[9] == 0x22 && frame[10] == 0x11);
  // trailing crc32 (LE) over magic..payload
  const uint32_t crc = ota::crc32(std::span<const uint8_t>(frame.data(), 11));
  CHECK(ota::get_u32(std::span<const uint8_t>(frame).subspan(11)) == crc);
}

static void test_round_trip_all_types() {
  std::printf("test_round_trip_all_types\n");
  ota::StreamParser parser;
  const uint8_t image_bytes[] = {0xE9, 0x06, 0x02, 0x2F, 0xAA, 0x55};

  std::vector<uint8_t> stream;
  auto append = [&stream](const std::vector<uint8_t> &f) {
    stream.insert(stream.end(), f.begin(), f.end());
  };
  append(ota::make_begin(1234567u));
  append(ota::make_data(image_bytes));
  append(ota::make_finish());
  append(ota::make_abort());
  append(ota::make_ok(6u));
  append(ota::make_error(static_cast<uint32_t>(5), "flash write failed"));
  append(ota::make_progress(4096u, 8192u));

  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 7);
  CHECK(parser.buffered() == 0);
  CHECK(parser.dropped_bytes() == 0);
  if (frames.size() != 7)
    return;

  CHECK(frames[0].type == MessageType::Begin);
  CHECK(ota::parse_u32_payload(frames[0]).value_or(0) == 1234567u);

  CHECK(frames[1].type == MessageType::Data);
  CHECK(frames[1].payload.size() == sizeof(image_bytes));
  CHECK(std::memcmp(frames[1].payload.data(), image_bytes, sizeof(image_bytes)) == 0);

  CHECK(frames[2].type == MessageType::Finish);
  CHECK(frames[2].payload.empty());

  CHECK(frames[3].type == MessageType::Abort);
  CHECK(frames[3].payload.empty());

  CHECK(frames[4].type == MessageType::Ok);
  CHECK(ota::parse_u32_payload(frames[4]).value_or(0) == 6u);

  CHECK(frames[5].type == MessageType::Error);
  const auto err = ota::parse_error(frames[5]);
  CHECK(err.has_value());
  if (err.has_value()) {
    CHECK(err->code == 5u);
    CHECK(err->message == "flash write failed");
  }

  CHECK(frames[6].type == MessageType::Progress);
  const auto prog = ota::parse_progress(frames[6]);
  CHECK(prog.has_value());
  if (prog.has_value()) {
    CHECK(prog->written == 4096u);
    CHECK(prog->total == 8192u);
  }
}

static void test_split_across_chunks() {
  std::printf("test_split_across_chunks\n");
  const uint8_t data_bytes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto stream = ota::make_data(data_bytes);
  const auto ok = ota::make_ok(10u);
  stream.insert(stream.end(), ok.begin(), ok.end());

  // Deliver one byte at a time; both frames must still come out, in order.
  ota::StreamParser parser;
  std::vector<Frame> frames;
  for (const uint8_t byte : stream) {
    auto out = parser.feed(std::span<const uint8_t>(&byte, 1));
    frames.insert(frames.end(), out.begin(), out.end());
  }
  CHECK(frames.size() == 2);
  CHECK(parser.dropped_bytes() == 0);
  if (frames.size() == 2) {
    CHECK(frames[0].type == MessageType::Data);
    CHECK(frames[0].payload.size() == sizeof(data_bytes));
    CHECK(frames[1].type == MessageType::Ok);
    CHECK(ota::parse_u32_payload(frames[1]).value_or(0) == 10u);
  }

  // Also deliver in two awkward halves that split the magic itself.
  ota::StreamParser parser2;
  auto first = parser2.feed(std::span<const uint8_t>(stream.data(), 1)); // just 'T'
  CHECK(first.empty());
  auto rest = parser2.feed(std::span<const uint8_t>(stream.data() + 1, stream.size() - 1));
  CHECK(rest.size() == 2);
}

static void test_resync_on_corruption() {
  std::printf("test_resync_on_corruption\n");
  const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};

  // Leading garbage, then a corrupted frame (bad CRC), then a good frame.
  std::vector<uint8_t> stream = {0x00, 0xFF, 0x42, 0x54 /* lone 'T' not followed by 'O' */};
  auto corrupted = ota::make_data(payload);
  corrupted[ota::kHeaderSize] ^= 0xFF; // flip a payload byte -> CRC mismatch
  stream.insert(stream.end(), corrupted.begin(), corrupted.end());
  const auto good = ota::make_ok(4u);
  stream.insert(stream.end(), good.begin(), good.end());

  ota::StreamParser parser;
  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 1);
  if (frames.size() == 1) {
    CHECK(frames[0].type == MessageType::Ok);
    CHECK(ota::parse_u32_payload(frames[0]).value_or(0) == 4u);
  }
  CHECK(parser.dropped_bytes() > 0);
  CHECK(parser.buffered() == 0);
}

static void test_oversized_len_rejected() {
  std::printf("test_oversized_len_rejected\n");
  // Hand-craft a frame whose length field exceeds kMaxPayloadSize (with a
  // valid CRC, so only the length cap can reject it), followed by a good frame.
  std::vector<uint8_t> bogus;
  ota::put_u16(bogus, ota::kMagic);
  bogus.push_back(static_cast<uint8_t>(MessageType::Data));
  ota::put_u32(bogus, static_cast<uint32_t>(ota::kMaxPayloadSize + 1));
  ota::put_u32(bogus, ota::crc32(std::span<const uint8_t>(bogus.data(), bogus.size())));

  std::vector<uint8_t> stream = bogus;
  const auto good = ota::make_finish();
  stream.insert(stream.end(), good.begin(), good.end());

  ota::StreamParser parser;
  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 1);
  if (frames.size() == 1)
    CHECK(frames[0].type == MessageType::Finish);
  CHECK(parser.dropped_bytes() > 0);
  // The oversized length must never be buffered/waited for.
  CHECK(parser.buffered() < ota::kMaxFrameSize);

  // The builder refuses to build an oversized frame outright.
  const std::vector<uint8_t> too_big(ota::kMaxPayloadSize + 1, 0xAB);
  CHECK(ota::make_data(too_big).empty());
  // ...but a maximum-size frame is fine and round-trips.
  const std::vector<uint8_t> max_size(ota::kMaxPayloadSize, 0xCD);
  const auto max_frame = ota::make_data(max_size);
  CHECK(max_frame.size() == ota::kMaxFrameSize);
  ota::StreamParser parser2;
  const auto max_frames = parser2.feed(max_frame);
  CHECK(max_frames.size() == 1);
  if (max_frames.size() == 1)
    CHECK(max_frames[0].payload == max_size);
}

static void test_malformed_reply_payloads() {
  std::printf("test_malformed_reply_payloads\n");
  // Wrong-size payloads must be rejected by the typed parse helpers.
  Frame f{MessageType::Ok, {0x01, 0x02}};
  CHECK(!ota::parse_u32_payload(f).has_value());
  Frame e{MessageType::Error, {0x01, 0x02, 0x03}};
  CHECK(!ota::parse_error(e).has_value());
  Frame p{MessageType::Progress, {0x01, 0x02, 0x03, 0x04}};
  CHECK(!ota::parse_progress(p).has_value());
  // An ERROR with just a code (no message) is valid.
  Frame e2{MessageType::Error, {0x05, 0x00, 0x00, 0x00}};
  const auto info = ota::parse_error(e2);
  CHECK(info.has_value());
  if (info.has_value()) {
    CHECK(info->code == 5u);
    CHECK(info->message.empty());
  }
}

int main() {
  test_crc32();
  test_frame_layout();
  test_round_trip_all_types();
  test_split_across_chunks();
  test_resync_on_corruption();
  test_oversized_len_rejected();
  test_malformed_reply_payloads();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
