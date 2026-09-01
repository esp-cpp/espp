// Host-buildable unit tests for the espp stream_frame codec (v2). Build & run:
//   c++ -std=c++20 -Werror -I components/stream_frame/include \
//       components/stream_frame/test/stream_frame_host_test.cpp -o test && ./test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string_view>
#include <vector>

#include "stream_frame.hpp"

namespace sf = espp::stream_frame;

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
  CHECK(sf::crc32(as_bytes("123456789")) == 0xCBF43926u); // standard check value
  CHECK(sf::crc32({}) == 0u);
  CHECK(sf::crc32(as_bytes("abc")) == 0x352441C2u);
  const uint32_t first = sf::crc32(as_bytes("12345"));
  CHECK(sf::crc32(as_bytes("6789"), first) == 0xCBF43926u); // chaining
}

static void test_flags() {
  std::printf("test_flags\n");
  const uint8_t req = sf::make_flags(false);
  const uint8_t rep = sf::make_flags(true);
  CHECK(!sf::flags_is_reply(req) && sf::flags_is_reply(rep));
  CHECK(sf::flags_version(req) == sf::kVersion && sf::flags_version(rep) == sf::kVersion);
}

static void test_frame_layout() {
  std::printf("test_frame_layout\n");
  // A reply frame: module 4, type 7, payload {0xAA,0xBB}.
  const uint8_t payload[] = {0xAA, 0xBB};
  const auto frame = sf::build_frame(/*reply=*/true, /*module=*/4, /*type=*/7, payload);
  CHECK(frame.size() == sf::kHeaderSize + sizeof(payload) + sf::kCrcSize);
  CHECK(frame[0] == 0x54 && frame[1] == 0x4F); // magic LE ("OT")
  CHECK(frame[2] == sf::make_flags(true));     // flags (version<<4 | reply)
  CHECK(frame[3] == 4);                        // module
  CHECK(frame[4] == 7);                        // type
  CHECK(frame[5] == 0x02 && frame[6] == 0 && frame[7] == 0 && frame[8] == 0); // len u32 LE = 2
  CHECK(frame[9] == 0xAA && frame[10] == 0xBB);                               // payload
  const uint32_t crc = sf::crc32(std::span<const uint8_t>(frame.data(), sf::kHeaderSize + 2));
  CHECK(sf::get_u32(std::span<const uint8_t>(frame).subspan(sf::kHeaderSize + 2)) == crc);
}

static void test_round_trip() {
  std::printf("test_round_trip\n");
  sf::StreamParser parser;
  std::vector<uint8_t> stream;
  auto append = [&](const std::vector<uint8_t> &f) {
    stream.insert(stream.end(), f.begin(), f.end());
  };
  const uint8_t data[] = {1, 2, 3, 4, 5};
  append(sf::build_frame(false, 0, 0x02, data)); // module 0, request
  append(sf::build_frame(true, 4, 0xC2));        // module 4, reply, empty payload
  append(sf::build_frame(false, 5, 0x50, data)); // module 5, request

  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 3);
  CHECK(parser.buffered() == 0 && parser.dropped_bytes() == 0);
  if (frames.size() != 3)
    return;
  CHECK(frames[0].module == 0 && frames[0].type == 0x02 && !frames[0].is_reply());
  CHECK(frames[0].payload.size() == sizeof(data));
  CHECK(std::memcmp(frames[0].payload.data(), data, sizeof(data)) == 0);
  CHECK(frames[1].module == 4 && frames[1].type == 0xC2 && frames[1].is_reply());
  CHECK(frames[1].payload.empty());
  CHECK(frames[2].module == 5 && frames[2].type == 0x50 && !frames[2].is_reply());
  CHECK(frames[0].version() == sf::kVersion);
}

static void test_split_across_chunks() {
  std::printf("test_split_across_chunks\n");
  const uint8_t data[] = {9, 8, 7};
  auto stream = sf::build_frame(false, 2, 0x10, data);
  const auto ok = sf::build_frame(true, 2, 0x11);
  stream.insert(stream.end(), ok.begin(), ok.end());
  sf::StreamParser parser;
  std::vector<sf::Frame> frames;
  for (const uint8_t byte : stream) { // one byte at a time
    auto out = parser.feed(std::span<const uint8_t>(&byte, 1));
    frames.insert(frames.end(), out.begin(), out.end());
  }
  CHECK(frames.size() == 2 && parser.dropped_bytes() == 0);
  if (frames.size() == 2) {
    CHECK(frames[0].type == 0x10 && frames[0].payload.size() == sizeof(data));
    CHECK(frames[1].type == 0x11 && frames[1].is_reply());
  }
}

static void test_resync_on_corruption() {
  std::printf("test_resync_on_corruption\n");
  const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
  std::vector<uint8_t> stream = {0x00, 0xFF, 0x42, 0x54 /* lone 'T' */};
  auto corrupted = sf::build_frame(false, 1, 0x02, payload);
  corrupted[sf::kHeaderSize] ^= 0xFF; // flip a payload byte -> CRC mismatch
  stream.insert(stream.end(), corrupted.begin(), corrupted.end());
  const auto good = sf::build_frame(true, 1, 0x05);
  stream.insert(stream.end(), good.begin(), good.end());
  sf::StreamParser parser;
  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 1);
  if (frames.size() == 1)
    CHECK(frames[0].type == 0x05 && frames[0].is_reply());
  CHECK(parser.dropped_bytes() > 0 && parser.buffered() == 0);
}

static void test_oversized_len_rejected() {
  std::printf("test_oversized_len_rejected\n");
  std::vector<uint8_t> bogus;
  sf::put_u16(bogus, sf::kMagic);
  bogus.push_back(sf::make_flags(false)); // flags
  bogus.push_back(0);                     // module
  bogus.push_back(0);                     // type
  sf::put_u32(bogus, static_cast<uint32_t>(sf::kMaxPayloadSize + 1));
  sf::put_u32(bogus, sf::crc32(std::span<const uint8_t>(bogus.data(), bogus.size())));
  std::vector<uint8_t> stream = bogus;
  const auto good = sf::build_frame(false, 0, 0x03);
  stream.insert(stream.end(), good.begin(), good.end());
  sf::StreamParser parser;
  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 1);
  if (frames.size() == 1)
    CHECK(frames[0].type == 0x03);
  CHECK(parser.dropped_bytes() > 0 && parser.buffered() < sf::kMaxFrameSize);
  // builder refuses an oversized payload
  const std::vector<uint8_t> too_big(sf::kMaxPayloadSize + 1, 0xAB);
  CHECK(sf::build_frame(false, 0, 0x02, too_big).empty());
  // a maximum-size frame round-trips
  const std::vector<uint8_t> max_size(sf::kMaxPayloadSize, 0xCD);
  const auto max_frame = sf::build_frame(false, 0, 0x02, max_size);
  CHECK(max_frame.size() == sf::kHeaderSize + sf::kMaxPayloadSize + sf::kCrcSize);
  CHECK(max_frame.size() <= sf::kMaxFrameSize);
  sf::StreamParser p2;
  const auto mf = p2.feed(max_frame);
  CHECK(mf.size() == 1 && (mf.empty() || mf[0].payload == max_size));
}

static void test_optional_correlation() {
  std::printf("test_optional_correlation\n");
  // Without correlation: byte-identical to a plain frame, no correlation field.
  const uint8_t p[] = {1, 2, 3};
  const auto plain = sf::build_frame(false, 4, 0x10, p);
  const auto plain_expected = sf::build_frame(false, 4, 0x10, p, std::nullopt);
  CHECK(plain == plain_expected);
  CHECK(plain.size() == sf::kHeaderSize + sizeof(p) + sf::kCrcSize);

  // With correlation: header grows by kCorrelationSize; the id round-trips.
  const auto withc = sf::build_frame(true, 4, 0xC0, p, uint16_t{0xBEEF});
  CHECK(withc.size() == sf::kHeaderSize + sf::kCorrelationSize + sizeof(p) + sf::kCrcSize);
  CHECK(withc[2] == (sf::make_flags(true) | sf::kFlagCorrelation)); // correlation flag set
  CHECK(withc[5] == 0xEF && withc[6] == 0xBE); // correlation u16 LE, after type

  sf::StreamParser parser;
  // Interleave a plain frame and a correlated one, split arbitrarily.
  std::vector<uint8_t> stream = plain;
  stream.insert(stream.end(), withc.begin(), withc.end());
  std::vector<sf::Frame> frames;
  for (const uint8_t b : stream) {
    auto out = parser.feed(std::span<const uint8_t>(&b, 1));
    frames.insert(frames.end(), out.begin(), out.end());
  }
  CHECK(frames.size() == 2 && parser.dropped_bytes() == 0);
  if (frames.size() == 2) {
    CHECK(!frames[0].has_correlation() && !frames[0].correlation.has_value());
    CHECK(frames[1].has_correlation() && frames[1].correlation.value_or(0) == 0xBEEF);
    CHECK(frames[1].module == 4 && frames[1].type == 0xC0 && frames[1].is_reply());
    CHECK(frames[1].payload.size() == sizeof(p));
  }
}

int main() {
  test_crc32();
  test_flags();
  test_frame_layout();
  test_round_trip();
  test_optional_correlation();
  test_split_across_chunks();
  test_resync_on_corruption();
  test_oversized_len_rejected();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
