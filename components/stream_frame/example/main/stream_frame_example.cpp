#include <chrono>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "stream_frame.hpp"

using namespace std::chrono_literals;

// Minimal demonstration of the espp::stream_frame codec: build a couple of
// frames, feed them (split across chunks) to a StreamParser, and log the
// decoded module / type / reply / payload. No hardware needed.
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "stream_frame Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting stream_frame example");

  //! [stream_frame example]
  namespace sf = espp::stream_frame;

  // Build a request (module 4, type 0x42, a u32 payload) and a reply.
  std::vector<uint8_t> payload;
  sf::put_u32(payload, 0xDEADBEEF);
  const auto request = sf::build_frame(/*reply=*/false, /*module=*/4, /*type=*/0x42, payload);
  const auto reply = sf::build_frame(/*reply=*/true, /*module=*/4, /*type=*/0xC2);

  // Concatenate and feed the parser one byte at a time to show it reassembles
  // frames split across reads.
  std::vector<uint8_t> stream = request;
  stream.insert(stream.end(), reply.begin(), reply.end());

  sf::StreamParser parser;
  for (const uint8_t byte : stream) {
    for (const auto &frame : parser.feed(std::span<const uint8_t>(&byte, 1))) {
      logger.info("frame: module={} type=0x{:02X} reply={} payload={} bytes", frame.module,
                  frame.type, frame.is_reply(), frame.payload.size());
    }
  }
  logger.info(
      "crc32(\"123456789\") = 0x{:08X} (expect 0xCBF43926); dropped {} bytes",
      sf::crc32(std::span<const uint8_t>(reinterpret_cast<const uint8_t *>("123456789"), 9)),
      parser.dropped_bytes());
  //! [stream_frame example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
