#include <chrono>
#include <thread>
#include <vector>

#include "dispatcher.hpp"
#include "logger.hpp"
#include "stream_frame.hpp"

using namespace std::chrono_literals;

// Demonstrates multiplexing several framed protocols over ONE byte stream with
// espp::Dispatcher: build frames with the espp::stream_frame codec, then route
// them to per-module handlers. No hardware needed — the "stream" here is an
// in-memory buffer, but the exact same feed()/register_module() pattern applies
// to a USB vendor / CDC / socket / UART receive path.
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Dispatcher Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting dispatcher example");

  //! [dispatcher example]
  namespace sf = espp::stream_frame;

  // Two toy protocols sharing one stream: "control" on module 0 and "telemetry"
  // on module 4 (module is a full byte, so up to 256 protocols can coexist).
  static constexpr uint8_t kModuleControl = 0;
  static constexpr uint8_t kModuleTelemetry = 4;

  espp::Dispatcher dispatcher;
  dispatcher.register_module(kModuleControl, [&](const sf::Frame &f) {
    logger.info("[control] {} type=0x{:02X} ({} payload bytes)", f.is_reply() ? "reply" : "request",
                f.type, f.payload.size());
  });
  dispatcher.register_module(kModuleTelemetry, [&](const sf::Frame &f) {
    uint32_t value = f.payload.size() == 4 ? sf::get_u32(f.payload) : 0;
    logger.info("[telemetry] type=0x{:02X} value={}", f.type, value);
  });

  // Build a mixed stream, as a peer would send it.
  std::vector<uint8_t> telemetry_payload;
  sf::put_u32(telemetry_payload, 42);
  std::vector<uint8_t> stream;
  auto append = [&](const std::vector<uint8_t> &frame) {
    stream.insert(stream.end(), frame.begin(), frame.end());
  };
  const uint8_t start[] = {'s', 't', 'a', 'r', 't'};
  append(sf::build_frame(/*reply=*/false, kModuleControl, 0x01, start));
  append(sf::build_frame(/*reply=*/false, kModuleTelemetry, 0x01, telemetry_payload));
  // A frame for an unregistered module is silently ignored.
  append(sf::build_frame(/*reply=*/false, 9, 0x00));
  append(sf::build_frame(/*reply=*/true, kModuleControl, 0x81)); // a reply/event

  // Feed it in two arbitrary chunks to show the parser reassembles split frames.
  const size_t half = stream.size() / 2;
  dispatcher.feed(std::span<const uint8_t>(stream.data(), half));
  dispatcher.feed(std::span<const uint8_t>(stream.data() + half, stream.size() - half));
  logger.info("done ({} bytes dropped while resyncing)", dispatcher.dropped_bytes());
  //! [dispatcher example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
