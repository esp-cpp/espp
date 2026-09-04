#include <chrono>
#include <string>
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
  // Register each module WITH discovery metadata (name / web app / description)
  // so a connected peer can enumerate them (see the discovery section below).
  dispatcher.register_module(
      kModuleControl,
      [&](const sf::Frame &f) {
        logger.info("[control] {} type=0x{:02X} ({} payload bytes)",
                    f.is_reply() ? "reply" : "request", f.type, f.payload.size());
      },
      {.name = "Control", .app = "control_console.html", .description = "Device control channel"});
  dispatcher.register_module(kModuleTelemetry,
                             [&](const sf::Frame &f) {
                               uint32_t value = f.payload.size() == 4 ? sf::get_u32(f.payload) : 0;
                               logger.info("[telemetry] type=0x{:02X} value={}", f.type, value);
                             },
                             {.name = "Telemetry",
                              .app = "telemetry_console.html",
                              .description = "Live telemetry stream"});

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

  // --- capability discovery -------------------------------------------------
  // A connected peer (e.g. the browser hub app) can ask WHICH modules this
  // device runs, over the reserved discovery module id 0xFF. Advertise a device
  // name + firmware, then opt in to auto-answering the query. serve_discovery()
  // is the ONLY path by which the Dispatcher sends: it hands the encoded reply
  // frame to the transmit callback we supply. On real hardware that callback is
  // usb.write_vendor / socket send / etc.; here we just capture it in-process
  // and decode it to show what a peer receives.
  dispatcher.set_device_info("espp Dispatcher Example", "1.0.0");
  std::vector<uint8_t> discovery_reply;
  dispatcher.serve_discovery(
      [&](std::span<const uint8_t> frame) { discovery_reply.assign(frame.begin(), frame.end()); });

  dispatcher.feed(sf::build_frame(/*reply=*/false, espp::Dispatcher::kDiscoveryModule,
                                  static_cast<uint8_t>(espp::Dispatcher::Discovery::ListModules)));

  const auto reply_frames = sf::StreamParser{}.feed(discovery_reply);
  if (!reply_frames.empty()) {
    const auto &p = reply_frames[0].payload;
    size_t i = 2; // skip [version][reserved]
    auto rd_str = [&]() {
      const uint8_t n = p[i++];
      std::string s(reinterpret_cast<const char *>(&p[i]), n);
      i += n;
      return s;
    };
    const std::string dev = rd_str();
    const std::string fw = rd_str();
    const uint8_t count = p[i++];
    logger.info("discovery: '{}' (fw {}) advertises {} module(s):", dev, fw, count);
    for (uint8_t m = 0; m < count; ++m) {
      const uint8_t id = p[i++];
      const std::string name = rd_str();
      const std::string app = rd_str();
      const std::string desc = rd_str();
      logger.info("  module {}: {} [app={}] — {}", id, name, app, desc);
    }
  }
  //! [dispatcher example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
