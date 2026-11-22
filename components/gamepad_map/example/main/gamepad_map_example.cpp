#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "file_system.hpp"
#include "gamepad_map.hpp"
#include "gamepad_profiles.hpp"
#include "gamepad_state_formatters.hpp"
#include "gamepad_state_hid_rp.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

// The HID report map of a BLE gamepad (a ShanWan BSP-D8 in its Android mode),
// as read from its Report Map characteristic: report 3 = consumer keys,
// report 4 = 8-bit X/Y/Z/Rz sticks, a hat, 16 buttons, Brake / Accelerator.
static const char *kReportMapHex =
    "050c0901a1018503751095011501269c0219012a9c028100c005010905a10185040901a1000930093109320935"
    "150026ff00750895048102c00939150025073500463b016514750495018142750495018101050919012910150025"
    "017501951081020502150026ff0009c409c5950275088102750895018101c0";

static std::vector<uint8_t> from_hex(const char *hex) {
  std::vector<uint8_t> out;
  for (; hex[0] && hex[1]; hex += 2)
    out.push_back(static_cast<uint8_t>(std::stoi(std::string(hex, 2), nullptr, 16)));
  return out;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "gamepad map example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting gamepad map example!");

  int passed = 0;
  int failed = 0;
  auto check = [&](const char *name, bool ok) {
    fmt::print("  [{}] {}\n", ok ? "PASS" : "FAIL", name);
    (ok ? passed : failed)++;
  };

  using B = espp::GamepadState::Button;
  using A = espp::GamepadState::Axis;

  {
    logger.info("Decode a BLE gamepad report, then remap it");
    //! [gamepad map example]
    // At connect: parse the controller's report map once and keep a decoder.
    // A bridge decodes every control as only itself and lets the map decide.
    auto report_map = espp::hid_rp::ReportMap::parse(from_hex(kReportMapHex));
    if (!report_map || !espp::hid_rp::GamepadDecoder::looks_like_gamepad(*report_map)) {
      logger.error("Not a gamepad report map");
      return;
    }
    espp::hid_rp::GamepadDecoder decoder(std::move(*report_map));
    decoder.set_combine_controls(false);

    // The map: Nintendo-style A/B (south <-> east), the left stick also on the
    // d-pad, analog triggers also as ZL / ZR, a 10% left-stick dead zone.
    espp::GamepadMap map;
    map.swap(B::South, B::East).left_stick_to_dpad().triggers_to_buttons();
    map.left_deadzone_percent = 10;
    std::string why;
    if (!map.valid(&why))
      logger.error("Invalid map: {}", why);

    // For every report: decode (BLE delivers the report ID out of band), map,
    // and hand the result on. Here: the south button (A), left stick fully
    // left, right trigger fully pressed.
    const std::vector<uint8_t> payload{0x00, 0x80, 0x80, 0x80, 0x0f, 0x01, 0x00, 0xff, 0x00, 0x00};
    espp::hid_rp::GamepadReport report;
    if (decoder.update(4, payload, report)) {
      const espp::GamepadState in = espp::to_gamepad_state(report);
      const espp::GamepadState out = map.apply(in);
      logger.info("controller: {}", in);
      logger.info("mapped:     {}", out);
    }
    //! [gamepad map example]
    espp::hid_rp::GamepadReport r;
    decoder.update(4, payload, r);
    const auto out = map.apply(espp::to_gamepad_state(r));
    check("south is mapped to east", out.pressed(B::East) && !out.pressed(B::South));
    check("the left stick also presses d-pad left", out.pressed(B::Left));
    check("the stick is kept", out.axis(A::LeftX) < -30000);
    check("the right trigger also presses R2", out.pressed(B::R2));
  }

  {
    logger.info("Per-controller profiles in a file");
    auto &fs = espp::FileSystem::get();
    const std::filesystem::path path = fs.get_root_path() / "gamepad_profiles.bin";
    std::error_code ec;
    std::filesystem::remove(path, ec); // start clean for the checks below
    //! [gamepad profiles example]
    // The default map is used for controllers without a profile; the file is
    // written on the first change.
    espp::GamepadMap default_map;
    default_map.invert(A::LeftY).invert(A::RightY); // up-positive sticks for this host
    espp::GamepadProfiles profiles({
        .file_path = path,
        .default_map = default_map,
        .log_level = espp::Logger::Verbosity::INFO,
    });

    // A profile for one controller type (vendor / product ID; from the BLE
    // Device Information PnP ID, or the USB device descriptor).
    espp::GamepadMap bsp_d8 = default_map;
    bsp_d8.swap(B::South, B::East);
    if (!profiles.set_profile({.vid = 0x1949, .pid = 0x0402, .name = "BSP-D8", .map = bsp_d8}, ec))
      logger.error("Could not save the profile: {}", ec.message());

    // When a controller connects, look its map up once and keep it.
    const espp::GamepadMap active = profiles.map_for(0x1949, 0x0402, "BSP-D8");
    logger.info("BSP-D8 map: {} rule(s); default map: {} rule(s)", active.rules.size(),
                profiles.default_map().rules.size());
    //! [gamepad profiles example]
    check("the profile is used for its controller", active == bsp_d8);
    check("other controllers get the default map", profiles.map_for(0x045e, 0x0b13) == default_map);
    espp::GamepadProfiles reloaded({.file_path = path});
    check("profiles survive a reload",
          reloaded.map_for(0x1949, 0x0402) == bsp_d8 && reloaded.default_map() == default_map);
    check("erase a profile", reloaded.erase_profile(0x1949, 0x0402, "", ec) && !ec &&
                                 reloaded.map_for(0x1949, 0x0402) == default_map);
  }

  logger.info("{} passed, {} failed", passed, failed);
  logger.info("Gamepad map example complete!");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
