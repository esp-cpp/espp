#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "hid-rp-3dconnexion.hpp"

/// Decodes the Input reports of a 3Dconnexion SpaceMouse (SpaceNavigator,
/// SpaceMouse Compact / Wireless / Pro, ...) into a 6-DoF state.
///
/// The devices split their sensor across three report ids: 1 = translation
/// (X, Y, Z), 2 = rotation (Rx, Ry, Rz), 3 = buttons. The espp::SpaceMouse*
/// report classes in hid-rp carry the exact wire layouts; this class routes a
/// raw report to the right one by its id byte and keeps the latest of each.
///
/// Some newer SpaceMouse firmware sends translation + rotation together in one
/// 12-byte report id 1; that layout is recognised too.
class SpaceMouseDecoder {
public:
  /// 3Dconnexion's USB vendor id (every current SpaceMouse).
  static constexpr uint16_t kVendorId3Dconnexion = 0x256F;
  /// The early SpaceNavigator / SpaceExplorer / SpacePilot were sold under
  /// Logitech's vendor id, which Logitech keyboards and mice share, so those are
  /// matched by product id.
  static constexpr uint16_t kVendorIdLogitech = 0x046D;
  static constexpr uint16_t kLogitechSpaceMousePids[] = {0xC626, 0xC627, 0xC628, 0xC629,
                                                         0xC62B, 0xC623, 0xC625, 0xC603};

  /// The number of buttons the decoder tracks (enough for a SpaceMouse Pro's
  /// 15 and the two-button Compact / Navigator).
  static constexpr size_t kButtonCount = 16;

  /// Axis range of the raw reports (SpaceNavigator: +-350).
  static constexpr int16_t kAxisMax = 350;

  struct State {
    int16_t x{0};  ///< translation, raw counts (+ = right)
    int16_t y{0};  ///< translation, raw counts (+ = toward the user)
    int16_t z{0};  ///< translation, raw counts (+ = down)
    int16_t rx{0}; ///< rotation about X (pitch), raw counts
    int16_t ry{0}; ///< rotation about Y (roll), raw counts
    int16_t rz{0}; ///< rotation about Z (yaw), raw counts
    std::array<bool, kButtonCount> buttons{};
    uint32_t translation_reports{0};
    uint32_t rotation_reports{0};
    uint32_t button_reports{0};
    uint32_t unknown_reports{0};
  };

  /// Whether a VID:PID is a SpaceMouse: any 3Dconnexion device, or one of the
  /// Logitech-branded SpaceNavigator / SpaceExplorer / SpacePilot ids.
  static bool is_spacemouse(uint16_t vid, uint16_t pid) {
    if (vid == kVendorId3Dconnexion)
      return true;
    if (vid != kVendorIdLogitech)
      return false;
    for (uint16_t p : kLogitechSpaceMousePids)
      if (p == pid)
        return true;
    return false;
  }

  /// Feed one raw Input report (report id in byte 0). \return true if the
  /// report was recognised and the state updated.
  bool decode(std::span<const uint8_t> report);

  const State &state() const { return state_; }
  void reset() { state_ = State{}; }

protected:
  State state_;
  std::vector<uint8_t> scratch_; ///< reused per report (no per-report allocation)
  espp::SpaceMouseTranslationInputReport<> translation_;
  espp::SpaceMouseRotationInputReport<> rotation_;
  espp::SpaceMouseButtonsInputReport<kButtonCount> buttons_;
};
