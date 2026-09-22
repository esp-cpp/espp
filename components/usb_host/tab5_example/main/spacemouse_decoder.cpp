#include "spacemouse_decoder.hpp"

#include <cstring>

namespace {
constexpr uint8_t kTranslationReportId = 1;
constexpr uint8_t kRotationReportId = 2;
constexpr uint8_t kButtonsReportId = 3;
constexpr size_t kAxesBytes = 3 * sizeof(int16_t);

int16_t le16(std::span<const uint8_t> b, size_t offset) {
  return static_cast<int16_t>(static_cast<uint16_t>(b[offset]) |
                              (static_cast<uint16_t>(b[offset + 1]) << 8));
}
} // namespace

bool SpaceMouseDecoder::decode(std::span<const uint8_t> report) {
  if (report.empty()) {
    ++state_.unknown_reports;
    return false;
  }
  const uint8_t report_id = report[0];
  const auto payload = report.subspan(1);
  switch (report_id) {
  case kTranslationReportId: {
    if (payload.size() < kAxesBytes)
      break;
    translation_.set_data(std::vector<uint8_t>(payload.begin(), payload.begin() + kAxesBytes));
    translation_.get_translation(state_.x, state_.y, state_.z);
    ++state_.translation_reports;
    // newer firmware packs rotation into the same report after translation
    if (payload.size() >= 2 * kAxesBytes) {
      state_.rx = le16(payload, kAxesBytes + 0);
      state_.ry = le16(payload, kAxesBytes + 2);
      state_.rz = le16(payload, kAxesBytes + 4);
      ++state_.rotation_reports;
    }
    return true;
  }
  case kRotationReportId: {
    if (payload.size() < kAxesBytes)
      break;
    rotation_.set_data(std::vector<uint8_t>(payload.begin(), payload.begin() + kAxesBytes));
    rotation_.get_rotation(state_.rx, state_.ry, state_.rz);
    ++state_.rotation_reports;
    return true;
  }
  case kButtonsReportId: {
    if (payload.empty())
      break;
    buttons_.set_data(std::vector<uint8_t>(payload.begin(), payload.end()));
    for (size_t i = 0; i < kButtonCount; ++i)
      state_.buttons[i] = buttons_.get_button(static_cast<int>(i + 1));
    ++state_.button_reports;
    return true;
  }
  default:
    break;
  }
  ++state_.unknown_reports;
  return false;
}
