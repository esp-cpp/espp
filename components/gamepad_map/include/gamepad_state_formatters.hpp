#pragma once

#include "format.hpp"
#include "gamepad_state.hpp"

/// Formats a GamepadState as the pressed buttons by name plus the axes, e.g.
/// `{buttons: [south, up], lx: 0, ly: -32767, rx: 0, ry: 0, lt: 0, rt: 32767}`.
template <> struct fmt::formatter<espp::GamepadState> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::GamepadState &s, FormatContext &ctx) const -> decltype(ctx.out()) {
    using B = espp::GamepadState::Button;
    using A = espp::GamepadState::Axis;
    auto out = fmt::format_to(ctx.out(), "{{buttons: [");
    bool first = true;
    for (size_t i = 0; i < espp::GamepadState::BUTTON_COUNT; i++) {
      const auto b = static_cast<B>(i);
      if (!s.pressed(b))
        continue;
      out = fmt::format_to(out, "{}{}", first ? "" : ", ", espp::GamepadState::name(b));
      first = false;
    }
    return fmt::format_to(out, "], lx: {}, ly: {}, rx: {}, ry: {}, lt: {}, rt: {}}}",
                          s.axis(A::LeftX), s.axis(A::LeftY), s.axis(A::RightX), s.axis(A::RightY),
                          s.axis(A::LeftTrigger), s.axis(A::RightTrigger));
  }
};
