#pragma once

#include <algorithm>

#include "format.hpp"

namespace espp {
class Rgb;
class Hsv;

/**
 * @brief Class representing a color using RGB color space.
 */
class Rgb {
public:
  float r{0}; ///< Red value ∈ [0, 1]
  float g{0}; ///< Green value ∈ [0, 1]
  float b{0}; ///< Blue value ∈ [0, 1]

  Rgb() = default;

  /**
   * @brief Construct an Rgb object from the provided rgb values.
   * @note If provided values outside the range [0,1], it will rescale them to
   *       be within the range [0,1] by dividing by 255.
   * @param r Floating point value for the red channel, should be in range [0,
   *        1]
   * @param g Floating point value for the green channel, should be in range
   *        [0, 1]
   * @param b Floating point value for the blue channel, should be in range
   *        [0, 1]
   */
  explicit Rgb(const float &r, const float &g, const float &b);

  /**
   * @brief Copy-construct an Rgb object from the provided object.
   * @note If provided values outside the range [0,1], it will rescale them to
   *       be within the range [0,1] by dividing by 255.
   * @param rgb Rgb struct containing the values to copy.
   */
  Rgb(const Rgb &rgb);

  /**
   * @brief Construct an Rgb object from the provided Hsv object.
   * @note This calls hsv.rgb() on the provided object, which means that invalid
   *       HSV data (not in the ranges [0,360], [0,1], and [0,1]) could lead to
   *       bad RGB data. The Rgb constructor will automatically convert the
   *       values to be in the proper range, but the perceived color will be
   *       changed.
   * @param hsv Hsv object to copy.
   */
  explicit Rgb(const Hsv &hsv);

  /**
   * @brief Assign the values of the provided Rgb object to this Rgb object.
   * @param other The Rgb object to copy.
   */
  Rgb &operator=(const Rgb &other) = default;

  /**
   * @brief Assign the values of the provided Hsv object to this Rgb object.
   * @note This calls hsv.rgb() on the provided object, which means that
   *       invalid HSV data (not in the ranges [0,360], [0,1], and [0,1])
   */
  Rgb &operator=(const Hsv &hsv);

  /**
   * @brief Perform additive color blending (averaging)
   * @param rhs Other color to add to this color to create the resultant color
   * @return Resultant color from blending this color with the \p rhs color.
   */
  Rgb operator+(const Rgb &rhs) const;

  /**
   * @brief Perform additive color blending (averaging)
   * @param rhs Other color to add to this color
   */
  Rgb &operator+=(const Rgb &rhs);

  /**
   * @brief Get a HSV representation of this RGB color.
   * @return An HSV object containing the HSV representation.
   */
  Hsv hsv() const;
};

/**
 * @brief Class representing a color using HSV color space.
 */
class Hsv {
public:
  float h{0}; ///< Hue ∈ [0, 360]
  float s{0}; ///< Saturation ∈ [0, 1]
  float v{0}; ///< Value ∈ [0, 1]

  Hsv() = default;

  /**
   * @brief Construct a Hsv object from the provided values.
   * @param h Hue - will be clamped to be in range [0, 360]
   * @param s Saturation - will be clamped to be in range [0, 1]
   * @param v Value - will be clamped to be in range [0, 1]
   */
  explicit Hsv(const float &h, const float &s, const float &v);

  /**
   * @brief Copy-construct the Hsv object
   * @param hsv Object to copy from.
   */
  Hsv(const Hsv &hsv);

  /**
   * @brief Construct Hsv object from Rgb object. Calls rgb.hsv() to perform
   *        the conversion.
   * @param rgb The Rgb object to convert and copy.
   */
  explicit Hsv(const Rgb &rgb);

  /**
   * @brief Assign the values of the provided Hsv object to this Hsv object.
   * @param other The Hsv object to copy.
   */
  Hsv &operator=(const Hsv &other) = default;

  /**
   * @brief Assign the values of the provided Rgb object to this Hsv object.
   * @param rgb The Rgb object to convert and copy.
   */
  Hsv &operator=(const Rgb &rgb);

  /**
   * @brief Get a RGB representation of this HSV color.
   * @return An RGB object containing the RGB representation.
   */
  Rgb rgb() const;
};

[[maybe_unused]] static auto color_code(const Rgb &rgb) {
  return fg(fmt::rgb(rgb.r * 255, rgb.g * 255, rgb.b * 255));
}
[[maybe_unused]] static auto color_code(const Hsv &hsv) {
  auto rgb = hsv.rgb();
  return fg(fmt::rgb(rgb.r * 255, rgb.g * 255, rgb.b * 255));
}

// equality operators
[[maybe_unused]] static bool operator==(const Rgb &lhs, const Rgb &rhs) {
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b;
}

[[maybe_unused]] static bool operator==(const Hsv &lhs, const Hsv &rhs) {
  return lhs.h == rhs.h && lhs.s == rhs.s && lhs.v == rhs.v;
}

// inequality operators
[[maybe_unused]] static bool operator!=(const Rgb &lhs, const Rgb &rhs) { return !(lhs == rhs); }

[[maybe_unused]] static bool operator!=(const Hsv &lhs, const Hsv &rhs) { return !(lhs == rhs); }
} // namespace espp

#include "format.hpp"

// for allowing easy serialization/printing of the
// Rgb
template <> struct fmt::formatter<espp::Rgb> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext> auto format(espp::Rgb const &rgb, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {}, {})", rgb.r, rgb.g, rgb.b);
  }
};

// for allowing easy serialization/printing of the
// Rgb
template <> struct fmt::formatter<espp::Hsv> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext> auto format(espp::Hsv const &hsv, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {}, {})", hsv.h, hsv.s, hsv.v);
  }
};
