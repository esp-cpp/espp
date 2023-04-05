#pragma once

#include <algorithm>

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
    Rgb(const float& r, const float& g, const float& b);
    Rgb(const Rgb& rgb);
    Rgb(const Hsv& hsv);

    Rgb& operator=(const Rgb& other) = default;

    /**
     * @brief Perform additive color blending (averaging)
     * @param rhs Other color to add to this color to create the resultant color
     * @return Resultant color from blending this color with the \p rhs color.
     */
    Rgb operator+ (const Rgb& rhs) const;

    /**
     * @brief Perform additive color blending (averaging)
     * @param rhs Other color to add to this color
     */
    Rgb& operator+= (const Rgb& rhs);

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
    Hsv(const float& h, const float& s, const float& v);
    Hsv(const Hsv& hsv);
    Hsv(const Rgb& rgb);

    Hsv& operator=(const Hsv& other) = default;

    /**
     * @brief Get a RGB representation of this HSV color.
     * @return An RGB object containing the RGB representation.
     */
    Rgb rgb() const;
  };
} // namespace espp


#include "format.hpp"

// for allowing easy serialization/printing of the
// Rgb
template<>
struct fmt::formatter<espp::Rgb>
{
  template<typename ParseContext>
  constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

  template<typename FormatContext>
  auto format(espp::Rgb const& rgb, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "({}, {}, {})", rgb.r, rgb.g, rgb.b);
  }
};

// for allowing easy serialization/printing of the
// Rgb
template<>
struct fmt::formatter<espp::Hsv>
{
  template<typename ParseContext>
  constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

  template<typename FormatContext>
  auto format(espp::Hsv const& hsv, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "({}, {}, {})", hsv.h, hsv.s, hsv.v);
  }
};
