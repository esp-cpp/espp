#pragma once

namespace espp {
/**
 * @brief The types allowed for encoders. Used as template parameters for
 * specialization and some std::enable_if around various encoder
 * functionality.
 */
enum class EncoderType {
  LINEAR,    /**< Linear encoder, can only output raw count values. */
  ROTATIONAL /**< Rotation encoder, can output raw count values as well as rotations, radians, and
                degrees. */
};
} // namespace espp
