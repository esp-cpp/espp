#pragma once

#include <cstddef>
#include <cstdint>

namespace espp {

/**
 * @brief Bucketed priority band ("quality of service" class) for schedulable
 *        work such as espp::ThreadPool jobs and espp::SocketReactor
 *        registrations.
 *
 * Bands are deliberately coarse (four fixed buckets, Linux-scheduler style
 * "bands" rather than fine-grained priorities): lower numeric value = more
 * urgent. QosBand::Normal is the default everywhere and preserves the
 * pre-band, single-FIFO-queue behavior when no other band is used.
 */
enum class QosBand : std::uint8_t {
  Critical = 0, ///< Most urgent. Serviced before all other bands.
  High = 1,     ///< Urgent, but yields to Critical.
  Normal = 2,   ///< Default band; matches the pre-band FIFO behavior.
  Low = 3,      ///< Background / bulk work; serviced only when the more urgent
                ///< bands are empty (subject to aging, see espp::ThreadPool).
};

/// Number of QosBand values; bands are indexed 0..kNumQosBands-1 (0 = most
/// urgent).
inline constexpr std::size_t kNumQosBands = 4;

} // namespace espp
