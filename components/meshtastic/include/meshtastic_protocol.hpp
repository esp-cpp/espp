#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "meshtastic_types.hpp"

namespace espp::meshtastic {

/// The djb2 string hash, used to derive the frequency slot from the channel
/// name.
/// \param str The string to hash
/// \return The 32-bit hash
uint32_t djb2_hash(std::string_view str);

/// The xor-of-all-bytes hash, used (over the channel name and PSK) to derive
/// the channel hash byte carried in the packet header.
/// \param data The bytes to hash
/// \return The 8-bit hash
uint8_t xor_hash(std::span<const uint8_t> data);

/// Compute the channel hash byte for a channel.
/// \param channel_name The (resolved) channel name, e.g. "LongFast"
/// \param psk The expanded pre-shared key (16 or 32 bytes)
/// \return The channel hash byte carried in the packet header
uint8_t channel_hash(std::string_view channel_name, std::span<const uint8_t> psk);

/// Get the display name of a modem preset. This is the name used for the
/// default channel (and therefore the frequency-slot hash) when no explicit
/// channel name is configured.
/// \param preset The modem preset
/// \return The display name, e.g. "LongFast"
const char *preset_display_name(ModemPreset preset);

/// Compute the full modem configuration (frequency, bandwidth, spreading
/// factor, coding rate) for a region / preset / channel name, following the
/// Meshtastic frequency-slot algorithm.
/// \param region The regulatory region
/// \param preset The modem preset
/// \param channel_name The channel name, or empty to use the preset's
///        display name (the default-channel behavior)
/// \param frequency_slot_override 1-based frequency slot override, or 0 to
///        derive the slot from the channel name hash (the default)
/// \return The modem configuration
ModemConfig compute_modem_config(Region region, ModemPreset preset,
                                 std::string_view channel_name = "",
                                 uint32_t frequency_slot_override = 0);

/// Pack a packet header into its 16-byte wire format (little-endian).
/// \param header The header to pack
/// \param out The buffer to pack into (must be at least HEADER_LENGTH bytes)
void pack_header(const PacketHeader &header, uint8_t *out);

/// Unpack a packet header from its 16-byte wire format.
/// \param data The frame data (must be at least HEADER_LENGTH bytes)
/// \return The unpacked header
PacketHeader unpack_header(const uint8_t *data);

} // namespace espp::meshtastic
