#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "meshtastic_types.hpp"

namespace espp::meshtastic {

/// Encode a Data message to protobuf wire format.
/// \param data The message to encode
/// \return The encoded bytes
std::vector<uint8_t> encode_data(const DataMessage &data);

/// Decode a Data message from protobuf wire format.
/// \param bytes The encoded bytes
/// \return The decoded message, or nullopt if the bytes are not a valid
///         protobuf encoding
std::optional<DataMessage> decode_data(std::span<const uint8_t> bytes);

/// Encode a User (node info) message to protobuf wire format.
/// \param user The message to encode
/// \return The encoded bytes
std::vector<uint8_t> encode_user(const User &user);

/// Decode a User (node info) message from protobuf wire format.
/// \param bytes The encoded bytes
/// \return The decoded message, or nullopt if the bytes are not a valid
///         protobuf encoding
std::optional<User> decode_user(std::span<const uint8_t> bytes);

/// Encode a Position message to protobuf wire format.
/// \param position The message to encode
/// \return The encoded bytes
std::vector<uint8_t> encode_position(const Position &position);

/// Decode a Position message from protobuf wire format.
/// \param bytes The encoded bytes
/// \return The decoded message, or nullopt if the bytes are not a valid
///         protobuf encoding
std::optional<Position> decode_position(std::span<const uint8_t> bytes);

} // namespace espp::meshtastic
