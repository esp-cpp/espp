#pragma once

#define __gnu_linux__

#include "alpaca/alpaca.h"

namespace espp {

/**
 * @brief Serialization convenience wrapper for espp, providing methods for
 * serializaing structures / classes into containers (such as c-sytle arrays,
 * std::arrays, std::vectors, etc.). and for deserializing such containers into
 * structures / classes.
 *
 * @note When defining types to be serialized, you _MUST_ use strictly sized
 * types, such as uint8_t / int8_t or uint32_t or int8_t instead of just int or
 * even size_t.
 *
 * @note This class does not really exist or do anything, but it's the only
 * way I could figure out how to get this documentation built into the system
 * :(
 *
 * \section serialization_ex1 (De-)Serialization Example
 * \snippet serialization_example.cpp serialization example
 * \section serialization_ex2 Complex Structure (De-)Serialization Example
 * \snippet serialization_example.cpp complex serialization example
 */
class __serialization_documentation__ {};

/**
 *  @brief Default serialization options for espp.
 */
static constexpr auto SERIALIZATION_DEFAULT_OPTIONS =
    alpaca::options::fixed_length_encoding | alpaca::options::with_version;

/**
 * @brief Serialize data using the default options into the container.
 * @param data Structure to be serialized.
 * @param container Container class to serialize into.
 * @return Number of bytes that were serialized.
 */
template <class T, class Container> auto serialize(const T &data, Container &container) -> size_t {
  return alpaca::serialize<SERIALIZATION_DEFAULT_OPTIONS>(data, container);
}

/**
 * @brief Serialize data using custom options into the container.
 * @param data Structure to be serialized.
 * @param container Container class to serialize into.
 * @return Number of bytes that were serialized.
 */
template <alpaca::options O, class T, class Container>
auto serialize(const T &data, Container &container) -> size_t {
  return alpaca::serialize<O>(data, container);
}

/**
 * @brief Deserialize from container into new object of type T using default
 *        serialization options.
 * @param container The container of serialized data representing an object of
 *        type T.
 * @param ec The error code that was generated during deserialization, if any.
 * @return The object that was deserialized. Only valid if !ec.
 */
template <class T, class Container>
auto deserialize(Container &container, std::error_code &ec) -> T {
  return alpaca::deserialize<SERIALIZATION_DEFAULT_OPTIONS, T>(container, ec);
}

/**
 * @brief Deserialize from container into new object of type T using custom
 *        serialization options.
 * @param container The container of serialized data representing an object of
 *        type T.
 * @param ec The error code that was generated during deserialization, if any.
 * @return The object that was deserialized. Only valid if !ec.
 */
template <alpaca::options O, class T, class Container>
auto deserialize(Container &container, std::error_code &ec) -> T {
  return alpaca::deserialize<O, T>(container, ec);
}

/**
 * @brief Deserialize num_bytes from container into new object of type T using
 *        default serialization options.
 * @param container The container of serialized data representing an object of
 *        type T.
 * @param num_bytes The number of bytes to deserialize from the container.
 * @param ec The error code that was generated during deserialization, if any.
 * @return The object that was deserialized. Only valid if !ec.
 */
template <class T, class Container>
auto deserialize(Container &container, const std::size_t num_bytes, std::error_code &ec) -> T {
  return alpaca::deserialize<SERIALIZATION_DEFAULT_OPTIONS, T>(container, num_bytes, ec);
}

/**
 * @brief Deserialize num_bytes from container into new object of type T using
 *        custom serialization options.
 * @param container The container of serialized data representing an object of
 *        type T.
 * @param num_bytes The number of bytes to deserialize from the container.
 * @param ec The error code that was generated during deserialization, if any.
 * @return The object that was deserialized. Only valid if !ec.
 */
template <alpaca::options O, class T, class Container>
auto deserialize(Container &container, const std::size_t num_bytes, std::error_code &ec) -> T {
  return alpaca::deserialize<O, T>(container, num_bytes, ec);
}
} // namespace espp
