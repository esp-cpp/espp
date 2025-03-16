#pragma once

#include <type_traits>

template <typename E> struct enable_bitmask_operators { static constexpr bool enable = false; };

template <typename E>
constexpr typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type operator|(E lhs,
                                                                                          E rhs) {
  using underlying = typename std::underlying_type<E>::type;
  return static_cast<E>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

template <typename E>
constexpr typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type operator^(E lhs,
                                                                                          E rhs) {
  using underlying = typename std::underlying_type<E>::type;
  return static_cast<E>(static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs));
}

template <typename E>
constexpr typename std::enable_if<enable_bitmask_operators<E>::enable, bool>::type
enum_has_flag(E value, E flag) {
  using underlying = typename std::underlying_type<E>::type;
  return (static_cast<underlying>(value) & static_cast<underlying>(flag)) ==
         static_cast<underlying>(flag);
}

template <typename E>
constexpr typename std::enable_if<enable_bitmask_operators<E>::enable, bool>::type
operator&(E value, E flag) {
  using underlying = typename std::underlying_type<E>::type;
  return (static_cast<underlying>(value) & static_cast<underlying>(flag));
}
