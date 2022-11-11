#pragma once

#define __gnu_linux__

#include "alpaca/alpaca.h"

namespace espp {

  static constexpr auto SERIALIZATION_DEFAULT_OPTIONS =
    alpaca::options::fixed_length_encoding |
    alpaca::options::with_version;

  template <class T, class Container>
  auto serialize(const T& data, Container& container) -> size_t {
    return alpaca::serialize<SERIALIZATION_DEFAULT_OPTIONS>(data, container);
  }

  template <alpaca::options O, class T, class Container>
  auto serialize(const T& data, Container& container) -> size_t {
    return alpaca::serialize<O>(data, container);
  }

  template <class T, class Container>
  auto deserialize(Container& container, std::error_code& ec) -> T {
    return alpaca::deserialize<SERIALIZATION_DEFAULT_OPTIONS, T>(container, ec);
  }

  template <alpaca::options O, class T, class Container>
  auto deserialize(Container& container, std::error_code& ec) -> T {
    return alpaca::deserialize<O, T>(container, ec);
  }

  template <class T, class Container>
  auto deserialize(Container& container, const std::size_t num_bytes, std::error_code& ec) -> T {
    return alpaca::deserialize<SERIALIZATION_DEFAULT_OPTIONS, T>(container, num_bytes, ec);
  }

  template <alpaca::options O, class T, class Container>
  auto deserialize(Container& container, const std::size_t num_bytes, std::error_code& ec) -> T {
    return alpaca::deserialize<O, T>(container, num_bytes, ec);
  }
}
