#pragma once

#include <functional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace espp {
namespace detail {
struct EventMap {
  // topic -> [component1, component2, ...] mapping for publishing components
  std::unordered_map<std::string, std::vector<std::string>> publishers;
  // topic -> [component1, component2, ...] mapping for subscribing components
  std::unordered_map<std::string, std::vector<std::string>> subscribers;
};

/**
 * @brief Template function to find a value's existence and index in a
 *        container. Useful primarily for vectors and the like.
 * @param value The value to retrieve existence/index for.
 * @param container The container to search for \p value in.
 * @return std::pair<bool, uint32_t> {True if \p value was found in \p
 *         container, index of \p value in \p container if found}
 */
template <typename T, typename C>
std::pair<bool, uint32_t> get_index_in_container(const T &value, const C &container) {
  auto iter = std::find(container.begin(), container.end(), value);
  auto exists = iter != container.end();
  size_t index = container.size();
  if (exists) {
    index = std::distance(container.begin(), iter);
  }
  return {exists, index};
}
} // namespace detail
} // namespace espp
