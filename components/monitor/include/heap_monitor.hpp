#pragma once

#include "esp_heap_caps.h"
#include "sdkconfig.h"

#include "base_component.hpp"

namespace espp {
/// HeapMonitor class
/// This class provides functionality to monitor and report heap memory usage
/// in ESP32 systems. It can be used to get information about different heap
/// regions based on their flags (e.g., MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL, etc.).
///
/// It provides methods to retrieve heap information, format it as a string,
/// and log it. The class can be configured with specific heap flags and a
/// name for the monitor.
///
/// \section heap_monitor_ex1 Heap Monitor Example
/// \snippet monitor_example.cpp HeapMonitor Example
class HeapMonitor : public BaseComponent {
public:
  /// Info about a heap region
  struct HeapInfo {
    int heap_flags;    ///< Heap region flags bitmask (e.g., MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
                       ///< etc.)
    size_t free_bytes; ///< Total free heap in bytes
    size_t min_free_bytes;     ///< Minimum free heap in bytes
    size_t largest_free_block; ///< Largest free block in bytes
    size_t allocated_bytes;    ///< Total allocated heap in bytes
    size_t total_size;         ///< Total heap size in bytes
  };

  /// Configuration for HeapMonitor
  struct Config {
    int heap_flags = MALLOC_CAP_DEFAULT;   ///< Heap region flags bitmask (e.g., MALLOC_CAP_8BIT,
                                           ///< MALLOC_CAP_INTERNAL, etc.)
    std::string_view name = "HeapMonitor"; ///< Name of the heap monitor
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::INFO; ///< Log level for heap info
  };

  /// @brief Constructor
  /// @param config Configuration for heap monitor
  HeapMonitor(const Config &config)
      : BaseComponent(config.name, config.log_level)
      , heap_flags_(config.heap_flags) {}

  /// @brief Get the name of the heap region based on the heap flags
  /// @param heap_flags Heap region flags bitmask (e.g., MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL, etc.)
  /// @return Name of the heap region as a string
  static std::string get_region_name(int heap_flags);

  /// @brief Get heap info for a specific heap region
  /// @param heap_flags Heap region flags bitmask (e.g., MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL, etc.)
  /// @return HeapInfo structure with heap information
  static HeapInfo get_info(int heap_flags);

  /// @brief Get heap info for a specific heap region
  /// @param heap_flags Vector of heap region flags bitmasks (e.g., MALLOC_CAP_8BIT,
  /// MALLOC_CAP_INTERNAL, etc.)
  /// @return Vector of HeapInfo structures with heap information for each region
  static std::vector<HeapInfo> get_info(const std::vector<int> &heap_flags);

  /// @brief Get heap info for the configured heap region
  /// @return HeapInfo structure with heap information
  HeapInfo get_info() const { return get_info(heap_flags_); }

  /// @brief Get the header for the CSV output
  /// @return CSV header string
  static const std::string get_csv_header() {
    return "heap_flags,free_bytes,min_free_bytes,largest_free_block,allocated_bytes,total_size";
  }

  /// @brief Get the header for the table output
  /// @return Table header string
  static const std::string get_table_header() {
    return " Min Free /     Free /  Biggest /    Total | Type";
  }

  /// @brief Get a string representation of the heap info in table format
  /// @param heap_flags Heap region flags bitmask(s) (e.g., MALLOC_CAP_8BIT,
  ///        MALLOC_CAP_INTERNAL, etc.)
  /// @return Table string representation of the heap info, each line
  ///         representing a heap region (entry in heap_flags vector)
  static std::string get_table(const std::vector<int> &heap_flags);

  /// @brief Get a string representation of the heap info in CSV format
  /// @param heap_flags Heap region flags bitmask(s) (e.g., MALLOC_CAP_8BIT,
  ///        MALLOC_CAP_INTERNAL, etc.)
  /// @return CSV string representation of the heap info, each line
  ///         representing a heap region (entry in heap_flags vector)
  static std::string get_csv(const std::vector<int> &heap_flags);

protected:
  int heap_flags_;
}; // class HeapMonitor
} // namespace espp

// libfmt formatting for the HeapInfo struct. Supports single line, csv, and
// table outputs using a presentation format specifier
template <> struct fmt::formatter<espp::HeapMonitor::HeapInfo> {
  // presentation format specifier: 's' for single line, 'c' for csv, 't' for table
  char presentation = 's';

  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    // Parse the presentation format and store it in the formatter:
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 's' || *it == 'c' || *it == 't')) {
      presentation = *it++;
    }

    // TODO: Check if reached the end of the range:
    // if (it != end && *it != '}') throw format_error("invalid format");

    // Return the iterator to the end of the parsed range:
    return it;
  }

  template <typename FormatContext>
  auto format(espp::HeapMonitor::HeapInfo const &hi, FormatContext &ctx) const {
    switch (presentation) {
    default:
      // intentional fall-through back to default formatting
    case 's': // single line
      return fmt::format_to(ctx.out(),
                            "HeapInfo: {{ flags: {}, free: {}, min_free: {}, largest_free_block: "
                            "{}, allocated: {}, total: {} }}",
                            hi.heap_flags, hi.free_bytes, hi.min_free_bytes, hi.largest_free_block,
                            hi.allocated_bytes, hi.total_size);
    case 'c': // csv
      return fmt::format_to(ctx.out(), "{},{},{},{},{},{}", hi.heap_flags, hi.free_bytes,
                            hi.min_free_bytes, hi.largest_free_block, hi.allocated_bytes,
                            hi.total_size);
    case 't': // table
      // Format as a table with 8-digit width for each field
      // [ min free bytes / free bytes / largest free block / total size ] heap flags
      return fmt::format_to(ctx.out(), "[{:8d} / {:8d} / {:8d} / {:8d}] {}", hi.min_free_bytes,
                            hi.free_bytes, hi.largest_free_block, hi.total_size,
                            espp::HeapMonitor::get_region_name(hi.heap_flags));
    }
  }
};
