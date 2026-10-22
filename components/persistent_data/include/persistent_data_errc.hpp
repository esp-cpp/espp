#pragma once

#include <string>
#include <system_error>

namespace espp {

/**
 * @brief Error codes reported by espp::PersistentData.
 * @details Convertible to std::error_code; compare with `ec == PersistentDataErrc::file_not_found`.
 */
enum class PersistentDataErrc {
  // no 0: 0 is "no error"
  file_not_found = 1, ///< No file at the configured path (nothing has been saved yet)
  io_error,           ///< The filesystem refused to open / read / write / rename the file
  file_too_large,     ///< The file is larger than Config::max_file_size
  bad_header,         ///< Wrong magic, wrong length or truncated file (not written by us or
                      ///< cut short)
  bad_checksum,       ///< The header + payload CRC32 does not match (corrupt file)
  unknown_version,    ///< The file's schema version is not in the version list (written by
                      ///< newer firmware, or support for that version was dropped)
  deserialize_failed, ///< The payload does not decode as the type registered for its version
  serialize_failed,   ///< The data could not be serialized
  layout_mismatch,    ///< The payload was written by a type with a different layout than the
                      ///< one registered for its version (a version number was reused)
};

/**
 * @brief The std::error_category for PersistentDataErrc.
 */
inline const std::error_category &persistent_data_category() {
  struct Category : std::error_category {
    const char *name() const noexcept override { return "persistent_data"; }
    std::string message(int ev) const override {
      switch (static_cast<PersistentDataErrc>(ev)) {
      case PersistentDataErrc::file_not_found:
        return "File not found";
      case PersistentDataErrc::io_error:
        return "Filesystem I/O error";
      case PersistentDataErrc::file_too_large:
        return "File larger than the configured maximum";
      case PersistentDataErrc::bad_header:
        return "Bad file header (wrong magic, wrong length or truncated)";
      case PersistentDataErrc::bad_checksum:
        return "Checksum mismatch (corrupt file)";
      case PersistentDataErrc::unknown_version:
        return "Unsupported schema version";
      case PersistentDataErrc::deserialize_failed:
        return "Payload does not decode as the type registered for its version";
      case PersistentDataErrc::serialize_failed:
        return "Could not serialize the data";
      case PersistentDataErrc::layout_mismatch:
        return "Payload layout does not match the type registered for its version";
      default:
        return "Unknown persistent_data error";
      }
    }
  };
  static const Category category;
  return category;
}

/// @brief Build a std::error_code from a PersistentDataErrc.
inline std::error_code make_error_code(PersistentDataErrc e) {
  return {static_cast<int>(e), persistent_data_category()};
}

} // namespace espp

template <> struct std::is_error_code_enum<espp::PersistentDataErrc> : std::true_type {};
