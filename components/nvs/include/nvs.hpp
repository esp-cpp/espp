#pragma once

#include <string>

#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"

#include "base_component.hpp"

namespace {
/**
 * @brief Custom C++ error codes used for Auth
 */
enum class NvsErrc {
  // no 0
  Namespace_Length_Too_Long = 10,
  Key_Length_Too_Long,
  Open_NVS_Handle_Failed,
  Write_NVS_Failed,
  Commit_NVS_Failed,
  Read_NVS_Failed,
  Key_Not_Found,
  Init_NVS_Failed,
  Erase_NVS_Failed,
};

struct NvsErrCategory : std::error_category {
  const char *name() const noexcept override;
  std::string message(int ev) const override;
};

const char *NvsErrCategory::name() const noexcept { return "nvs"; }

std::string NvsErrCategory::message(int ev) const {
  switch (static_cast<NvsErrc>(ev)) {
  case NvsErrc::Namespace_Length_Too_Long:
    return "Namespace too long, must be <= 15 characters";

  case NvsErrc::Key_Length_Too_Long:
    return "Key too long, must be <= 15 characters";

  case NvsErrc::Open_NVS_Handle_Failed:
    return "Failed to open NVS handle";

  case NvsErrc::Write_NVS_Failed:
    return "Failed to write to NVS";

  case NvsErrc::Commit_NVS_Failed:
    return "Failed to commit to NVS";

  case NvsErrc::Read_NVS_Failed:
    return "Failed to read from NVS";

  case NvsErrc::Key_Not_Found:
    return "Key not found in NVS";

  case NvsErrc::Init_NVS_Failed:
    return "Failed to initialize NVS";

  case NvsErrc::Erase_NVS_Failed:
    return "Failed to erase NVS";

  default:
    return "(unrecognized error)";
  }
}

const NvsErrCategory theNvsErrCategory{};
} // namespace

namespace espp {
class NVSHandle : public BaseComponent {
public:
  /// @brief Construct a new NVSHandle object
  /// @param[in] ns_name Namespace for NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Create an NVSHandle object for the key-value pairs in the ns_name namespace
  explicit NVSHandle(const char *ns_name, std::error_code &ec)
      : BaseComponent("NVSHandle", espp::Logger::Verbosity::WARN) {
    if (strlen(ns_name) > 15) {
      logger_.error("Namespace too long, must be <= 15 characters: {}", ns_name);
      ec = make_error_code(NvsErrc::Namespace_Length_Too_Long);
      return;
    }

    esp_err_t err;
    handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
    }
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Reads the value of key into value, if key exists
  template <typename T>
  void get(const char *key, T &value, std::error_code &ec) {
    check_key_length(key, ec);
    if (ec)
      return;
    esp_err_t err;
    T readvalue;
    err = handle->get_item(key, readvalue);
    switch (err) {
    case ESP_OK:
      value = readvalue;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      ec = make_error_code(NvsErrc::Key_Not_Found);
      logger_.error("The value is not initialized in NVS, key = '{}'", key);
      break;
    default:
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
    }
    return;
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void get(std::string_view key, T &value, std::error_code &ec) {
    get(key.data(), value, ec);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] key NVS Key of the bool to read
  /// @param[in] value bool to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair
  void get(const char *key, bool &value, std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    get<uint8_t>(key, u8, ec);
    if (!ec)
      value = static_cast<bool>(u8);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] key NVS Key of the bool to read
  /// @param[in] value bool to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, bool &value, std::error_code &ec) {
    get(key.data(), value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] key NVS Key of the string to read
  /// @param[in] value string to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, std::string &value,
               std::error_code &ec) {
    get(key.data(), value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] key NVS Key of the string to read
  /// @param[in] value string to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(const char *key, std::string &value, std::error_code &ec) {
    check_key_length(key, ec);
    if (ec)
      return;
    esp_err_t err;
    std::size_t len = 0;
    err = handle->get_item_size(nvs::ItemType::SZ, key, len);
    if (err != ESP_OK) {
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
      return;
    }
    value.resize(len);
    err = handle->get_string(key, value.data(), len);
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
      logger_.error("Error {} reading from NVS!", esp_err_to_name(err));
      return;
    }
    return;
  }

  /// @brief Save a variable in the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Saves the key/variable pair without committing the NVS.
  template <typename T>
  void set(const char *key, T value, std::error_code &ec) {
    check_key_length(key, ec);
    if (ec)
      return;
    esp_err_t err;
    err = handle->set_item(key, value);
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Write_NVS_Failed);
      logger_.error("Error {} writing to NVS!", esp_err_to_name(err));
    }
    return;
  }

  /// @brief Save a variable in the NVS
  /// @param[in] key NVS Key of the variable to save
  /// @param[in] value Variable to save
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void set(std::string_view key, T value, std::error_code &ec) {
    set(key.data(), value, ec);
  }

  /// @brief Set a bool in the NVS
  /// @param[in] key NVS Key of the bool to set
  /// @param[in] value bool to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(std::string_view key, bool value, std::error_code &ec) {
    set(key.data(), value, ec);
  }

  /// @brief Set a bool in the NVS
  /// @param[in] key NVS Key of the bool to set
  /// @param[in] value bool to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(const char *key, bool value, std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    set<uint8_t>(key, u8, ec);
  }

  /// @brief Set a string in the NVS
  /// @param[in] key NVS Key of the string to set
  /// @param[in] value string to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(std::string_view key, const std::string &value,
               std::error_code &ec) {
    set(key.data(), value, ec);
  }

  /// @brief Set a string in the NVS
  /// @param[in] key NVS Key of the string to set
  /// @param[in] value string to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(const char *key, const std::string &value,
               std::error_code &ec) {
    check_key_length(key, ec);
    if (ec)
      return;
    esp_err_t err;
    err = handle->set_string(key, value.data());
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Write_NVS_Failed);
      logger_.error("Error {} writing to NVS!", esp_err_to_name(err));
      return;
    }
    return;
  }

  /// @brief Commit changes
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Commits changes to the NVS
  void commit(std::error_code &ec) {
    esp_err_t err = handle->commit();
    if (err != ESP_OK) {
      logger_.error("Error {} committing to NVS!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Commit_NVS_Failed);
    }
    return;
  }

protected:
  std::unique_ptr<nvs::NVSHandle> handle;

  void check_key_length(const char *key, std::error_code &ec) {
    if (strlen(key) > 15) {
      logger_.error("Key too long, must be <= 15 characters: {}", key);
      ec = make_error_code(NvsErrc::Key_Length_Too_Long);
      return;
    }
  }

  /**
   * @brief overload of std::make_error_code used by custom error codes.
   */
  std::error_code make_error_code(NvsErrc e) { return {static_cast<int>(e), theNvsErrCategory}; }

}; // Class NVSHandle

/**
 * @brief Class to interface with the NVS.
 * @details This class is used to interface with the ESP NVS system. It is used to init and erase
 * the partition, or get and set variables
 *
 * @section nvs_ex1 NVS Example
 * @snippet nvs_example.cpp nvs example
 */
class Nvs : public BaseComponent {
public:
  /**
   * @brief Construct a new NVS object.
   */
  explicit Nvs()
      : BaseComponent("Nvs", espp::Logger::Verbosity::WARN) {}

  /// @brief Initialize the NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details If the flash init fails because of no free pages or new version, it erases the NVS
  /// and re-initializes
  void init(std::error_code &ec) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      erase(ec);
      err = nvs_flash_init();
    }
    if (err != ESP_OK) {
      logger_.error("NVS INIT FAILED");
      ec = make_error_code(NvsErrc::Init_NVS_Failed);
    }
  }

  /// @brief Erase the NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  void erase(std::error_code &ec) {
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
      logger_.error("Failed to erase NVS partition: {}", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Erase_NVS_Failed);
      return;
    }
  }

  /// @brief Save a variable in the NVS and commit
  /// @param[in] ns_name Namespace of the variable to save
  /// @param[in] key NVS Key of the variable to save
  /// @param[in] value Variable to save
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void set_var(std::string_view ns_name, std::string_view key, T value, std::error_code &ec) {
    set_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void get_var(std::string_view ns_name, std::string_view key, T &value, std::error_code &ec) {
    get_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[in] default_value If the key isn't found in the NVS, this value is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void get_or_set_var(std::string_view ns_name, std::string_view key, T &value, T default_value,
                      std::error_code &ec) {
    get_or_set_var(ns_name.data(), key.data(), value, default_value, ec);
  }

  /// @brief Set a bool in the NVS
  /// @param[in] ns_name Namespace of the bool to set
  /// @param[in] key NVS Key of the bool to set
  /// @param[in] value bool to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set_var(std::string_view ns_name, std::string_view key, bool value, std::error_code &ec) {
    set_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Set a string in the NVS
  /// @param[in] ns_name Namespace of the string to set
  /// @param[in] key NVS Key of the string to set
  /// @param[in] value string to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set_var(std::string_view ns_name, std::string_view key, const std::string &value,
               std::error_code &ec) {
    set_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Set a string in the NVS
  /// @param[in] ns_name Namespace of the string to set
  /// @param[in] key NVS Key of the string to set
  /// @param[in] value string to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set_var(const char *ns_name, const char *key, const std::string &value,
               std::error_code &ec) {
    check_lengths(ns_name, key, ec);
    if (ec)
      return;
    esp_err_t err;
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return;
    }
    err = handle->set_string(key, value.data());
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Write_NVS_Failed);
      logger_.error("Error {} writing to NVS!", esp_err_to_name(err));
      return;
    }
    err = handle->commit();
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Commit_NVS_Failed);
      logger_.error("Error {} committing to NVS!", esp_err_to_name(err));
    }
    return;
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] ns_name Namespace of the bool to read
  /// @param[in] key NVS Key of the bool to read
  /// @param[in] value bool to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get_var(std::string_view ns_name, std::string_view key, bool &value, std::error_code &ec) {
    get_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] ns_name Namespace of the string to read
  /// @param[in] key NVS Key of the string to read
  /// @param[in] value string to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get_var(std::string_view ns_name, std::string_view key, std::string &value,
               std::error_code &ec) {
    get_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] ns_name Namespace of the string to read
  /// @param[in] key NVS Key of the string to read
  /// @param[in] value string to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get_var(const char *ns_name, const char *key, std::string &value, std::error_code &ec) {
    check_lengths(ns_name, key, ec);
    if (ec)
      return;
    esp_err_t err;
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return;
    }
    std::size_t len = 0;
    err = handle->get_item_size(nvs::ItemType::SZ, key, len);
    if (err != ESP_OK) {
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
      return;
    }
    value.resize(len);
    err = handle->get_string(key, value.data(), len);
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
      logger_.error("Error {} reading from NVS!", esp_err_to_name(err));
      return;
    }
    return;
  }

  /// @brief Reads a bool from the NVS. Writes the default value if the key is not found
  /// @param[in] ns_name Namespace of the bool to read
  /// @param[in] key NVS Key of the bool to read
  /// @param[in] value bool to read
  /// @param[in] default_value If the key isn't found in the NVS, this bool is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get_or_set_var(std::string_view ns_name, std::string_view key, bool &value,
                      bool default_value, std::error_code &ec) {
    get_or_set_var(ns_name.data(), key.data(), value, default_value, ec);
  }

  /// @brief Reads a string from the NVS. Writes the default value if the key is not found
  /// @param[in] ns_name Namespace of the string to read
  /// @param[in] key NVS Key of the string to read
  /// @param[in] value string to read
  /// @param[in] default_value If the key isn't found in the NVS, this string is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get_or_set_var(std::string_view ns_name, std::string_view key, std::string &value,
                      const std::string &default_value, std::error_code &ec) {
    get_or_set_var(ns_name.data(), key.data(), value, default_value, ec);
  }

  /// @brief Reads a string from the NVS. Writes the default value if the key is not found
  /// @param[in] ns_name Namespace of the string to read
  /// @param[in] key NVS Key of the string to read
  /// @param[in] value string to read
  /// @param[in] default_value If the key isn't found in the NVS, this string is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get_or_set_var(const char *ns_name, const char *key, std::string &value,
                      const std::string &default_value, std::error_code &ec) {
    check_lengths(ns_name, key, ec);
    if (ec)
      return;
    // don't set default value unless the key is valid
    value = default_value;
    esp_err_t err;
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return;
    }
    std::size_t len = 0;
    err = handle->get_item_size(nvs::ItemType::SZ, key, len);
    if (err != ESP_OK || len == 0) {
      logger_.warn("The value is not initialized yet! ns::key '{}::{}' set to: {}", ns_name, key,
                   default_value);
      err = handle->set_string(key, default_value.data());
      if (err != ESP_OK) {
        logger_.error("Error {} writing to NVS!", esp_err_to_name(err));
        ec = make_error_code(NvsErrc::Write_NVS_Failed);
        return;
      }
      err = handle->commit();
      if (err != ESP_OK) {
        logger_.error("Error {} committing to NVS!", esp_err_to_name(err));
        ec = make_error_code(NvsErrc::Commit_NVS_Failed);
      }
    } else {
      value.resize(len);
      err = handle->get_string(key, value.data(), len);
      if (err != ESP_OK) {
        ec = make_error_code(NvsErrc::Read_NVS_Failed);
        logger_.error("Error {} reading from NVS!", esp_err_to_name(err));
        return;
      }
    }
    return;
  }

  /// @brief Save a variable in the NVS and commit
  /// @param[in] ns_name Namespace of the variable to save
  /// @param[in] key NVS Key of the variable to save
  /// @param[in] value Variable to save
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Saves the key/variable pair, and commits the NVS.
  template <typename T>
  void set_var(const char *ns_name, const char *key, T value, std::error_code &ec) {
    check_lengths(ns_name, key, ec);
    if (ec)
      return;
    esp_err_t err;
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return;
    }
    err = handle->set_item(key, value);
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Write_NVS_Failed);
      logger_.error("Error {} writing to NVS!", esp_err_to_name(err));
      return;
    }
    err = handle->commit();
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Commit_NVS_Failed);
      logger_.error("Error {} committing to NVS!", esp_err_to_name(err));
    }
    return;
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair
  template <typename T>
  void get_var(const char *ns_name, const char *key, T &value, std::error_code &ec) {
    check_lengths(ns_name, key, ec);
    if (ec)
      return;
    esp_err_t err;
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return;
    }
    T readvalue;
    err = handle->get_item(key, readvalue);
    switch (err) {
    case ESP_OK:
      value = readvalue;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      logger_.error("The value is not initialized in NVS, ns::key = '{}::{}'", ns_name, key);
      ec = make_error_code(NvsErrc::Key_Not_Found);
      break;
    default:
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
    }
    return;
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[in] default_value If the key isn't found in the NVS, this value is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair, If the key isn't found in the NVS, default_value is saved
  /// to NVS
  template <typename T>
  void get_or_set_var(const char *ns_name, const char *key, T &value, T default_value,
                      std::error_code &ec) {
    check_lengths(ns_name, key, ec);
    if (ec)
      return;
    // don't set default value unless the key is valid
    value = default_value;
    esp_err_t err;
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return;
    }
    T readvalue;
    err = handle->get_item(key, readvalue);
    switch (err) {
    case ESP_OK:
      value = readvalue;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      logger_.warn("The value is not initialized yet! ns::key '{}::{}' set to: {}", ns_name, key,
                   default_value);
      err = handle->set_item(key, default_value);
      if (err != ESP_OK) {
        logger_.error("Error {} writing to NVS!", esp_err_to_name(err));
        ec = make_error_code(NvsErrc::Write_NVS_Failed);
        return;
      }
      err = handle->commit();
      if (err != ESP_OK) {
        logger_.error("Error {} committing to NVS!", esp_err_to_name(err));
        ec = make_error_code(NvsErrc::Commit_NVS_Failed);
      }
      break;
    default:
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
    }
    return;
  }

  /// @brief Set a bool in the NVS
  /// @param[in] ns_name Namespace of the bool to set
  /// @param[in] key NVS Key of the bool to set
  /// @param[in] value bool to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set_var(const char *ns_name, const char *key, bool value, std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    set_var<uint8_t>(ns_name, key, u8, ec);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] ns_name Namespace of the bool to read
  /// @param[in] key NVS Key of the bool to read
  /// @param[in] value bool to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair
  void get_var(const char *ns_name, const char *key, bool &value, std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    get_var<uint8_t>(ns_name, key, u8, ec);
    if (!ec)
      value = static_cast<bool>(u8);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] ns_name Namespace of the bool to read
  /// @param[in] key NVS Key of the bool to read
  /// @param[in] value bool to read
  /// @param[in] default_value If the key isn't found in the NVS, this bool is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair, If the key isn't found in the NVS, default_value is saved
  /// to NVS
  void get_or_set_var(const char *ns_name, const char *key, bool &value, bool default_value,
                      std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    get_or_set_var<uint8_t>(ns_name, key, u8, static_cast<uint8_t>(default_value), ec);
    if (!ec)
      value = static_cast<bool>(u8);
  }

  protected:
    void check_lengths(const char *ns_name, const char *key, std::error_code &ec) {
      if (strlen(ns_name) > 15) {
        logger_.error("Namespace too long, must be <= 15 characters: {}", ns_name);
        ec = make_error_code(NvsErrc::Namespace_Length_Too_Long);
        return;
      }
      if (strlen(key) > 15) {
        logger_.error("Key too long, must be <= 15 characters: {}", key);
        ec = make_error_code(NvsErrc::Key_Length_Too_Long);
        return;
      }
    }

    /**
     * @brief overload of std::make_error_code used by custom error codes.
     */
    std::error_code make_error_code(NvsErrc e) { return {static_cast<int>(e), theNvsErrCategory}; }

}; // Class Nvs
} // namespace espp
