#pragma once

#include <string>

#include <nvs.h>
#include <nvs.hpp>
#include <nvs_flash.h>
#include <nvs_handle.hpp>

#include "base_component.hpp"
#include "nvs_errc.hpp"
#include "nvs_handle_espp.hpp"

namespace espp {
/**
 * @brief Class to manage NVS handles.
 * @details This class provides an interface for managing specific ESP NVS namespaces,
 * enabling operations like reading, writing, and committing key-value pairs. It
 * encapsulates all direct interactions with the NVS to ensure proper error handling
 * and namespace management.
 *
 * @section nvshandle_ex1 NvsHandle Example
 * @snippet nvs_example.cpp nvshandle example
 */
class NvsHandle : public BaseComponent {
public:
  /// @brief Default constructor for NvsHandle
  /// @details Initializes the NvsHandle without a specific namespace.
  NvsHandle()
      : BaseComponent("NvsHandle", espp::Logger::Verbosity::WARN)
      , handle_(nullptr) {}

  /// @brief Construct a new NvsHandle object
  /// @param[in] ns_name Namespace for NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Create an NvsHandle object for the key-value pairs in the ns_name namespace
  explicit NvsHandle(const char *ns_name, std::error_code &ec)
      : BaseComponent("NvsHandle", espp::Logger::Verbosity::WARN) {
    if (!init(ns_name, ec)) {
      logger_.error("Failed to initialize NvsHandle with namespace '{}'", ns_name);
    }
  }

  /// @brief Construct a new NvsHandle object
  /// @param[in] ns_name Namespace for NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Create an NvsHandle object for the key-value pairs in the ns_name namespace
  explicit NvsHandle(std::string_view ns_name, std::error_code &ec)
      : BaseComponent("NvsHandle", espp::Logger::Verbosity::WARN) {
    if (!init(ns_name, ec)) {
      logger_.error("Failed to initialize NvsHandle with namespace '{}'", ns_name);
    }
  }

  /// @brief Initializes the NvsHandle with a namespace
  /// @param[in] ns_name Namespace for NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Initializes the NvsHandle object for the key-value pairs in the ns_name namespace
  /// @return true if successful, false otherwise
  bool init(const char *ns_name, std::error_code &ec) {
    if (strlen(ns_name) > 15) {
      logger_.error("Namespace too long, must be <= 15 characters: {}", ns_name);
      ec = make_error_code(NvsErrc::Namespace_Length_Too_Long);
      return false;
    }

    esp_err_t err;
    handle_ = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
    if (err != ESP_OK) {
      logger_.error("Error {} opening NVS handle for namespace '{}'!", esp_err_to_name(err),
                    ns_name);
      ec = make_error_code(NvsErrc::Open_NVS_Handle_Failed);
      return false;
    }
    return true;
  }

  /// @brief Initializes the NvsHandle with a namespace
  /// @param[in] ns_name Namespace for NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Initializes the NvsHandle object for the key-value pairs in the ns_name namespace
  /// @return true if successful, false otherwise
  bool init(std::string_view ns_name, std::error_code &ec) { return init(ns_name.data(), ec); }

  /// @brief Reads a variable from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Reads the value of key into value, if key exists
  template <typename T> void get(const char *key, T &value, std::error_code &ec) {
    if (!check_handle_initialized(ec))
      return;

    if (!check_key(key, ec))
      return;

    esp_err_t err;
    T readvalue;
    err = handle_->get_item(key, readvalue);
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
  /// @param[out] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T> void get(std::string_view key, T &value, std::error_code &ec) {
    get(key.data(), value, ec);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] key NVS Key of the bool to read
  /// @param[out] value bool to read
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
  /// @param[out] value bool to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, bool &value, std::error_code &ec) { get(key.data(), value, ec); }

  /// @brief Reads a float from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Float variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(const char *key, float &value, std::error_code &ec) {
    uint32_t u32;
    get(key, u32, ec);
    if (!ec)
      memcpy(&value, &u32, sizeof(float));
  }

  /// @brief Reads a float from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Float variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, float &value, std::error_code &ec) { get(key.data(), value, ec); }

  /// @brief Reads a string from the NVS
  /// @param[in] key NVS Key of the string to read
  /// @param[inout] value string to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, std::string &value, std::error_code &ec) {
    get(key.data(), value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] key NVS Key of the string to read
  /// @param[out] value string to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(const char *key, std::string &value, std::error_code &ec) {
    if (!check_handle_initialized(ec))
      return;

    if (!check_key(key, ec))
      return;

    esp_err_t err;
    std::size_t len = 0;
    err = handle_->get_item_size(nvs::ItemType::SZ, key, len);
    if (err != ESP_OK) {
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
      return;
    }
    value.resize(len);
    err = handle_->get_string(key, value.data(), len);
    if (err != ESP_OK) {
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
      logger_.error("Error {} reading from NVS!", esp_err_to_name(err));
      return;
    }
    return;
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Variable to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Reads the value of key into value, if key exists
  template <typename T>
  void get(const char *key, T &value, const T default_value, std::error_code &ec) {
    if (!check_handle_initialized(ec))
      return;

    if (!check_key(key, ec))
      return;

    esp_err_t err;
    T readvalue;
    err = handle_->get_item(key, readvalue);
    switch (err) {
    case ESP_OK:
      value = readvalue;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      logger_.warn("The value is not initialized yet! key '{}' set to: {}", key, default_value);
      set(key, default_value, ec);
      if (ec)
        return;
      value = default_value;
      break;
    default:
      logger_.error("Error {} reading!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Read_NVS_Failed);
    }
    return;
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Variable to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void get(std::string_view key, T &value, const T default_value, std::error_code &ec) {
    get(key.data(), value, default_value, ec);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] key NVS Key of the bool to read
  /// @param[out] value bool to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair
  void get(const char *key, bool &value, const bool default_value, std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    uint8_t u8_default = static_cast<uint8_t>(default_value);
    get<uint8_t>(key, u8, u8_default, ec);
    if (!ec)
      value = static_cast<bool>(u8);
  }

  /// @brief Reads a bool from the NVS
  /// @param[in] key NVS Key of the bool to read
  /// @param[out] value bool to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, bool &value, const bool default_value, std::error_code &ec) {
    get(key.data(), value, default_value, ec);
  }

  /// @brief Reads a float from the NVS
  /// @param[in] key NVS Key of the bool to read
  /// @param[out] value float to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair
  void get(const char *key, float &value, const float default_value, std::error_code &ec) {
    uint32_t u32;
    uint32_t u32_default;
    memcpy(&u32_default, &default_value, sizeof(uint32_t));
    get(key, u32, u32_default, ec);
    if (!ec)
      memcpy(&value, &u32, sizeof(float));
  }

  /// @brief Reads a float from the NVS
  /// @param[in] key NVS Key of the float to read
  /// @param[out] value float to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, float &value, const float default_value, std::error_code &ec) {
    get(key.data(), value, default_value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] key NVS Key of the string to read
  /// @param[out] value string to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(std::string_view key, std::string &value, const std::string &default_value,
           std::error_code &ec) {
    get(key.data(), value, default_value, ec);
  }

  /// @brief Reads a string from the NVS
  /// @param[in] key NVS Key of the string to read
  /// @param[out] value string to read
  /// @param[in] default_value Default value to return if key is not found
  /// @param[out] ec Saves a std::error_code representing success or failure
  void get(const char *key, std::string &value, const std::string &default_value,
           std::error_code &ec) {
    if (!check_handle_initialized(ec))
      return;

    if (!check_key(key, ec))
      return;

    esp_err_t err;
    std::size_t len = 0;
    err = handle_->get_item_size(nvs::ItemType::SZ, key, len);
    if (err != ESP_OK || len == 0) {
      logger_.warn("The value is not initialized yet! key '{}' set to: {}", key, default_value);
      set(key, default_value, ec);
      if (ec)
        return;
      value = default_value;
    } else {
      value.resize(len);
      err = handle_->get_string(key, value.data(), len);
      if (err != ESP_OK) {
        ec = make_error_code(NvsErrc::Read_NVS_Failed);
        logger_.error("Error {} reading from NVS!", esp_err_to_name(err));
        return;
      }
    }
  }

  /// @brief Save a variable in the NVS
  /// @param[in] key NVS Key of the variable to read
  /// @param[in] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Saves the key/variable pair without committing the NVS.
  template <typename T> void set(const char *key, T value, std::error_code &ec) {
    if (!check_handle_initialized(ec))
      return;

    if (!check_key(key, ec))
      return;

    esp_err_t err;
    err = handle_->set_item(key, value);
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
  template <typename T> void set(std::string_view key, T value, std::error_code &ec) {
    set(key.data(), value, ec);
  }

  /// @brief Set a bool in the NVS
  /// @param[in] key NVS Key of the bool to set
  /// @param[in] value bool to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(std::string_view key, bool value, std::error_code &ec) { set(key.data(), value, ec); }

  /// @brief Set a bool in the NVS
  /// @param[in] key NVS Key of the bool to set
  /// @param[in] value bool to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(const char *key, bool value, std::error_code &ec) {
    uint8_t u8 = static_cast<uint8_t>(value);
    set<uint8_t>(key, u8, ec);
  }

  /// @brief Set a float in the NVS
  /// @param[in] key NVS Key of the float to set
  /// @param[in] value float to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(std::string_view key, float value, std::error_code &ec) { set(key.data(), value, ec); }

  /// @brief Set a float in the NVS
  /// @param[in] key NVS Key of the float to set
  /// @param[in] value float to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(const char *key, float value, std::error_code &ec) {
    uint32_t u32;
    memcpy(&u32, &value, sizeof(uint32_t));
    set(key, u32, ec);
  }

  /// @brief Set a string in the NVS
  /// @param[in] key NVS Key of the string to set
  /// @param[in] value string to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(std::string_view key, const std::string &value, std::error_code &ec) {
    set(key.data(), value, ec);
  }

  /// @brief Set a string in the NVS
  /// @param[in] key NVS Key of the string to set
  /// @param[in] value string to set
  /// @param[out] ec Saves a std::error_code representing success or failure
  void set(const char *key, const std::string &value, std::error_code &ec) {
    if (!check_handle_initialized(ec))
      return;

    if (!check_key(key, ec))
      return;

    esp_err_t err;
    err = handle_->set_string(key, value.data());
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
    if (!check_handle_initialized(ec))
      return;

    esp_err_t err = handle_->commit();
    if (err != ESP_OK) {
      logger_.error("Error {} committing to NVS!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Commit_NVS_Failed);
    }
    return;
  }

  /// @brief Erase a key from the NVS
  /// @param[in] key NVS Key to erase
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @return true if successful, false otherwise
  bool erase(std::string_view key, std::error_code &ec) { return erase(key.data(), ec); }

  /// @brief Erase a key from the NVS
  /// @param[in] key NVS Key to erase
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @return true if successful, false otherwise
  bool erase(const char *key, std::error_code &ec) {
    if (!handle_) {
      logger_.error("NVS Handle not initialized!");
      return false;
    }

    if (!check_key(key, ec))
      return false;

    esp_err_t err = handle_->erase_item(key);
    if (err != ESP_OK) {
      logger_.error("Error {} erasing key '{}' from NVS!", esp_err_to_name(err), key);
      ec = make_error_code(NvsErrc::Erase_NVS_Key_Failed);
      return false;
    }
    return true;
  }

  /// @brief Erase all keys from the NVS associated with the namespace / handle
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @return true if successful, false otherwise
  bool erase(std::error_code &ec) {
    if (!handle_) {
      logger_.error("NVS Handle not initialized!");
      return false;
    }

    esp_err_t err = handle_->erase_all();
    if (err != ESP_OK) {
      logger_.error("Error {} erasing all keys from NVS!", esp_err_to_name(err));
      ec = make_error_code(NvsErrc::Erase_NVS_Namespace_Failed);
      return false;
    }
    return true;
  }

protected:
  std::unique_ptr<nvs::NVSHandle> handle_;

  bool check_key(const char *key, std::error_code &ec) {
    if (strlen(key) > 15) {
      logger_.error("Key too long, must be <= 15 characters: {}", key);
      ec = make_error_code(NvsErrc::Key_Length_Too_Long);
      return false;
    }
    return true;
  }

  bool check_handle_initialized(std::error_code &ec) {
    if (!handle_) {
      ec = make_error_code(NvsErrc::Handle_Uninitialized);
      logger_.error("NVS Handle not initialized!");
      return false;
    }
    return true;
  }

  /**
   * @brief overload of std::make_error_code used by custom error codes.
   */
  std::error_code make_error_code(NvsErrc e) { return {static_cast<int>(e), theNvsErrCategory}; }

}; // Class NvsHandle
} // namespace espp
