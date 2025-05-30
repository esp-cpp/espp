#pragma once

#include <string>

#include <nvs.h>
#include <nvs_flash.h>
#include <nvs_handle.hpp>

#include "base_component.hpp"
#include "nvs_errc.hpp"
#include "nvs_handle_espp.hpp"

namespace espp {
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
      ec = make_nvs_error_code(NvsErrc::Init_NVS_Failed);
    }
  }

  /// @brief Erase the NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  bool erase(std::error_code &ec) {
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
      logger_.error("Failed to erase NVS partition: {}", esp_err_to_name(err));
      ec = make_nvs_error_code(NvsErrc::Erase_NVS_Failed);
      return false;
    }
    return true;
  }

  /// @brief Erase a namespace from the NVS
  /// @param[in] ns_name Namespace of the variable to erase
  /// @param[out] ec Saves a std::error_code representing success or failure
  bool erase(std::string_view ns_name, std::error_code &ec) {
    NvsHandle handle(ns_name.data(), ec);
    if (ec)
      return false;

    if (!handle.erase(ec))
      return false;

    return true;
  }

  /// @brief Erase a key from the NVS
  /// @param[in] ns_name Namespace of the variable to erase
  /// @param[in] key NVS Key of the variable to erase
  /// @param[out] ec Saves a std::error_code representing success or failure
  bool erase(std::string_view ns_name, std::string_view key, std::error_code &ec) {
    NvsHandle handle(ns_name.data(), ec);
    if (ec)
      return false;

    if (!handle.erase(key, ec))
      return false;

    return true;
  }

  /// @brief Get a handle to the given namespace
  /// @param[in] ns_name Namespace of the variable to get a handle to
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @return NvsHandle for the given namespace
  NvsHandle get_handle(std::string_view ns_name, std::error_code &ec) {
    return NvsHandle(ns_name, ec);
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
  /// @param[out] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void get_var(std::string_view ns_name, std::string_view key, T &value, std::error_code &ec) {
    get_var(ns_name.data(), key.data(), value, ec);
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Variable to read
  /// @param[in] default_value If the key isn't found in the NVS, this value is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  template <typename T>
  void get_or_set_var(std::string_view ns_name, std::string_view key, T &value, T default_value,
                      std::error_code &ec) {
    get_or_set_var(ns_name.data(), key.data(), value, default_value, ec);
  }

  /// @brief Save a variable in the NVS and commit
  /// @param[in] ns_name Namespace of the variable to save
  /// @param[in] key NVS Key of the variable to save
  /// @param[in] value Variable to save
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Saves the key/variable pair, and commits the NVS.
  template <typename T>
  void set_var(const char *ns_name, const char *key, T value, std::error_code &ec) {
    NvsHandle handle(ns_name, ec);
    if (ec)
      return;

    handle.set(key, value, ec);
    if (ec)
      return;

    handle.commit(ec);
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Variable to read
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair
  template <typename T>
  void get_var(const char *ns_name, const char *key, T &value, std::error_code &ec) {
    NvsHandle handle(ns_name, ec);
    if (ec)
      return;

    handle.get(key, value, ec);
  }

  /// @brief Reads a variable from the NVS
  /// @param[in] ns_name Namespace of the variable to read
  /// @param[in] key NVS Key of the variable to read
  /// @param[out] value Variable to read
  /// @param[in] default_value If the key isn't found in the NVS, this value is saved to NVS
  /// @param[out] ec Saves a std::error_code representing success or failure
  /// @details Read the key/variable pair, If the key isn't found in the NVS, default_value is saved
  /// to NVS
  template <typename T>
  void get_or_set_var(const char *ns_name, const char *key, T &value, T default_value,
                      std::error_code &ec) {
    NvsHandle handle(ns_name, ec);
    if (ec)
      return;

    handle.get(key, value, default_value, ec);
  }
}; // Class Nvs
} // namespace espp
