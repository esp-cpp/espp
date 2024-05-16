#pragma once

#include <string>

#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"

#include "base_component.hpp"

namespace espp {
/**
 * @brief Class to interface with the NVS.
 * @details This class is used to interface with the ESP NVS system. It is used to init and erase the partition, or get and set variables
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
      : BaseComponent( "Nvs", espp::Logger::Verbosity::WARN) {}

    /// @brief Initialize the NVS
    /// @param[out] ec Saves a std::error_code representing success or failure
    /// @details If the flash init fails because of no free pages or new version, it erases the NVS and re-initializes
    void init(std::error_code &ec) {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            erase(ec);
            err = nvs_flash_init();
        }
        if (err != ESP_OK) {
            logger_.error("NVS INIT FAILED");
            ec = std::make_error_code(std::errc::protocol_error);
        }
    }

    /// @brief Erase the NVS
    /// @param[out] ec Saves a std::error_code representing success or failure
    void erase(std::error_code ec) {
        esp_err_t err = nvs_flash_erase();
        if (err != ESP_OK) {
            logger_.error("Failed to erase NVS partition: {%s}", esp_err_to_name(err));
            ec = std::make_error_code(std::errc::protocol_error);
            return;
        }
    }

    /// @brief Save a variable in the NVS and commit
    /// @param[in] ns_name Namespace of the variable to save
    /// @param[in] key NVS Key of the variable to save
    /// @param[in] value Variable to save
    /// @param[out] ec Saves a std::error_code representing success or failure
    /// @details Saves the key/variable pair, and commits the NVS. 
    template<typename T> void set_var(const char *ns_name, const char *key, T value, std::error_code &ec) {
        check_lengths(ns_name, key, ec);
        if(ec)
            return;
        esp_err_t err;
        std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
        if (err != ESP_OK) {
            logger_.error("Error {%s} opening NVS handle!", esp_err_to_name(err));
            ec = std::make_error_code(std::errc::protocol_error);
            return;
        }
        err = handle->set_item(key, value);
        if (err != ESP_OK) {
            ec = std::make_error_code(std::errc::protocol_error);
            logger_.error("Error {%s} writing to NVS!", esp_err_to_name(err));
            return;
        }
        err = handle->commit();
        if (err != ESP_OK) {
            ec = std::make_error_code(std::errc::protocol_error);
            logger_.error("Error {%s} committing to NVS!", esp_err_to_name(err));
        }
        return;
    }

    /// @brief Reads a variable from the NVS
    /// @param[in] ns_name Namespace of the variable to read
    /// @param[in] key NVS Key of the variable to read
    /// @param[in] value Variable to read
    /// @param[out] ec Saves a std::error_code representing success or failure
    /// @details Read the key/variable pair
    template<typename T> void get_var(const char *ns_name, const char *key, T &value, std::error_code &ec) {
        check_lengths(ns_name, key, ec);
        if(ec)
            return;
        esp_err_t err;
        std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
        if (err != ESP_OK) {
            logger_.error("Error {%s} opening NVS handle!", esp_err_to_name(err));
            ec = std::make_error_code(std::errc::protocol_error);
            return;
        }    
        T readvalue;
        err = handle->get_item(key, readvalue);
        switch (err) {
        case ESP_OK:
            value = readvalue;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            logger_.error("The value is not initialized in NVS, key: {%s}", key);
            ec = std::make_error_code(std::errc::invalid_argument);
            break;
        default:
            logger_.error("Error {%s} reading!", esp_err_to_name(err));
            ec = std::make_error_code(std::errc::protocol_error);
        }
        return;
    }

    /// @brief Reads a variable from the NVS
    /// @param[in] ns_name Namespace of the variable to read
    /// @param[in] key NVS Key of the variable to read
    /// @param[in] value Variable to read
    /// @param[in] default_value If the key isn't found in the NVS, this value is saved to NVS
    /// @param[out] ec Saves a std::error_code representing success or failure
    /// @details Read the key/variable pair, If the key isn't found in the NVS, default_value is saved to NVS
    template<typename T> void get_or_set_var(const char *ns_name, const char *key, T &value, T default_value, std::error_code &ec) {
        value = default_value;
        check_lengths(ns_name, key, ec);
        if (ec)
            return;
        esp_err_t err;
        std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle(ns_name, NVS_READWRITE, &err);
        if (err != ESP_OK) {
            logger_.error("Error {%s} opening NVS handle!", esp_err_to_name(err));
            ec = std::make_error_code(std::errc::protocol_error);
            return;
        }    
        T readvalue;
        err = handle->get_item(key, readvalue);
        switch (err) {
        case ESP_OK:
            value = readvalue;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            logger_.warn("The value is not initialized yet! Set to: {}", default_value);
            err = handle->set_item(key, default_value);
            if (err != ESP_OK) {
                logger_.error("Error {%s} writing to NVS!", esp_err_to_name(err));
                ec = std::make_error_code(std::errc::protocol_error);
                return;
            }
            err = handle->commit();
            if (err != ESP_OK) {
                logger_.error("Error {%s} committing to NVS!", esp_err_to_name(err));
                ec = std::make_error_code(std::errc::protocol_error);
            }
            break;
        default:
            logger_.error("Error {%s} reading!", esp_err_to_name(err));
            ec = std::make_error_code(std::errc::protocol_error);
        }
        return;
    }

    /// @brief Reads a bool from the NVS
    /// @param[in] ns_name Namespace of the bool to read
    /// @param[in] key NVS Key of the bool to read
    /// @param[in] value bool to read
    /// @param[out] ec Saves a std::error_code representing success or failure
    /// @details Read the key/variable pair
    void get_var(const char *ns_name, const char *key, bool &value, std::error_code &ec) {
        uint8_t u8 = static_cast<uint8_t>(value);
        get_var<uint8_t>("system", "factory_mode", u8, ec);
        if(!ec)
            value = static_cast<bool>(u8);
    }

    /// @brief Reads a bool from the NVS
    /// @param[in] ns_name Namespace of the bool to read
    /// @param[in] key NVS Key of the bool to read
    /// @param[in] value bool to read
    /// @param[in] default_value If the key isn't found in the NVS, this bool is saved to NVS
    /// @param[out] ec Saves a std::error_code representing success or failure
    /// @details Read the key/variable pair, If the key isn't found in the NVS, default_value is saved to NVS
    void get_or_set_var(const char *ns_name, const char *key, bool &value, bool default_value, std::error_code &ec) {
        uint8_t u8 = static_cast<uint8_t>(value);
        get_or_set_var<uint8_t>("system", "factory_mode", u8, static_cast<uint8_t>(default_value), ec);
        if(!ec)
            value = static_cast<bool>(u8);
    }

protected:
    void check_lengths(const char *ns_name, const char *key, std::error_code &ec) {
        if(strlen(ns_name) > 15) {
            logger_.error("Namespace too long, must be <= 15 characters: {}", ns_name);
            ec = std::make_error_code(std::errc::argument_out_of_domain);
            return;
        }
        if(strlen(key) > 15) {
            logger_.error("Key too long, must be <= 15 characters: {}", key);
            ec = std::make_error_code(std::errc::argument_out_of_domain);
            return;
        }
    }

}; //Class Nvs
} // namespace espp