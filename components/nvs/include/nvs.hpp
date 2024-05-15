#pragma once

#include <string>

#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"
#include "base_component.hpp"

namespace espp {

class Nvs : public BaseComponent {
public:

    explicit Nvs()
      : BaseComponent( "Nvs", espp::Logger::Verbosity::WARN) {}


    void init_nvs(std::error_code &ec) {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        if (err != ESP_OK) {
            logger_.error("NVS INIT FAILED");
            ec = std::make_error_code(std::errc::protocol_error);
        }
    }

    template<typename T> void set_nvs_var(const char *ns_name, const char *key, T value, std::error_code &ec) {
        init_nvs(ec);
        if (ec)
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

    template<typename T> void get_nvs_var(const char *ns_name, const char *key, T &value, T default_value, std::error_code &ec) {
        value = default_value;
        init_nvs(ec);
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

    void get_nvs_var(const char *ns_name, const char *key, bool &value, bool default_value, std::error_code &ec) {
        uint8_t u8 = static_cast<uint8_t>(value);
        get_nvs_var<uint8_t>("system", "factory_mode", u8, static_cast<uint8_t>(default_value), ec);
        if(!ec)
            value = static_cast<bool>(u8);
    }

}; //Class Nvs
} // namespace espp