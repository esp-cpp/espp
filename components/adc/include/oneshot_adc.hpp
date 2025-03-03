#pragma once

#include <algorithm>
#include <atomic>
#include <mutex>
#include <optional>
#include <unordered_map>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "adc_types.hpp"
#include "base_component.hpp"

namespace espp {
/**
 * @brief OneshotAdc provides a wrapper around the ESP-IDF oneshot adc
 *        subsystem, enabling simple, direct (blocking / slow) measurements of
 *        analog values. The \c read_mv() function will always take a new
 *        measurement (therefore it is blocking).
 *
 * \section adc_oneshot_ex1 Oneshot ADC Example
 * \snippet adc_example.cpp oneshot adc example
 */
class OneshotAdc : public BaseComponent {
public:
  /**
   *  @brief Configure the unit for which to read adc values from the provided
   *         channels.
   *
   *  @note The \c unit must be the same as what is provided in each element
   *         of \p channels.
   */
  struct Config {
    adc_unit_t unit; /**< Unit this oneshot reader will be associated with. @note all channels must
                        use this unit. */
    std::vector<AdcConfig> channels; /**< Channels to read from, with associated attenuations. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity for the adc logger. */
  };

  /**
   * @brief Initialize the oneshot adc reader.
   * @param config Config used to initialize the reader.
   */
  explicit OneshotAdc(const Config &config)
      : BaseComponent("OneShotAdc", config.log_level) {
    init(config);
  }

  /**
   * @brief Delete and destroy the adc reader.
   */
  ~OneshotAdc() {
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle_));
    if (calibrated_) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      logger_.info("deregister Curve Fitting calibration scheme");
      ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc_cali_handle_));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      logger_.info("deregister Line Fitting calibration scheme");
      ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(adc_cali_handle_));
#endif
    }
  }

  /**
   * @brief Take a new ADC reading for all configured channels and convert it to
   *        voltage (mV) if the unit was properly calibrated. If it was not
   *        properly calibrated, then it will return the same value as \c
   *        read_raw().
   * @return std::vector<float> Voltage in mV for all configured channels (if
   *         they were configured).
   */
  std::vector<int> read_all_mv() {
    std::vector<int> values;
    values.reserve(configs_.size());
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto &config : configs_) {
      int raw = 0;
      auto err = adc_oneshot_read(adc_handle_, config.channel, &raw);
      if (err != ESP_OK) {
        logger_.error("Couldn't read oneshot: {} - '{}'", err, esp_err_to_name(err));
        values.push_back(0);
      } else {
        values.push_back(raw_to_mv(raw));
      }
    }
    return values;
  }

  /**
   * @brief Take a new ADC reading for the provided \p config.
   * @param config The channel configuration to take a reading from.
   * @return std::optional<float> raw value for the provided channel config (if
   *         it was configured).
   */
  std::optional<int> read_raw(const AdcConfig &config) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (std::find(configs_.begin(), configs_.end(), config) != configs_.end()) {
      int raw;
      auto err = adc_oneshot_read(adc_handle_, config.channel, &raw);
      if (err == ESP_OK) {
        return raw;
      } else {
        logger_.error("Couldn't read oneshot: {} - '{}'", err, esp_err_to_name(err));
      }
    } else {
      logger_.error("{} not configured for oneshot use!", config);
    }
    return {};
  }

  /**
   * @brief Take a new ADC reading for the provided \p config and convert it to
   *        voltage (mV) if the unit was properly calibrated. If it was not
   *        properly calibrated, then it will return the same value as \c
   *        read_raw().
   * @param config The channel configuration to take a reading from.
   * @return std::optional<float> Voltage in mV for the provided channel config
   *         (if it was configured).
   */
  std::optional<int> read_mv(const AdcConfig &config) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto maybe_raw = read_raw(config);
    if (maybe_raw.has_value()) {
      return raw_to_mv(maybe_raw.value());
    }
    return {};
  }

protected:
  int raw_to_mv(int raw) {
    int mv = raw;
    if (calibrated_) {
      auto err = adc_cali_raw_to_voltage(adc_cali_handle_, raw, &mv);
      if (err != ESP_OK) {
        logger_.error("Could not convert raw to voltage: {} - '{}'", err, esp_err_to_name(err));
      }
    } else {
      logger_.warn("not calibrated, cannot convert raw to mv - returning raw value!");
    }
    return mv;
  }

  void init(const Config &config) {
    adc_oneshot_unit_init_cfg_t init_config;
    memset(&init_config, 0, sizeof(init_config));
    init_config.unit_id = config.unit;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));
    for (const auto &conf : config.channels) {
      auto channel = conf.channel;
      auto attenuation = conf.attenuation;
      auto channel_unit = conf.unit;
      if (config.unit != channel_unit) {
        logger_.warn("Channel configuration invalid, main unit ({}) != channel unit ({})",
                     (int)config.unit, (int)channel_unit);
      }
      adc_oneshot_chan_cfg_t oneshot_config;
      memset(&oneshot_config, 0, sizeof(oneshot_config));
      oneshot_config.bitwidth = ADC_BITWIDTH_DEFAULT;
      oneshot_config.atten = attenuation;
      ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, channel, &oneshot_config));
      configs_.push_back(conf);
    }
    calibration_init(config.unit, configs_[0].attenuation);
  }

  void calibration_init(adc_unit_t unit, adc_atten_t attenuation) {
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated_) {
      logger_.info("calibration scheme version is Curve Fitting");
      adc_cali_curve_fitting_config_t cali_config;
      memset(&cali_config, 0, sizeof(cali_config));
      cali_config.unit_id = unit;
      cali_config.atten = attenuation;
      cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
      ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_);
      if (ret == ESP_OK) {
        calibrated_ = true;
      }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated_) {
      logger_.info("calibration scheme version is Line Fitting");
      adc_cali_line_fitting_config_t cali_config;
      memset(&cali_config, 0, sizeof(cali_config));
      cali_config.unit_id = unit;
      cali_config.atten = attenuation;
      cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
      ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle_);
      if (ret == ESP_OK) {
        calibrated_ = true;
      }
    }
#endif
    if (ret == ESP_OK) {
      logger_.info("Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated_) {
      logger_.warn("eFuse not burnt, skip software calibration");
    } else {
      logger_.error("Invalid arg or no memory");
    }
  }

  std::recursive_mutex mutex_;
  std::vector<AdcConfig> configs_;
  adc_oneshot_unit_handle_t adc_handle_;
  adc_cali_handle_t adc_cali_handle_;
  std::atomic<bool> calibrated_{false};
};
} // namespace espp
