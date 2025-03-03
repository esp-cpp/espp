#pragma once

#include <atomic>
#include <optional>
#include <unordered_map>
#include <vector>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"

#include "adc_types.hpp"
#include "base_component.hpp"
#include "task.hpp"

namespace espp {
/**
 * @brief ContinuousAdc provides a wrapper around the ESP-IDF continuous adc
 *        subsystem, enabling high-frequency, filtered measurements of analog
 *        values. The \c get_mv() function will always return the most up to
 *        date value, without needing to perform additional reads (therefore
 *        it is non-blocking).
 *
 * @note The available modes, frequencies, and throughput is dependent on
 *       whether you run on ESP32, ESP32s2, or ESP32s3.
 *
 * \section adc_continuous_ex1 Continuous ADC Example
 * \snippet adc_example.cpp continuous adc example
 */
class ContinuousAdc : public espp::BaseComponent {
public:
  /**
   *  @brief Configure the sample rate (globally applied to each channel),
   *  select the number of channels, and the conversion mode (for S2 S3)
   */
  struct Config {
    size_t sample_rate_hz;           /**< Samples per second to read from each channel. */
    std::vector<AdcConfig> channels; /**< Channels to read from, with associated attenuations. */
    adc_digi_convert_mode_t convert_mode; /**< Conversion mode (unit 1, unit 2, alternating, or
                                             both). May depend on ESP32 chip. */
    size_t task_priority{5};       /**< Priority to run the adc data reading / filtering task. */
    size_t window_size_bytes{256}; /**< Amount of bytes to allocate for the DMA buffer when reading
                                      results. Larger values lead to more filtering. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity for the adc logger. */
  };

  /**
   * @brief Initialize and start the continuous adc reader.
   * @param config Config used to initialize the reader.
   */
  explicit ContinuousAdc(const Config &config)
      : BaseComponent("ContinuousAdc", config.log_level)
      , sample_rate_hz_(config.sample_rate_hz)
      , window_size_bytes_(config.window_size_bytes)
      , num_channels_(config.channels.size())
      , conv_mode_(config.convert_mode)
      , result_data_(window_size_bytes_, 0xcc) {
    // set the rate limit for the logger
    logger_.set_rate_limit(std::chrono::milliseconds(100));
    // initialize the adc continuous subsystem
    init(config.channels);
    // and start the task
    using namespace std::placeholders;
    task_ = espp::Task::make_unique(
        {.callback = std::bind(&ContinuousAdc::update_task, this, _1, _2, _3),
         .task_config =
             {
                 .name = "ContinuousAdc Task",
                 .priority = config.task_priority,
             },
         .log_level = espp::Logger::Verbosity::WARN});
    task_->start();
  }

  /**
   * @brief Stop, deinit, and destroy the adc reader.
   */
  ~ContinuousAdc() {
    // to force the task to return from the wait
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle_, &mustYield);
    // then stop the task
    task_->stop();
    stop();
    ESP_ERROR_CHECK(adc_continuous_deinit(handle_));
    // clean up the calibration data
    for (const auto &config : configs_) {
      [[maybe_unused]] auto id = get_id(config);
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(cali_handles_[id]));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(cali_handles_[id]));
#endif
    }
  }

  /**
   * @brief Start the continuous adc reader.
   */
  void start() {
    if (running_) {
      return;
    }
    running_ = true;
    ESP_ERROR_CHECK(adc_continuous_start(handle_));
  }

  /**
   * @brief Stop the continuous adc reader.
   */
  void stop() {
    if (!running_) {
      return;
    }
    running_ = false;
    ESP_ERROR_CHECK(adc_continuous_stop(handle_));
  }

  /**
   * @brief Get the most up to date filtered voltage (in mV) from the
   *        provided \p channel.
   * @param config The config used to initialize the channel (includes unit and
   *        channel)
   * @return std::optional<float> voltage in mV for the provided channel (if
   *         it was configured and the adc is running).
   */
  std::optional<float> get_mv(const AdcConfig &config) {
    if (!running_) {
      return {};
    }
    std::lock_guard<std::mutex> lock{data_mutex_};
    int id = get_id(config);
    if (values_.find(id) != values_.end()) {
      return values_[id];
    } else {
      return {};
    }
  }

  /**
   * @brief Get the most up to date sampling rate (in Hz) from the provided
   *        \p channel.
   * @param config The config used to initialize the channel (includes unit and
   *        channel)
   * @return std::optional<float> Rate in Hz for the provided channel (if it
   *         was configured and the adc is running).
   */
  std::optional<float> get_rate(const AdcConfig &config) {
    if (!running_) {
      return {};
    }
    std::lock_guard<std::mutex> lock{data_mutex_};
    int id = get_id(config);
    if (actual_rates_.find(id) != actual_rates_.end()) {
      return actual_rates_[id];
    } else {
      return {};
    }
  }

protected:
  int get_id(const AdcConfig &config) { return get_id(config.unit, config.channel); }

  int get_id(adc_unit_t unit, adc_channel_t channel) {
    // We want to make sure that the id is unique for each channel, so we
    // multiply the unit by 32 (which is currently larger than the number of
    // channels on any ESP32 chip), and then add the channel number to get a
    // unique id.
    return static_cast<int>(unit) * 32 + static_cast<int>(channel);
  }

  bool update_task(std::mutex &, std::condition_variable &, bool &) {
    task_handle_ = xTaskGetCurrentTaskHandle();
    static auto previous_timestamp = std::chrono::high_resolution_clock::now();
    // wait until conversion is ready (will be notified by the registered
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    auto current_timestamp = std::chrono::high_resolution_clock::now();
    for (const auto &config : configs_) {
      auto id = get_id(config);
      sums_[id] = 0;
      num_samples_[id] = 0;
    }
    esp_err_t ret;
    uint32_t ret_num = 0;
    ret = adc_continuous_read(handle_, result_data_.data(), window_size_bytes_, &ret_num, 0);
    if (ret == ESP_ERR_TIMEOUT) {
      // this is ok, we read faster than the hardware could give us data
      return false;
    }
    // we are ok with ERR_INVALID_STATE since we actually have good data lol...
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
      logger_.error("Could not read adc data: {} - '{}'", ret, esp_err_to_name(ret));
      return false;
    }
    for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *p = reinterpret_cast<adc_digi_output_data_t *>(&result_data_[i]);
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
      if (output_format_ == ADC_DIGI_OUTPUT_FORMAT_TYPE1) {
        auto unit = (conv_mode_ == ADC_CONV_SINGLE_UNIT_1) ? ADC_UNIT_1 : ADC_UNIT_2;
        auto channel = (adc_channel_t)p->type1.channel;
        auto id = get_id(unit, channel);
        auto data = p->type1.data;
        sums_[id] += data;
        num_samples_[id]++;
      }
#endif
#if !CONFIG_IDF_TARGET_ESP32
      if (output_format_ == ADC_DIGI_OUTPUT_FORMAT_TYPE2) {
        if (check_valid_data(p)) {
          auto unit = (adc_unit_t)p->type2.unit;
          auto channel = (adc_channel_t)p->type2.channel;
          auto data = p->type2.data;
          auto id = get_id(unit, channel);
          sums_[id] += data;
          num_samples_[id]++;
        } else {
          // we have invalid data? how?!
          logger_.error("invalid data!");
        }
      }
#endif
    } // for()
    // measure elapsed time
    float elapsed_seconds =
        std::chrono::duration<float>(current_timestamp - previous_timestamp).count();
    if (elapsed_seconds == 0) {
      // prevent divide by 0 later
      elapsed_seconds = 0.001f;
    }
    previous_timestamp = current_timestamp;
    // filter / update the measurements
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      for (auto &config : configs_) {
        auto id = get_id(config);
        float num_samples = num_samples_[id];
        if (num_samples > 0) {
          float sum = sums_[id];
          // note: the data collected above is uncalibrated (raw) values so we need to convert
          // to millivolts
          // TODO: use ESP32 hardware-accelerated filter
          // simple filter
          int millivolts = 0;
          adc_cali_raw_to_voltage(cali_handles_[id], sum / num_samples, &millivolts);
          values_[id] = float(millivolts);
          actual_rates_[id] = num_samples / elapsed_seconds;
        }
        logger_.debug_rate_limited("CH{} - {}, {}, {:.2f}, {:.2f}", (int)config.channel, sums_[id],
                                   num_samples_[id], values_[id], actual_rates_[id]);
      }
    }

    // don't want to stop the task
    return false;
  }

#if !CONFIG_IDF_TARGET_ESP32
  static bool check_valid_data(const adc_digi_output_data_t *data) {
    const unsigned int unit = data->type2.unit;
    if (unit > 2)
      return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
      return false;
    return true;
  }
#endif

  void init(const std::vector<AdcConfig> &channels) {
    adc_continuous_handle_cfg_t adc_config;
    memset(&adc_config, 0, sizeof(adc_config));
    adc_config.max_store_buf_size = 1024;
    adc_config.conv_frame_size = window_size_bytes_;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle_));

#if CONFIG_IDF_TARGET_ESP32
    // ESP32 only supports ADC1 DMA mode
    output_format_ = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
    if (conv_mode_ != ADC_CONV_SINGLE_UNIT_1) {
      logger_.error("Configured for ESP32, which only supports ADC_CONV_SINGLE_UNIT_1, "
                    "but provided invalid output convert mode {}",
                    (int)conv_mode_);
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    // if we get both units, then it is type 2, otherwise type1
    output_format_ = (conv_mode_ == ADC_CONV_ALTER_UNIT || conv_mode_ == ADC_CONV_BOTH_UNIT)
                         ? ADC_DIGI_OUTPUT_FORMAT_TYPE2
                         : ADC_DIGI_OUTPUT_FORMAT_TYPE1;
#elif CONFIG_IDF_TARGET_ESP32S3
    // S3 only supports type 2
    output_format_ = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#endif
    adc_continuous_config_t dig_cfg;
    memset(&dig_cfg, 0, sizeof(dig_cfg));
    dig_cfg.sample_freq_hz = sample_rate_hz_ * num_channels_;
    dig_cfg.conv_mode = conv_mode_;
    dig_cfg.format = output_format_;

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX];
    memset(&adc_pattern[0], 0, sizeof(adc_digi_pattern_config_t) * SOC_ADC_PATT_LEN_MAX);
    dig_cfg.pattern_num = num_channels_;

    for (int i = 0; i < num_channels_; i++) {
      auto conf = channels[i];
      adc_pattern[i].atten = conf.attenuation;
      adc_pattern[i].channel = conf.channel;
      adc_pattern[i].unit = conf.unit;
      adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

      logger_.info("adc_pattern[{}].atten is 0x{:02x}", i, (int)adc_pattern[i].atten);
      logger_.info("adc_pattern[{}].channel is 0x{:02x}", i, (int)adc_pattern[i].channel);
      logger_.info("adc_pattern[{}].unit is 0x{:02x}", i, (int)adc_pattern[i].unit);

      // get the id for this config
      auto id = get_id(conf);

      // create the calibration data
      auto calibrated =
          init_calibration(conf.unit, conf.attenuation, (adc_bitwidth_t)adc_pattern[i].bit_width,
                           &cali_handles_[id]);
      logger_.info("adc_pattern[{}].cali_handle is calibrated: {}", i, calibrated);

      // save the channel
      configs_.push_back(conf);
      // update our dictionaries
      sums_[id] = 0;
      num_samples_[id] = 0;
      values_[id] = 0;
      actual_rates_[id] = 0;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle_, &dig_cfg));

    adc_continuous_evt_cbs_t cbs;
    memset(&cbs, 0, sizeof(cbs));
    cbs.on_conv_done = s_conv_done_cb;
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle_, &cbs, &task_handle_));
  }

  bool init_calibration(adc_unit_t unit, adc_atten_t atten, adc_bitwidth_t bitwidth,
                        adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    logger_.info("calibration scheme version is {}", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config;
    memset(&cali_config, 0, sizeof(cali_config));
    cali_config.unit_id = unit;
    cali_config.atten = atten;
    cali_config.bitwidth = bitwidth;
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    // cppcheck-suppress knownConditionTrueFalse
    if (!calibrated) {
      logger_.info("calibration scheme version is {}", "Line Fitting");
      adc_cali_line_fitting_config_t cali_config;
      memset(&cali_config, 0, sizeof(cali_config));
      cali_config.unit_id = unit;
      cali_config.atten = atten;
      cali_config.bitwidth = bitwidth;
      ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
      if (ret == ESP_OK) {
        calibrated = true;
      }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
      logger_.info("Calibration Success");
      // cppcheck-suppress knownConditionTrueFalse
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
      logger_.warn("eFuse not burnt, skip software calibration");
    } else {
      logger_.error("Invalid arg or no memory");
    }

    return calibrated;
  }

  static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    auto s_task_handle = (TaskHandle_t *)user_data;
    if (*s_task_handle) {
      vTaskNotifyGiveFromISR(*s_task_handle, &mustYield);
    }
    return (mustYield == pdTRUE);
  }

  size_t sample_rate_hz_;
  size_t window_size_bytes_;
  size_t num_channels_;
  adc_continuous_handle_t handle_;
  adc_digi_convert_mode_t conv_mode_;
  adc_digi_output_format_t output_format_;
  std::vector<uint8_t> result_data_;
  std::unique_ptr<Task> task_;
  TaskHandle_t task_handle_{NULL};
  std::mutex data_mutex_;

  std::atomic<bool> running_{false};

  std::vector<AdcConfig> configs_;
  // these maps are indexed by get_id(config.unit, config.channel)
  std::unordered_map<int, adc_cali_handle_t> cali_handles_;
  std::unordered_map<int, size_t> sums_;
  std::unordered_map<int, size_t> num_samples_;
  std::unordered_map<int, float> values_;
  std::unordered_map<int, float> actual_rates_;
};
} // namespace espp
