#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <optional>
#include <vector>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"

#include "esp_idf_version.h"
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

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
 * @note The available conversion modes, sample frequency range, and
 *       throughput depend on the chip. The aggregate sample frequency
 *       (sample_rate_hz * number of channels) must be within
 *       [SOC_ADC_SAMPLE_FREQ_THRES_LOW, SOC_ADC_SAMPLE_FREQ_THRES_HIGH] for
 *       the chip (e.g. 20 kHz - 2 MHz on ESP32; 611 Hz - 83.3 kHz on
 *       ESP32-S3 / C3 / C6 / P4 and other newer chips). Some chips (ESP32,
 *       ESP32-C3, ESP32-S3) only support continuous (DMA) mode on ADC unit
 *       1, and on ESP32-P4 ADC2 continuous conversions currently produce
 *       all-zero samples with esp-idf (oneshot on ADC2 works fine), so
 *       prefer ADC1 there as well.
 *
 * @note If a channel could not be calibrated (e.g. the calibration eFuse is
 *       not burnt), \c get_mv() returns the filtered raw value for that
 *       channel instead of millivolts (matching OneshotAdc's behavior).
 *
 * @warning On ESP32-P4 (verified on hardware with esp-idf v6.0.1),
 *          initializing and then deleting the oneshot ADC driver (e.g. a
 *          destructed espp::OneshotAdc) before starting continuous mode
 *          causes all continuous conversions to produce all-zero samples,
 *          on either unit. If you need both drivers on the P4, create the
 *          ContinuousAdc first, or keep the OneshotAdc alive.
 *
 * \section adc_continuous_ex1 Continuous ADC Example
 * \snippet adc_example.cpp continuous adc example
 */
class ContinuousAdc : public espp::BaseComponent {
public:
  /**
   *  @brief Configure the sample rate (globally applied to each channel),
   *  select the number of channels, and optionally the conversion mode.
   */
  struct Config {
    size_t sample_rate_hz;           /**< Samples per second to read from each channel. */
    std::vector<AdcConfig> channels; /**< Channels to read from, with associated attenuations. */
    adc_digi_convert_mode_t convert_mode{
        static_cast<adc_digi_convert_mode_t>(0)}; /**< Conversion mode (unit 1, unit 2,
                                 alternating, or both). Support depends on the chip. Leave at the
                                 default (0) to derive it automatically from the units used by \c
                                 channels. */
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
    previous_timestamp_ = std::chrono::high_resolution_clock::now();
    task_->start();
  }

  /**
   * @brief Stop, deinit, and destroy the adc reader.
   */
  ~ContinuousAdc() {
    // stop the hardware first, so the conversion-done ISR (which references
    // the task we are about to stop) can no longer fire
    stop();
    // wake the task if it is blocked waiting for a conversion notification
    // (its wait is bounded, so this just speeds up the shutdown), then stop
    // it
    if (task_handle_) {
      xTaskNotifyGive(task_handle_);
    }
    task_->stop();
    ESP_ERROR_CHECK(adc_continuous_deinit(handle_));
    // clean up the calibration data
    for (auto cali_handle : cali_handles_) {
      if (cali_handle == nullptr) {
        continue;
      }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(cali_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(cali_handle));
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
   * @note If the channel could not be calibrated, the filtered raw value is
   *       returned instead.
   */
  std::optional<float> get_mv(const AdcConfig &config) {
    if (!running_) {
      return {};
    }
    auto index = get_index(config);
    if (index == INVALID_INDEX) {
      return {};
    }
    std::lock_guard<std::mutex> lock{data_mutex_};
    return values_[index];
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
    auto index = get_index(config);
    if (index == INVALID_INDEX) {
      return {};
    }
    std::lock_guard<std::mutex> lock{data_mutex_};
    return actual_rates_[index];
  }

protected:
  // Unique id for a (unit, channel) pair. 32 is larger than the number of
  // channels on any ESP32 chip, so the id is unique across units.
  static constexpr int MAX_IDS = 2 * 32;
  static constexpr uint8_t INVALID_INDEX = 0xFF;

  static constexpr int get_id(adc_unit_t unit, adc_channel_t channel) {
    return static_cast<int>(unit) * 32 + static_cast<int>(channel);
  }

  // Get the dense index (0 .. num_channels_ - 1) for a (unit, channel) pair,
  // or INVALID_INDEX if it was not configured.
  uint8_t get_index(adc_unit_t unit, adc_channel_t channel) const {
    int id = get_id(unit, channel);
    if (id < 0 || id >= MAX_IDS) {
      return INVALID_INDEX;
    }
    return id_to_index_[id];
  }

  uint8_t get_index(const AdcConfig &config) const {
    return get_index(config.unit, config.channel);
  }

  bool update_task(std::mutex &, std::condition_variable &, bool &) {
    task_handle_ = xTaskGetCurrentTaskHandle();
    // wait (bounded, so that stopping the task never blocks forever) until
    // conversion results are ready; we will be notified by the registered
    // conversion-done callback
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) {
      // timed out; no data ready
      return false; // don't want to stop the task
    }
    auto current_timestamp = std::chrono::high_resolution_clock::now();
    std::fill(sums_.begin(), sums_.end(), 0);
    std::fill(num_samples_.begin(), num_samples_.end(), 0);
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
    for (uint32_t i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *p = reinterpret_cast<adc_digi_output_data_t *>(&result_data_[i]);
      uint8_t index = INVALID_INDEX;
      int parsed_unit = -1;
      int parsed_channel = -1;
      uint32_t data = 0;
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
      if (output_format_ == ADC_DIGI_OUTPUT_FORMAT_TYPE1) {
        // type1 data has no unit field; the unit is implied by the (single
        // unit) conversion mode
        parsed_unit = (conv_mode_ == ADC_CONV_SINGLE_UNIT_2) ? 1 : 0;
        parsed_channel = p->type1.channel;
        data = p->type1.data;
      }
#endif
#if !CONFIG_IDF_TARGET_ESP32
      if (output_format_ == ADC_DIGI_OUTPUT_FORMAT_TYPE2) {
        if (check_valid_data(p)) {
          parsed_unit = p->type2.unit;
          parsed_channel = p->type2.channel;
          data = p->type2.data;
        } else {
          logger_.warn_rate_limited("invalid data (unit {} channel {})!", (int)p->type2.unit,
                                    (int)p->type2.channel);
          continue;
        }
      }
#endif
      if (parsed_unit >= 0) {
        index = get_index((adc_unit_t)parsed_unit, (adc_channel_t)parsed_channel);
      }
      if (index == INVALID_INDEX) {
        // Not a channel we were configured with. The pattern table only
        // contains the configured channels, so getting here usually means the
        // chip encodes the output (unit / channel fields) differently than
        // expected - log it to aid debugging.
        logger_.warn_rate_limited("dropping sample with unexpected unit {} / channel {} (raw {})",
                                  parsed_unit, parsed_channel, data);
        continue;
      }
      sums_[index] += data;
      num_samples_[index]++;
    } // for()
    // measure elapsed time
    float elapsed_seconds =
        std::chrono::duration<float>(current_timestamp - previous_timestamp_).count();
    if (elapsed_seconds == 0) {
      // prevent divide by 0 later
      elapsed_seconds = 0.001f;
    }
    previous_timestamp_ = current_timestamp;
    // filter / update the measurements
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      for (size_t index = 0; index < num_channels_; index++) {
        float num_samples = num_samples_[index];
        if (num_samples > 0) {
          float sum = sums_[index];
          // note: the data collected above is uncalibrated (raw) values
          // TODO: use ESP32 hardware-accelerated filter
          // simple filter
          float average = sum / num_samples;
          if (cali_handles_[index] != nullptr) {
            // convert to millivolts
            int millivolts = 0;
            adc_cali_raw_to_voltage(cali_handles_[index], static_cast<int>(average), &millivolts);
            values_[index] = float(millivolts);
          } else {
            // no calibration available for this channel; report the raw value
            values_[index] = average;
          }
          actual_rates_[index] = num_samples / elapsed_seconds;
        }
        logger_.debug_rate_limited("CH {} - {}, {}, {:.2f}, {:.2f}", (int)configs_[index].channel,
                                   sums_[index], num_samples_[index], values_[index],
                                   actual_rates_[index]);
      }
    }

    // don't want to stop the task
    return false;
  }

#if !CONFIG_IDF_TARGET_ESP32
  static bool check_valid_data(const adc_digi_output_data_t *data) {
    const unsigned int unit = data->type2.unit;
    if (unit >= SOC_ADC_PERIPH_NUM)
      return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
      return false;
    return true;
  }
#endif

  void init(const std::vector<AdcConfig> &channels) {
    id_to_index_.fill(INVALID_INDEX);
    if (channels.empty()) {
      logger_.error("No channels provided!");
      return;
    }

    // validate the channels against the chip's capabilities
    bool has_unit_1 = false;
    bool has_unit_2 = false;
    for (const auto &conf : channels) {
      int unit = static_cast<int>(conf.unit);
      if (unit >= SOC_ADC_PERIPH_NUM) {
        logger_.error("{} invalid: this chip only has {} ADC unit(s)", conf, SOC_ADC_PERIPH_NUM);
      } else if (static_cast<int>(conf.channel) >= SOC_ADC_CHANNEL_NUM(unit)) {
        logger_.error("{} invalid: the unit only has {} channels", conf, SOC_ADC_CHANNEL_NUM(unit));
      }
#ifdef SOC_ADC_DIG_SUPPORTED_UNIT
      if (!SOC_ADC_DIG_SUPPORTED_UNIT(unit)) {
        logger_.error("{} invalid: this chip does not support continuous (DMA) mode on this unit",
                      conf);
      }
#endif
      has_unit_1 |= conf.unit == ADC_UNIT_1;
      has_unit_2 |= conf.unit == ADC_UNIT_2;
    }

#if CONFIG_IDF_TARGET_ESP32P4
    if (has_unit_2) {
      // empirically (esp-idf v6.0, M5Stack Tab5): the conversion frames
      // arrive at the configured rate but every sample is all-zero
      logger_.warn("ADC2 continuous (DMA) conversions on ESP32-P4 currently produce all-zero "
                   "samples with esp-idf (oneshot on ADC2 works fine); prefer ADC1 channels");
    }
#endif

    // derive the conversion mode from the channels if it was not explicitly
    // provided
    if (static_cast<int>(conv_mode_) == 0) {
      if (has_unit_1 && has_unit_2) {
        conv_mode_ = ADC_CONV_ALTER_UNIT;
      } else if (has_unit_2) {
        conv_mode_ = ADC_CONV_SINGLE_UNIT_2;
      } else {
        conv_mode_ = ADC_CONV_SINGLE_UNIT_1;
      }
      logger_.info("Derived conversion mode {} from the channel units", (int)conv_mode_);
    }

    // validate the aggregate sample frequency against the chip's supported
    // range
    size_t sample_freq_hz = sample_rate_hz_ * num_channels_;
    if (sample_freq_hz < SOC_ADC_SAMPLE_FREQ_THRES_LOW ||
        sample_freq_hz > SOC_ADC_SAMPLE_FREQ_THRES_HIGH) {
      logger_.error("Aggregate sample frequency {} Hz (sample_rate_hz * number of channels) is "
                    "outside this chip's supported range [{}, {}] Hz",
                    sample_freq_hz, SOC_ADC_SAMPLE_FREQ_THRES_LOW, SOC_ADC_SAMPLE_FREQ_THRES_HIGH);
    }

    adc_continuous_handle_cfg_t adc_config;
    memset(&adc_config, 0, sizeof(adc_config));
    adc_config.max_store_buf_size = std::max<size_t>(1024, window_size_bytes_ * 4);
    adc_config.conv_frame_size = window_size_bytes_;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle_));

    // Select the output format we will parse for this chip / mode (mirrors
    // the logic inside the esp_adc adc_continuous driver)
#if CONFIG_IDF_TARGET_ESP32
    // ESP32 only supports ADC1 DMA mode with type1 output
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
#else
    // all other chips (S3, C2, C3, C6, H2, P4, ...) only support type 2
    output_format_ = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#endif
    adc_continuous_config_t dig_cfg;
    memset(&dig_cfg, 0, sizeof(dig_cfg));
    dig_cfg.sample_freq_hz = sample_freq_hz;
    dig_cfg.conv_mode = conv_mode_;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    // Older IDF versions require the output format in the config; from IDF
    // 6.0 the field is deprecated and the driver derives the format itself
    // (with the same logic as output_format_ above, which we still need for
    // parsing the results).
    dig_cfg.format = output_format_;
#endif

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX];
    memset(&adc_pattern[0], 0, sizeof(adc_digi_pattern_config_t) * SOC_ADC_PATT_LEN_MAX);
    dig_cfg.pattern_num = num_channels_;

    // allocate the (dense) per-channel data, indexed by the position of the
    // channel in the configured channel list
    cali_handles_.resize(num_channels_, nullptr);
    sums_.resize(num_channels_, 0);
    num_samples_.resize(num_channels_, 0);
    values_.resize(num_channels_, 0);
    actual_rates_.resize(num_channels_, 0);

    for (size_t i = 0; i < num_channels_; i++) {
      auto conf = channels[i];
      adc_pattern[i].atten = conf.attenuation;
      adc_pattern[i].channel = conf.channel;
      adc_pattern[i].unit = conf.unit;
      adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

      logger_.info("adc_pattern[{}].atten is 0x{:02x}", i, (int)adc_pattern[i].atten);
      logger_.info("adc_pattern[{}].channel is 0x{:02x}", i, (int)adc_pattern[i].channel);
      logger_.info("adc_pattern[{}].unit is 0x{:02x}", i, (int)adc_pattern[i].unit);

      // map the (unit, channel) id to this dense index
      int id = get_id(conf.unit, conf.channel);
      if (id >= 0 && id < MAX_IDS) {
        id_to_index_[id] = i;
      }

      // create the calibration data
      auto calibrated = init_calibration(
          conf.unit, conf.attenuation, (adc_bitwidth_t)adc_pattern[i].bit_width, &cali_handles_[i]);
      logger_.info("adc_pattern[{}].cali_handle is calibrated: {}", i, calibrated);

      // save the channel
      configs_.push_back(conf);
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
  std::chrono::high_resolution_clock::time_point previous_timestamp_;

  std::atomic<bool> running_{false};

  std::vector<AdcConfig> configs_;
  // maps (unit, channel) ids to indices into the dense per-channel vectors
  // below (INVALID_INDEX if the channel was not configured)
  std::array<uint8_t, MAX_IDS> id_to_index_;
  // these vectors are indexed by the position of the channel in the
  // configured channel list
  std::vector<adc_cali_handle_t> cali_handles_;
  std::vector<uint32_t> sums_;
  std::vector<uint32_t> num_samples_;
  std::vector<float> values_;
  std::vector<float> actual_rates_;
};
} // namespace espp
