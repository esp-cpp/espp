#pragma once

#include <math.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "base_component.hpp"
#include "encoder_types.hpp"

namespace espp {
/**
 *  @brief Class providing ABI/Z encoder functionality using the pulse count
 *         hardware in the ESP chips. Can be configured to measure purely
 *         linear (EncoderType::LINEAR) position or rotational
 *         (EncoderType::ROTATIONAL) position.
 *
 * \section encoder_ex1 AbiEncoder (ROTATIONAL) Example
 * \snippet encoder_example.cpp abi encoder rotational example
 * \section encoder_ex2 AbiEncoder (LINEAR) Example
 * \snippet encoder_example.cpp abi encoder linear example
 */
template <EncoderType T = EncoderType::ROTATIONAL> class AbiEncoder : public BaseComponent {
public:
  struct Config {
    int a_gpio;     /**< GPIO number for the a channel pulse. */
    int b_gpio;     /**< GPIO number for the b channel pulse. */
    int i_gpio{-1}; /**< GPIO number for the index (I/Z) pulse). @note This is currently unused. */
    int16_t high_limit; /**< High limit for the hardware counter before it resets to 0. Lowering (to
                           zero) this value increases the number of interrupts / overflows of the
                           counter. */
    int16_t
        low_limit; /**< Low limit for the hardware counter before it resets to 0. Lowering (to zero)
                      this value increases the number of interrupts / overflows of the counter. */
    size_t counts_per_revolution{0}; /**< How many counts equate to a single revolution. @note this
                                        unused if the type is not EncoderType::ROTATIONAL. */
    size_t max_glitch_ns{1000};      /**< Max glitch witdth in nanoseconds that is ignored. 0 will
                                        disable the glitch filter. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity for the adc logger. */
  };

  /**
   * @brief Initialize the pulse count hardware for the ABI encoder.
   *
   * @note This does not start the pulse count hardware, for that you must
   *       call the start() method at least once.
   */
  template <EncoderType type = T>
  explicit AbiEncoder(const Config &config)
      : BaseComponent("AbiEncoder", config.log_level) {
    // we only care about counts_per_revolution if it is EncoderType::ROTATIONAL
    if constexpr (type == EncoderType::ROTATIONAL) {
      if (config.counts_per_revolution == 0) {
        logger_.warn("Configured as ROTATIONAL encoder, but provided a 0 counts_per_revolution, "
                     "setting CPR to config.high_limit {}",
                     config.high_limit);
        counts_per_revolution_ = config.high_limit;
      } else {
        counts_per_revolution_ = config.counts_per_revolution;
      }
    }
    init(config);
  }

  /**
   * @brief Stop the pulse count hardware then disable and delete the channels
   *        / units associated with this AbiEncoder.
   */
  ~AbiEncoder() {
    stop();
    esp_err_t err;
    err = pcnt_unit_disable(pcnt_unit_);
    if (err != ESP_OK) {
      logger_.error("Could not disable the unit: {} '{}'", err, esp_err_to_name(err));
    }
    err = pcnt_del_channel(pcnt_channel_a_);
    if (err != ESP_OK) {
      logger_.error("Could not delete channel a: {} '{}'", err, esp_err_to_name(err));
    }
    err = pcnt_del_channel(pcnt_channel_b_);
    if (err != ESP_OK) {
      logger_.error("Could not delete channel b: {} '{}'", err, esp_err_to_name(err));
    }
    err = pcnt_del_unit(pcnt_unit_);
    if (err != ESP_OK) {
      logger_.error("Could not delete the unit: {} '{}'", err, esp_err_to_name(err));
    }
  }

  /**
   * @brief Get the total count (including under/overflows) since it was
   *        created or clear()-ed last.
   * @return Total count as a signed integer.
   */
  int get_count() {
    int current = 0;
    pcnt_unit_get_count(pcnt_unit_, &current);
    return current + count_.load();
  }

  /**
   * @brief Get the total number of revolutions this ABI encoder has measured
   *        since it was created or clear()-ed last.
   * @note This is only available if the AbiEncoder is
   *       EncoderType::ROTATIONAL.
   * @return Number of revolutions, as a floating point number.
   */
  float get_revolutions() requires(T == EncoderType::ROTATIONAL) {
    auto raw = get_count();
    return (float)raw / (float)counts_per_revolution_;
  }

  /**
   * @brief Get the total number of radians this ABI encoder has measured
   *        since it was created or clear()-ed last.
   * @note This is only available if the AbiEncoder is
   *       EncoderType::ROTATIONAL.
   * @return Number of radians, as a floating point number.
   */
  float get_radians() requires(T == EncoderType::ROTATIONAL) {
    return get_revolutions() * 2.0f * M_PI;
  }

  /**
   * @brief Get the total number of degrees this ABI encoder has measured
   *        since it was created or clear()-ed last.
   * @note This is only available if the AbiEncoder is
   *       EncoderType::ROTATIONAL.
   * @return Number of degrees, as a floating point number.
   */
  float get_degrees() requires(T == EncoderType::ROTATIONAL) { return get_revolutions() * 360.0f; }

  /**
   * @brief Start the pulse count hardware.
   * @return True if it was successfully started.
   */
  bool start() {
    logger_.info("Starting");
    auto err = pcnt_unit_start(pcnt_unit_);
    if (err != ESP_OK) {
      logger_.error("Could not start: {} '{}'", err, esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /**
   * @brief Stop the pulse count hardware.
   * @note This does not clear the counter so calling start() later will allow
   *       it to pick up where it left off.
   * @return True if it was successfully stopped.
   */
  bool stop() {
    logger_.info("Stopping");
    auto err = pcnt_unit_stop(pcnt_unit_);
    if (err != ESP_OK) {
      logger_.error("Could not stop: {} '{}'", err, esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /**
   * @brief Clear the hardware counter and internal accumulator.
   * @return True if the pulse count hardware was cleared.
   */
  bool clear() {
    logger_.info("Clearing count");
    auto err = pcnt_unit_clear_count(pcnt_unit_);
    count_ = 0;
    if (err != ESP_OK) {
      logger_.error("Could clear count: {} '{}'", err, esp_err_to_name(err));
      return false;
    }
    return true;
  }

protected:
  static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata,
                            void *user_ctx) {
    // NOTE: this function runs in ISR context
    AbiEncoder *obj = (AbiEncoder *)user_ctx;
    if (obj) {
      // increment the counter by the watch point
      obj->count_ = obj->count_.load() + edata->watch_point_value;
    }
    // didn't wake up any high priority tasks
    return false;
  }

  void init(const Config &config) {
    esp_err_t err;
    logger_.info("Initializing");
    // create
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = config.high_limit;
    unit_config.low_limit = config.low_limit;
    err = pcnt_new_unit(&unit_config, &pcnt_unit_);
    if (err != ESP_OK) {
      logger_.error("Could not create new pcnt unit: {} '{}'", err, esp_err_to_name(err));
    }

    set_glitch_filter(config.max_glitch_ns);

    // configure channel a (edge on a, level on b)
    pcnt_chan_config_t a_config{};
    a_config.edge_gpio_num = config.a_gpio;
    a_config.level_gpio_num = config.b_gpio;
    err = pcnt_new_channel(pcnt_unit_, &a_config, &pcnt_channel_a_);
    if (err != ESP_OK) {
      logger_.error("Could not create channel a: {} '{}'", err, esp_err_to_name(err));
    }
    // configure channel b (edge on b, level on a)
    pcnt_chan_config_t b_config{};
    b_config.edge_gpio_num = config.b_gpio;
    b_config.level_gpio_num = config.a_gpio;
    err = pcnt_new_channel(pcnt_unit_, &b_config, &pcnt_channel_b_);
    if (err != ESP_OK) {
      logger_.error("Could not create channel a: {} '{}'", err, esp_err_to_name(err));
    }

    // now configure how the counter changes for edge transitions and level
    // actions
    pcnt_channel_set_edge_action(pcnt_channel_a_, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_channel_a_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(pcnt_channel_b_, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_channel_b_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // add watch points and register callbacks
    int watch_points[] = {config.low_limit, config.high_limit};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
      pcnt_unit_add_watch_point(pcnt_unit_, watch_points[i]);
    }
    pcnt_event_callbacks_t cbs = {.on_reach = &AbiEncoder::pcnt_on_reach};
    pcnt_unit_register_event_callbacks(pcnt_unit_, &cbs, this);

    // enable
    pcnt_unit_enable(pcnt_unit_);
    // clear the count
    clear();
    // ready to start
  }

  bool set_glitch_filter(uint32_t max_glitch_ns) {
    esp_err_t err;
    if (max_glitch_ns) {
      logger_.info("Setting glitch filter to {}ns", max_glitch_ns);
      pcnt_glitch_filter_config_t filter_config = {
          .max_glitch_ns = max_glitch_ns,
      };
      err = pcnt_unit_set_glitch_filter(pcnt_unit_, &filter_config);
    } else {
      logger_.info("Disabling glitch filter");
      err = pcnt_unit_set_glitch_filter(pcnt_unit_, nullptr);
    }
    if (err != ESP_OK) {
      logger_.error("Could not set glitch filter: {} '{}'", err, esp_err_to_name(err));
      return false;
    }
    return true;
  }

  std::atomic<int32_t> count_;
  std::atomic<size_t> counts_per_revolution_;
  pcnt_unit_handle_t pcnt_unit_{nullptr};
  pcnt_channel_handle_t pcnt_channel_a_{nullptr};
  pcnt_channel_handle_t pcnt_channel_b_{nullptr};
};
} // namespace espp
