#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <esp_http_server.h>

#include "base_component.hpp"
#include "oneshot_adc.hpp"

namespace espp {
/**
 * @brief Remote Debug Component
 *
 * Provides a web-based interface for remote GPIO control and ADC monitoring.
 * Allows real-time control of GPIO pins and plotting of ADC values.
 *
 * Features:
 * - GPIO control (set high/low, read state, configure mode)
 * - ADC value reading and real-time plotting
 * - WebSocket support for live data streaming
 * - Configurable sampling rates
 * - Mobile-friendly responsive interface
 *
 * \section remote_debug_ex1 Remote Debug Example
 * \snippet remote_debug_example.cpp remote debug example
 */
class RemoteDebug : public BaseComponent {
public:
  /**
   * @brief GPIO configuration
   */
  struct GpioConfig {
    gpio_num_t pin;                     ///< GPIO pin number
    gpio_mode_t mode{GPIO_MODE_OUTPUT}; ///< Pin mode (input/output)
    std::string label{""};              ///< Optional label for UI
  };

  /**
   * @brief ADC configuration
   */
  struct AdcConfig {
    adc_unit_t unit;                    ///< ADC unit (ADC_UNIT_1 or ADC_UNIT_2)
    adc_channel_t channel;              ///< ADC channel
    adc_atten_t atten{ADC_ATTEN_DB_12}; ///< Attenuation (affects voltage range)
    std::string label{""};              ///< Optional label for UI
  };

  /**
   * @brief Configuration for remote debug
   */
  struct Config {
    std::vector<GpioConfig> gpios;                        ///< GPIO pins to expose
    std::vector<AdcConfig> adcs;                          ///< ADC channels to monitor
    uint16_t server_port{8080};                           ///< HTTP server port
    std::chrono::milliseconds adc_sample_rate{100};       ///< ADC sampling interval
    size_t adc_history_size{1000};                        ///< Number of ADC samples to keep
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /**
   * @brief Construct remote debug interface
   * @param config Configuration structure
   */
  explicit RemoteDebug(const Config &config);

  /**
   * @brief Destructor
   */
  ~RemoteDebug();

  /**
   * @brief Start the debug server
   * @return true if started successfully
   */
  bool start();

  /**
   * @brief Stop the debug server
   */
  void stop();

  /**
   * @brief Check if server is running
   * @return true if active
   */
  bool is_active() const { return is_active_; }

  /**
   * @brief Set GPIO output level
   * @param pin GPIO pin number
   * @param level Level (0=low, 1=high)
   * @return true if successful
   */
  bool set_gpio(gpio_num_t pin, int level);

  /**
   * @brief Read GPIO input level
   * @param pin GPIO pin number
   * @return Level (0 or 1), or -1 on error
   */
  int get_gpio(gpio_num_t pin);

  /**
   * @brief Read ADC value
   * @param unit ADC unit
   * @param channel ADC channel
   * @return Raw ADC value, or -1 on error
   */
  int read_adc(adc_unit_t unit, adc_channel_t channel);

protected:
  void init(const Config &config);
  bool start_server();
  void stop_server();
  void adc_sampling_task();

  // HTTP handlers
  static esp_err_t root_handler(httpd_req_t *req);
  static esp_err_t gpio_get_handler(httpd_req_t *req);
  static esp_err_t gpio_set_handler(httpd_req_t *req);
  static esp_err_t adc_get_handler(httpd_req_t *req);
  static esp_err_t adc_data_handler(httpd_req_t *req);

  std::string generate_html() const;
  std::string get_gpio_state_json() const;
  std::string get_adc_data_json() const;

  Config config_;
  httpd_handle_t server_{nullptr};
  adc_oneshot_unit_handle_t adc1_handle_{nullptr};
  adc_oneshot_unit_handle_t adc2_handle_{nullptr};

  std::map<gpio_num_t, GpioConfig> gpio_map_;
  std::map<std::pair<adc_unit_t, adc_channel_t>, AdcConfig> adc_map_;

  // ADC data storage
  struct AdcData {
    std::vector<int> values;
    std::vector<uint64_t> timestamps;
    size_t write_index{0};
  };
  std::map<std::pair<adc_unit_t, adc_channel_t>, AdcData> adc_data_;
  std::mutex adc_mutex_;

  std::atomic<bool> is_active_{false};
  std::atomic<bool> sampling_active_{false};
  std::unique_ptr<std::thread> sampling_thread_;
};
} // namespace espp
