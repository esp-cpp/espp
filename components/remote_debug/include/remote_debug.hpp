#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <driver/gpio.h>
#include <esp_http_server.h>

#include "base_component.hpp"
#include "oneshot_adc.hpp"
#include "timer.hpp"

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
  struct AdcChannelConfig {
    adc_channel_t channel;              ///< ADC channel
    adc_atten_t atten{ADC_ATTEN_DB_12}; ///< Attenuation (affects voltage range)
    std::string label{""};              ///< Optional label for UI
  };

  /**
   * @brief Configuration for remote debug
   */
  struct Config {
    std::string device_name{"ESP32 Device"};         ///< Device name shown in UI title
    std::vector<GpioConfig> gpios;                   ///< GPIO pins to expose
    std::vector<AdcChannelConfig> adc1_channels;     ///< ADC1 channels to monitor
    std::vector<AdcChannelConfig> adc2_channels;     ///< ADC2 channels to monitor
    uint16_t server_port{8080};                      ///< HTTP server port
    std::chrono::milliseconds adc_sample_rate{100};  ///< ADC sampling interval
    std::chrono::milliseconds gpio_update_rate{100}; ///< GPIO state update interval
    size_t adc_history_size{1000};                   ///< Number of ADC samples to keep
    size_t adc_batch_size{10};                       ///< Number of ADC samples to send per update
    size_t task_priority{5};                         ///< Priority for update tasks
    size_t task_stack_size{4096};                    ///< Stack size for update tasks
    bool enable_log_capture{false};                  ///< Enable stdout redirection to file
    std::string log_file_path{
        "debug.log"}; ///< Path to log file. Will be appended to espp::FileSystem::get_root_path().
    size_t max_log_size{100000};                          ///< Maximum log file size in bytes
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
   * @brief Read GPIO level
   * @param pin GPIO pin number
   * @return Level (0 or 1), or -1 on error
   */
  int get_gpio(gpio_num_t pin);

  /**
   * @brief Configure GPIO mode
   * @param pin GPIO pin number
   * @param mode GPIO mode (input/output)
   * @return true if successful
   */
  bool configure_gpio(gpio_num_t pin, gpio_mode_t mode);

protected:
  void init(const Config &config);
  bool start_server();
  void stop_server();
  void adc_sampling_task();
  void gpio_update_task();

  // HTTP handlers
  static esp_err_t root_handler(httpd_req_t *req);
  static esp_err_t gpio_get_handler(httpd_req_t *req);
  static esp_err_t gpio_set_handler(httpd_req_t *req);
  static esp_err_t gpio_config_handler(httpd_req_t *req);
  static esp_err_t adc_data_handler(httpd_req_t *req);
  static esp_err_t logs_handler(httpd_req_t *req);

  std::string generate_html() const;
  std::string get_gpio_state_json() const;
  std::string get_adc_data_json() const;
  std::string get_logs() const;
  void setup_log_redirection();
  void cleanup_log_redirection();

  Config config_;
  httpd_handle_t server_{nullptr};

  std::unique_ptr<OneshotAdc> adc1_;
  std::unique_ptr<OneshotAdc> adc2_;

  std::unordered_map<gpio_num_t, GpioConfig> gpio_map_;
  std::unordered_map<gpio_num_t, int> gpio_state_; // Cached GPIO states
  std::mutex gpio_mutex_;

  // ADC data storage - ring buffer implementation
  struct AdcData {
    std::vector<float> values;
    std::vector<uint64_t> timestamps;
    size_t write_index{0}; // Current write position in ring buffer
    size_t count{0};       // Number of valid samples (up to buffer size)
    adc_channel_t channel;
    std::string label;
  };
  std::vector<AdcData> adc1_data_;
  std::vector<AdcData> adc2_data_;
  std::mutex adc_mutex_;

  std::atomic<bool> is_active_{false};
  std::atomic<bool> sampling_active_{false};
  std::unique_ptr<Timer> adc_timer_;
  std::unique_ptr<Timer> gpio_timer_;

  // Log redirection
  FILE *log_file_{nullptr};
  FILE *original_stdout_{nullptr};
};
} // namespace espp

// for libfmt formatting of gpio_mode_t
template <> struct fmt::formatter<gpio_mode_t> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(const gpio_mode_t &mode, FormatContext &ctx) const {
    std::string mode_str;
    switch (mode) {
    case GPIO_MODE_INPUT:
      mode_str = "INPUT";
      break;
    case GPIO_MODE_OUTPUT:
      mode_str = "OUTPUT";
      break;
    case GPIO_MODE_INPUT_OUTPUT:
      mode_str = "INPUT_OUTPUT";
      break;
    case GPIO_MODE_DISABLE:
      mode_str = "DISABLE";
      break;
    default:
      mode_str = "UNKNOWN";
      break;
    }
    return fmt::formatter<std::string>::format(mode_str, ctx);
  }
};

// for libfmt formatting of gpio_num_t
template <> struct fmt::formatter<gpio_num_t> : fmt::formatter<int> {
  template <typename FormatContext> auto format(const gpio_num_t &pin, FormatContext &ctx) const {
    return fmt::formatter<int>::format(static_cast<int>(pin), ctx);
  }
};
