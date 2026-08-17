#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <system_error>

#include "base_component.hpp"

namespace espp {

/**
 * @brief Native USB CDC-ACM transport built on ESP-IDF's `esp_tinyusb` managed
 *        component and the ESP32-S3 / -S2 / -P4 USB-OTG peripheral.
 *
 * @details `espp::UsbCdc` presents a single dedicated CDC-ACM (virtual serial
 * port) interface on the native USB peripheral with a *configurable* VID/PID and
 * manufacturer / product / serial strings. This lets a device advertise its own
 * identifiers (e.g. ODrive-like) on a link that is completely separate from the
 * ESP console (which normally rides the built-in USB-Serial-JTAG peripheral or a
 * UART). Incoming bytes are delivered to a user callback and outgoing bytes are
 * sent via write().
 *
 * The class is a thin, idiomatic espp wrapper: it does not throw, reports
 * initialization failures via `std::error_code`, and marshals the TinyUSB RX
 * callback (which runs in the TinyUSB device task context) into the user's
 * receive callback.
 *
 * @note Only one instance per CDC port should be created. USB-OTG is only
 *       available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
 *
 * @note The receive callback is invoked from the TinyUSB device task. Keep it
 *       short and non-blocking; it is safe to call write() from within it.
 *
 * \section usb_cdc_ex1 UsbCdc Example
 * \snippet usb_cdc_example.cpp usb_cdc_example
 */
class UsbCdc : public BaseComponent {
public:
  /**
   * @brief Callback invoked with received bytes.
   * @param data Span of received bytes (valid only for the duration of the call).
   */
  using receive_callback_fn = std::function<void(std::span<const uint8_t> data)>;

  /**
   * @brief Configuration for the UsbCdc transport.
   */
  struct Config {
    uint16_t vid{0x1209};                /**< USB Vendor ID advertised in the device descriptor.
                                              Defaults to the pid.codes VID used by ODrive. */
    uint16_t pid{0x0d32};                /**< USB Product ID advertised in the device descriptor.
                                              Defaults to an ODrive-like PID. */
    std::string manufacturer{"espp"};    /**< Manufacturer string descriptor. */
    std::string product{"espp USB CDC"}; /**< Product string descriptor. */
    std::string serial_number{"000000000001"}; /**< Serial number string descriptor. */
    std::string interface_name{"espp CDC"};    /**< CDC interface string descriptor. */
    receive_callback_fn on_receive{nullptr};   /**< Callback invoked with received bytes. May be
                                                    set/replaced later via set_receive_callback(). */
    size_t rx_chunk_size{64}; /**< Size of the buffer used to drain the CDC RX FIFO per read. */
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Construct a UsbCdc transport. Does not touch hardware until
   *        initialize() is called.
   * @param config Configuration parameters.
   */
  explicit UsbCdc(const Config &config);

  /**
   * @brief Uninstalls the CDC-ACM interface and TinyUSB driver if initialized.
   */
  ~UsbCdc();

  // Non-copyable, non-movable (holds a stable `this` used by the C callback).
  UsbCdc(const UsbCdc &) = delete;
  UsbCdc &operator=(const UsbCdc &) = delete;

  /**
   * @brief Install the TinyUSB driver and initialize the CDC-ACM interface using
   *        the configured descriptors / VID-PID / strings.
   * @param[out] ec Set on failure.
   * @return true on success, false otherwise (ec is set).
   * @note Safe to call once. Subsequent calls while already initialized are no-ops.
   */
  bool initialize(std::error_code &ec);

  /**
   * @brief Queue bytes for transmission over the CDC interface and flush.
   * @param data Bytes to send.
   * @param[out] ec Set on failure (e.g. not initialized).
   * @return true if all bytes were queued, false otherwise.
   * @note Uses a non-blocking flush; safe to call from the receive callback.
   */
  bool write(std::span<const uint8_t> data, std::error_code &ec);

  /**
   * @brief Convenience overload of write() that ignores errors.
   * @param data Bytes to send.
   * @return true if all bytes were queued, false otherwise.
   */
  bool write(std::span<const uint8_t> data);

  /**
   * @brief Set or replace the receive callback.
   * @param cb Callback to invoke with received bytes (may be nullptr to detach).
   */
  void set_receive_callback(const receive_callback_fn &cb);

  /**
   * @brief Whether initialize() has completed successfully.
   */
  bool is_initialized() const;

  /**
   * @brief Whether a USB host has opened (asserted DTR on) the CDC port.
   */
  bool is_connected() const;

  /**
   * @brief Internal: drain the CDC RX FIFO and dispatch to the receive callback.
   * @note Invoked from the TinyUSB device task via the C callback trampoline.
   *       Not intended to be called by application code.
   */
  void handle_rx();

private:
  struct Impl; // holds TinyUSB descriptors, kept alive for driver lifetime
  std::unique_ptr<Impl> impl_;

  Config config_;
  bool initialized_{false};

  std::mutex cb_mutex_;
  receive_callback_fn on_receive_;
};

} // namespace espp
