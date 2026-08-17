#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>

#include "base_component.hpp"
#include "usb_device.hpp"

namespace espp {

/**
 * @brief Native USB CDC-ACM transport: a thin CDC-only preset over
 *        `espp::UsbDevice`.
 *
 * @details `espp::UsbCdc` presents a single dedicated CDC-ACM (virtual serial
 * port) interface on the native USB peripheral with a *configurable* VID/PID and
 * manufacturer / product / serial strings. It is kept for back-compatibility and
 * is implemented on top of the composable `espp::UsbDevice` (which can also add a
 * vendor-specific / WebUSB interface, HID, MSC, ...). For anything beyond a plain
 * serial port, prefer `espp::UsbDevice` directly.
 *
 * Incoming bytes are delivered to a user callback and outgoing bytes are sent via
 * write(). The class does not throw and reports initialization failures via
 * `std::error_code`.
 *
 * @note Only one `espp::UsbCdc` / `espp::UsbDevice` instance may exist at a time.
 *       USB-OTG is only available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
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
   */
  bool initialize(std::error_code &ec);

  /**
   * @brief Queue bytes for transmission over the CDC interface and flush.
   * @param data Bytes to send.
   * @param[out] ec Set on failure (e.g. not initialized).
   * @return true if all bytes were queued, false otherwise.
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

private:
  Config config_;
  std::unique_ptr<UsbDevice> device_;
};

} // namespace espp
