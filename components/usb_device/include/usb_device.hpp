#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "base_component.hpp"

namespace espp {

/**
 * @brief Composable native-USB device built on ESP-IDF's `esp_tinyusb` managed
 *        component and the ESP32-S3 / -S2 / -P4 USB-OTG peripheral.
 *
 * @details `espp::UsbDevice` assembles a USB device from a *set of selectable
 * functions* rather than hard-coding a single class. Today it can enable a
 * **CDC-ACM** (virtual serial port) function and/or a **vendor-specific**
 * function (bInterfaceClass 0xFF, one bulk IN + one bulk OUT) that optionally
 * advertises **WebUSB** + **MS OS 2.0** descriptors so a browser can talk to it
 * driverlessly, and/or a **HID** function (one interrupt IN, optionally one
 * interrupt OUT) carrying an application-supplied report descriptor (e.g. a
 * gamepad built with the espp `hid-rp` component). Interface numbers, endpoint
 * addresses and string indices are allocated *sequentially* as functions are
 * enabled, and the device checks the result against the USB-OTG endpoint budget
 * (reporting an error via `std::error_code` if it is exceeded).
 *
 * The design also leaves room for an **MSC** function to be added later without
 * changing the descriptor-building model (see `MscFunction` below and the
 * endpoint-budget table in the README).
 *
 * The VID/PID and manufacturer / product / serial strings are configurable so a
 * device can advertise its own identifiers (e.g. ODrive-like) on a link that is
 * completely separate from the ESP console (which normally rides the built-in
 * USB-Serial-JTAG peripheral or a UART).
 *
 * The class is idiomatic espp: it does not throw, reports initialization
 * failures via `std::error_code`, and marshals the TinyUSB RX callbacks (which
 * run in the TinyUSB device task context) into per-function user callbacks.
 *
 * @note Only one `espp::UsbDevice` (or `espp::UsbCdc`) may exist at a time; the
 *       TinyUSB device stack, the vendor RX routing and the BOS/WebUSB control
 *       requests are all global. USB-OTG is only available on the ESP32-S2,
 *       ESP32-S3 and ESP32-P4 targets.
 *
 * @note Receive callbacks are invoked from the TinyUSB device task. Keep them
 *       short and non-blocking; it is safe to call the matching write() from
 *       within them.
 *
 * \section usb_device_ex1 UsbDevice (composite CDC + Vendor/WebUSB) Example
 * \snippet usb_cdc_example.cpp usb_cdc_example
 */
class UsbDevice : public BaseComponent {
public:
  /**
   * @brief Callback invoked with received bytes.
   * @param data Span of received bytes (valid only for the duration of the call).
   */
  using receive_callback_fn = std::function<void(std::span<const uint8_t> data)>;

  /**
   * @brief CDC-ACM (virtual serial port) function.
   *
   * Consumes 1 interrupt IN (notification) + 1 bulk IN + 1 bulk OUT endpoint
   * (across two USB interfaces joined by an IAD).
   */
  struct CdcFunction {
    std::string interface_name{"espp CDC"};  /**< CDC interface string descriptor. */
    receive_callback_fn on_receive{nullptr}; /**< Callback invoked with received bytes. */
    size_t rx_chunk_size{64}; /**< Buffer size used to drain the CDC RX FIFO per read. */
  };

  /**
   * @brief Vendor-specific function (bInterfaceClass 0xFF) carrying a raw byte
   *        stream over one bulk IN + one bulk OUT endpoint.
   *
   * When `webusb` is true a BOS descriptor advertising the WebUSB platform
   * capability (with `webusb_vendor_code` + landing-page index 1) and an MS OS
   * 2.0 platform capability (with `ms_os_vendor_code`, so Windows binds WinUSB
   * automatically with no driver) is exposed, and the WebUSB URL / MS-OS-2.0
   * descriptor vendor control requests are answered.
   */
  struct VendorFunction {
    std::string interface_name{"espp Vendor"}; /**< Vendor interface string descriptor. */
    receive_callback_fn on_receive{nullptr};   /**< Callback invoked with received bytes. */
    size_t rx_chunk_size{64}; /**< Buffer size used to drain the vendor RX FIFO per read. */
    bool webusb{true}; /**< Advertise WebUSB + MS OS 2.0 descriptors for driverless access. */
    /**
     * @brief WebUSB landing-page URL. When `url_scheme` is 0 (http) or 1 (https)
     *        the URL must be given *without* a scheme (the scheme is prepended by
     *        the host from `url_scheme`). When `url_scheme` is 255 the URL must
     *        instead *include* its own scheme (e.g. "http://..."). Defaults to
     *        the espp docs-hosted board console + ESP flasher (scheme-less,
     *        https), a general-purpose Web Serial monitor and esptool-js flasher.
     * @note The descriptor length (3 + URL bytes) must fit a uint8_t, so the URL
     *       is limited to 252 bytes; `initialize()` rejects a longer URL.
     */
    std::string landing_page_url{"esp-cpp.github.io/espp/apps/board_console.html"};
    uint8_t url_scheme{1};         /**< 0 = http, 1 = https, 255 = URL includes its own scheme. */
    uint8_t webusb_vendor_code{1}; /**< bRequest used for the WebUSB URL control request. */
    uint8_t ms_os_vendor_code{
        2}; /**< bRequest used for the MS OS 2.0 descriptor control request. */
  };

  /**
   * @brief HID (Human Interface Device) function.
   *
   * A HID function consumes 1 interrupt IN endpoint (and optionally 1 interrupt
   * OUT if `has_out_endpoint` is set). It advertises the application-supplied
   * `report_descriptor` bytes (the TinyUSB HID class driver returns them from
   * `tud_hid_descriptor_report_cb`), and input reports are sent with
   * `UsbDevice::write_hid_report()`. The descriptor bytes are typically built
   * with the espp `hid-rp` component (e.g. `espp::GamepadInputReport`); the
   * component itself stays descriptor-bytes based and does not depend on hid-rp.
   *
   * Requires the TinyUSB HID class driver to be compiled in
   * (`CONFIG_TINYUSB_HID_COUNT` > 0, which defines `CFG_TUD_HID`); otherwise
   * enabling this function makes `initialize()` fail with
   * `std::errc::function_not_supported`.
   */
  struct HidFunction {
    std::string interface_name{"espp HID"};   /**< HID interface string descriptor. */
    std::vector<uint8_t> report_descriptor{}; /**< HID report descriptor bytes. */
    bool has_out_endpoint{false};             /**< Whether to allocate an interrupt OUT endpoint. */
    uint8_t poll_interval_ms{10};             /**< Interrupt IN polling interval (bInterval), ms. */
  };

  /**
   * @brief (Future) MSC (mass storage) function extension point. Not implemented yet.
   *
   * An MSC function consumes 1 bulk IN + 1 bulk OUT endpoint and requires SCSI +
   * storage callbacks (read10 / write10 / inquiry / capacity). Enabling it today
   * makes initialize() fail with `std::errc::function_not_supported`.
   */
  struct MscFunction {
    std::string interface_name{"espp MSC"};
    // Future: SCSI inquiry strings + read/write/capacity callbacks.
  };

  /**
   * @brief Configuration for the composable UsbDevice.
   */
  struct Config {
    uint16_t vid{0x1209}; /**< USB Vendor ID (defaults to the pid.codes VID used by ODrive). */
    uint16_t pid{0x0d32}; /**< USB Product ID (defaults to an ODrive-like PID). */
    std::string manufacturer{"espp"};          /**< Manufacturer string descriptor. */
    std::string product{"espp USB Device"};    /**< Product string descriptor. */
    std::string serial_number{"000000000001"}; /**< Serial number string descriptor. */

    std::optional<CdcFunction> cdc{};       /**< Enable a CDC-ACM function. */
    std::optional<VendorFunction> vendor{}; /**< Enable a vendor-specific / WebUSB function. */
    std::optional<HidFunction> hid{};       /**< Enable a HID function. */
    std::optional<MscFunction> msc{};       /**< (Future) enable an MSC function. */

    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Construct a UsbDevice. Does not touch hardware until initialize().
   * @param config Configuration parameters.
   */
  explicit UsbDevice(const Config &config);

  /**
   * @brief Uninstalls the enabled functions and the TinyUSB driver if initialized.
   */
  ~UsbDevice();

  // Non-copyable, non-movable (holds a stable `this` used by the C callbacks).
  UsbDevice(const UsbDevice &) = delete;
  UsbDevice &operator=(const UsbDevice &) = delete;

  /**
   * @brief Install the TinyUSB driver and initialize the enabled functions using
   *        the configured descriptors / VID-PID / strings.
   * @param[out] ec Set on failure (invalid config, endpoint budget exceeded,
   *                driver install failure, or unsupported function requested).
   * @return true on success, false otherwise (ec is set).
   */
  bool initialize(std::error_code &ec);

  /**
   * @brief Queue bytes for transmission over the CDC function and flush.
   * @param data Bytes to send.
   * @param[out] ec Set on failure (e.g. CDC not enabled / not initialized, or
   *        the TX FIFO could not accept all bytes - see note below).
   * @return true if all bytes were queued, false otherwise.
   * @note Same backpressure contract as write_vendor(): if the TX FIFO
   *       (CONFIG_TINYUSB_CDC_TX_BUFSIZE) fills mid-write, this call sleep-waits
   *       (bounded, 250 ms) for the TinyUSB task to drain it - EXCEPT when
   *       called from TinyUSB-callback context (e.g. from inside a receive
   *       callback, which runs on the TinyUSB task): there the drain can never
   *       happen while this call blocks, so writes are ALL-OR-NOTHING - if the
   *       whole frame does not fit in the FIFO up front, the call fails fast
   *       with `no_buffer_space` WITHOUT enqueueing any bytes (a partially-
   *       enqueued frame would poison the byte stream for the host). To reliably
   *       send frames larger than the TX FIFO in response to received data,
   *       queue the work to your own task rather than writing directly from the
   *       receive callback (or size the FIFO to hold a full frame).
   */
  bool write_cdc(std::span<const uint8_t> data, std::error_code &ec);

  /// @brief Convenience overload of write_cdc() that ignores errors.
  bool write_cdc(std::span<const uint8_t> data);

  /**
   * @brief Queue bytes for transmission over the vendor function and flush.
   * @param data Bytes to send.
   * @param[out] ec Set on failure (e.g. vendor not enabled / not initialized,
   *        or the TX FIFO could not accept all bytes - see note below).
   * @return true if all bytes were queued, false otherwise.
   * @note If the TX FIFO (CONFIG_TINYUSB_VENDOR_TX_BUFSIZE) fills mid-write,
   *       this call sleep-waits (bounded, 250 ms) for the TinyUSB task to
   *       drain it - EXCEPT when called from TinyUSB-callback context (e.g.
   *       from inside a receive callback, which runs on the TinyUSB task):
   *       there the drain can never happen while this call blocks, so writes
   *       are ALL-OR-NOTHING - if the whole frame does not fit in the FIFO up
   *       front, the call fails fast with `no_buffer_space` WITHOUT enqueueing
   *       any bytes (a partially-enqueued frame would poison the byte stream
   *       for the host). To reliably send frames larger than the TX FIFO in
   *       response to received data, queue the work to your own task rather
   *       than writing directly from the receive callback (or size the FIFO
   *       to hold a full frame).
   */
  bool write_vendor(std::span<const uint8_t> data, std::error_code &ec);

  /// @brief Convenience overload of write_vendor() that ignores errors.
  bool write_vendor(std::span<const uint8_t> data);

  /**
   * @brief Send a HID input report on the HID function's interrupt IN endpoint.
   * @param report_id HID report id (0 if the report descriptor has no report id;
   *        otherwise the id baked into the descriptor, e.g. 1 for the gamepad).
   * @param report Report payload bytes (without the report-id prefix).
   * @param[out] ec Set on failure (HID not enabled / not initialized, host not
   *        ready, or the HID class driver is not compiled in).
   * @return true if the report was queued for transmission, false otherwise.
   */
  bool write_hid_report(uint8_t report_id, std::span<const uint8_t> report, std::error_code &ec);

  /// @brief Convenience overload of write_hid_report() that ignores errors.
  bool write_hid_report(uint8_t report_id, std::span<const uint8_t> report);

  /// @brief Whether the HID function is enabled, mounted and ready to accept a
  ///        new input report (no report in flight).
  bool is_hid_ready() const;

  /// @brief Set or replace the CDC receive callback (nullptr to detach).
  void set_cdc_receive_callback(const receive_callback_fn &cb);

  /// @brief Set or replace the vendor receive callback (nullptr to detach).
  void set_vendor_receive_callback(const receive_callback_fn &cb);

  /// @brief Whether initialize() has completed successfully.
  bool is_initialized() const;

  /// @brief Whether the CDC function is enabled and a host has asserted DTR.
  bool is_cdc_connected() const;

  /// @brief Whether the vendor function is enabled and the device is mounted.
  bool is_vendor_connected() const;

  //
  // Internal: invoked from the TinyUSB device task via C trampolines / weak
  // overrides. Not intended to be called by application code.
  //

  /// @brief Internal: drain the CDC RX FIFO and dispatch to the CDC callback.
  void handle_cdc_rx();

  /// @brief Internal: dispatch received vendor bytes to the vendor callback.
  /// @param buffer When non-null (TinyUSB zero-copy RX variant, RX_BUFSIZE==0),
  ///        the just-received bytes to dispatch directly. When null (the FIFO
  ///        variant), the FIFO is drained via `tud_vendor_read()` instead.
  /// @param bufsize Number of bytes at @p buffer (0 when @p buffer is null).
  void handle_vendor_rx(const uint8_t *buffer = nullptr, size_t bufsize = 0);

  /// @brief Internal: pointer to the BOS descriptor bytes (nullptr if none).
  const uint8_t *bos_descriptor() const;

  /// @brief Internal: pointer to the MS OS 2.0 descriptor bytes (nullptr if none).
  const uint8_t *ms_os_20_descriptor(uint16_t &total_len) const;

  /// @brief Internal: pointer to the WebUSB URL descriptor bytes (nullptr if none).
  const uint8_t *webusb_url_descriptor(uint8_t &length) const;

  /// @brief Internal: pointer to the stored HID report descriptor bytes (nullptr
  ///        if the HID function is not enabled). Returned to the TinyUSB HID
  ///        class driver from `tud_hid_descriptor_report_cb`.
  const uint8_t *hid_report_descriptor() const;

  /// @brief Internal: config for the vendor control-request handler.
  const std::optional<VendorFunction> &vendor_config() const { return config_.vendor; }

  /// @brief Internal: the singleton instance handling the global USB callbacks.
  static UsbDevice *instance();

private:
  struct Impl; // holds TinyUSB descriptors, kept alive for driver lifetime
  std::unique_ptr<Impl> impl_;

  Config config_;
  std::atomic<bool> initialized_{false}; // read from the TinyUSB task via the write paths

  std::mutex cb_mutex_;
  receive_callback_fn on_cdc_receive_;
  receive_callback_fn on_vendor_receive_;

  // Preallocated RX scratch buffers (sized in initialize()) so the TinyUSB-task
  // RX handlers stay allocation-free (no heap churn on the hot path).
  std::vector<uint8_t> cdc_rx_buf_;
  std::vector<uint8_t> vendor_rx_buf_;
};

} // namespace espp
