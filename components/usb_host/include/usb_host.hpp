#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "usb/hid_host.h" // usb_host_hid managed component (pulls in the usb host library)

#include "base_component.hpp"

namespace espp {

// Forward-declare the extern "C" trampolines (defined in usb_host.cpp inside
// `namespace espp`) so the in-class friend declarations below refer to these
// existing C-linkage declarations rather than introducing conflicting
// C++-linkage symbols. The USB HID host driver invokes them from its background
// task with a `void *arg` set to the owning UsbHost.
extern "C" void espp_usb_host_driver_event_cb(hid_host_device_handle_t handle,
                                              const hid_host_driver_event_t event, void *arg);
extern "C" void espp_usb_host_interface_event_cb(hid_host_device_handle_t handle,
                                                 const hid_host_interface_event_t event, void *arg);

/**
 * @brief Native-USB **host** built on the ESP-IDF USB Host library (`usb`) and
 *        the `usb_host_hid` class driver, for the ESP32-S2 / -S3 / -P4 USB-OTG
 *        peripheral acting as a host.
 *
 * @details `espp::UsbHost` is the counterpart to `espp::UsbDevice`: instead of
 * enumerating *as* a USB device, it drives the bus as a **host**, enumerates
 * attached devices, and exposes the **HID** class devices it finds (mice,
 * keyboards, gamepads, and vendor-specific HID devices such as an
 * `espp::WdiUsbPeripheral`). It owns the whole host-side lifecycle:
 *
 * - installs the USB Host library and runs its event-handling task,
 * - installs the HID class driver (with its own background task),
 * - on device attach, opens each HID interface and (optionally) starts receiving
 *   its **Input** reports, delivering them to a per-device callback,
 * - lets the application send **Output** reports (and issue the HID class
 *   Get/Set Report / Idle / Protocol control requests) back to a device,
 * - and cleans everything up on teardown.
 *
 * Report directions are named from the connected **device's** point of view (as
 * in the USB HID spec): an *Input* report is device→host (delivered to
 * `HidDevice`'s input callback), an *Output* report is host→device (sent with
 * `HidDevice::send_output_report()`). This is deliberately symmetric with
 * `espp::UsbDevice`'s HID function, so the two sides of a link (e.g. the two
 * roles of the `wdi` component) mirror each other.
 *
 * The class is idiomatic espp: it does not throw, reports failures via
 * `std::error_code`, and marshals the USB-host driver's C callbacks (which run
 * in the HID driver's background task) into per-device `std::function`s.
 *
 * @note Only one `espp::UsbHost` may exist at a time: the USB Host library and
 *       the HID class driver are global singletons. USB-OTG **host** mode is
 *       only available on the ESP32-S2, ESP32-S3 and ESP32-P4, and the board
 *       must be able to source VBUS to the attached device (a self-powered hub
 *       or a board with a VBUS switch); the host does not manage board power.
 *
 * @note Device-connected / disconnected / input-report callbacks are invoked
 *       from the HID driver's background task. Keep them short and non-blocking;
 *       it is safe to call `HidDevice::send_output_report()` and the other
 *       device methods from within them.
 *
 * \section usb_host_ex1 UsbHost (generic HID host) Example
 * \snippet usb_host_example.cpp usb_host_example
 */
class UsbHost : public BaseComponent {
public:
  /**
   * @brief A HID interface on a device connected to the host.
   *
   * Created by `UsbHost` when a HID device is attached; handed to the
   * application (as a `std::shared_ptr`) through the connect / disconnect
   * callbacks and `UsbHost::devices()`. Owns nothing itself -- the underlying
   * driver handle is owned by `UsbHost` -- and becomes inert once the device is
   * disconnected (methods then fail with `std::errc::no_such_device`).
   */
  class HidDevice {
  public:
    /// @brief Callback invoked with a raw HID Input report from the device.
    /// @param data The report bytes. For a device whose report descriptor uses
    ///        report IDs, byte 0 is the report ID (matching how
    ///        `espp::UsbDevice`'s HID receive callback delivers OUT reports).
    using input_callback_fn = std::function<void(std::span<const uint8_t> data)>;

    /// @brief Device descriptor identity (VID/PID + string descriptors).
    struct Info {
      uint16_t vid{0};             ///< idVendor
      uint16_t pid{0};             ///< idProduct
      std::string manufacturer{};  ///< iManufacturer string (UTF-8)
      std::string product{};       ///< iProduct string (UTF-8)
      std::string serial_number{}; ///< iSerialNumber string (UTF-8)
    };

    /// @brief HID interface parameters.
    struct Params {
      uint8_t address{0};          ///< USB device address
      uint8_t interface_number{0}; ///< bInterfaceNumber of this HID interface
      uint8_t sub_class{0};        ///< bInterfaceSubClass (1 = boot interface)
      uint8_t protocol{0};         ///< bInterfaceProtocol (1 = keyboard, 2 = mouse, 0 = none)
    };

    /// @brief The identity of the connected device.
    Info info() const;
    /// @brief The parameters of this HID interface.
    Params params() const;

    /// @brief The device's HID report descriptor (a copy).
    /// @return The raw report-descriptor bytes (empty if unavailable). A copy is
    ///         returned rather than a view into driver-owned memory, so it stays
    ///         valid even if the device disconnects concurrently.
    std::vector<uint8_t> report_descriptor() const;

    /// @brief Install the callback invoked with each Input report.
    void set_input_callback(input_callback_fn cb);

    /// @brief Start receiving Input reports (called automatically on open when
    ///        `Config::auto_start` is set).
    bool start(std::error_code &ec);
    /// @brief Stop receiving Input reports.
    bool stop(std::error_code &ec);

    /// @brief Send a HID **Output** report to the device (host→device).
    /// @param report_id The report ID (0 if the descriptor is not report-ID'd).
    /// @param data The report payload (without the report-ID byte).
    /// @param ec Set on failure.
    /// @return true on success.
    bool send_output_report(uint8_t report_id, std::span<const uint8_t> data, std::error_code &ec);

    /// @brief Request a report from the device (HID class Get_Report).
    /// @param report_type One of HID_REPORT_TYPE_INPUT / _OUTPUT / _FEATURE.
    /// @param report_id The report ID.
    /// @param buffer Buffer that receives the report.
    /// @param out_length Number of bytes written into @p buffer.
    /// @param ec Set on failure.
    bool get_report(uint8_t report_type, uint8_t report_id, std::span<uint8_t> buffer,
                    size_t &out_length, std::error_code &ec);

    /// @brief Set the device's idle rate (HID class Set_Idle).
    bool set_idle(uint8_t duration, uint8_t report_id, std::error_code &ec);
    /// @brief Set the device's HID protocol (boot vs report; HID class Set_Protocol).
    bool set_protocol(hid_report_protocol_t protocol, std::error_code &ec);

    /// @brief Whether the device is still connected/usable.
    bool is_connected() const { return connected_.load(); }

    /// @brief The underlying driver handle (for advanced use).
    hid_host_device_handle_t handle() const { return handle_; }

  private:
    friend class UsbHost;
    HidDevice(hid_host_device_handle_t handle, size_t rx_buffer_size)
        : handle_(handle)
        , rx_buffer_(rx_buffer_size) {}

    // Called by UsbHost (in the driver task) when the interface reports input.
    void deliver_input();
    void mark_disconnected() { connected_.store(false); }

    hid_host_device_handle_t handle_{nullptr};
    std::atomic<bool> connected_{true};
    std::atomic<bool> started_{false};
    mutable std::mutex cb_mutex_;
    input_callback_fn on_input_{nullptr};
    // Fixed-size scratch for the current Input report. Sized from
    // Config::max_input_report_size; a report longer than this is truncated (the
    // driver copies at most this many bytes), so raise it if your device sends
    // larger reports.
    std::vector<uint8_t> rx_buffer_;
  };

  /// @brief Callback invoked when a HID device is connected / disconnected.
  using device_callback_fn = std::function<void(const std::shared_ptr<HidDevice> &device)>;

  /// @brief Predicate deciding whether to open a newly attached HID interface.
  ///        Return false to ignore it (no callbacks, not listed in devices()).
  using open_filter_fn =
      std::function<bool(const HidDevice::Info &info, const HidDevice::Params &params)>;

  /// @brief Configuration for the USB host.
  struct Config {
    device_callback_fn on_device_connected{nullptr};    ///< a HID device attached and opened
    device_callback_fn on_device_disconnected{nullptr}; ///< a HID device detached
    open_filter_fn should_open{nullptr}; ///< optional filter (default: open every HID interface)
    bool auto_start{true};        ///< start receiving Input reports as soon as a device opens
    size_t task_stack_size{4096}; ///< stack for the USB-host-library event task
    size_t task_priority{5};      ///< priority of the USB-host-library event task
    int task_core_id{-1};         ///< core for the host tasks (-1 = no affinity)
    /// @brief Per-device Input-report buffer size. A report larger than this is
    ///        truncated (the driver copies at most this many bytes); raise it if
    ///        your device sends larger reports. 64 covers full-speed HID.
    size_t max_input_report_size{64};
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  /// @brief Construct a USB host. Call initialize() to actually install the stack.
  explicit UsbHost(const Config &config);

  /// @brief Uninstall the stack (if still installed).
  ~UsbHost();

  UsbHost(const UsbHost &) = delete;
  UsbHost &operator=(const UsbHost &) = delete;

  /// @brief Install the USB Host library + HID class driver and start the tasks.
  /// @param ec Set on failure.
  /// @return true on success.
  bool initialize(std::error_code &ec);

  /// @brief Uninstall the HID class driver + USB Host library and stop the tasks.
  /// @param ec Set on failure.
  /// @return true on success.
  bool deinitialize(std::error_code &ec);

  /// @brief Whether the host stack is installed.
  bool is_initialized() const { return initialized_.load(); }

  /// @brief Snapshot of the currently connected (opened) HID devices.
  std::vector<std::shared_ptr<HidDevice>> devices() const;

private:
  friend void espp_usb_host_driver_event_cb(hid_host_device_handle_t, const hid_host_driver_event_t,
                                            void *);
  friend void espp_usb_host_interface_event_cb(hid_host_device_handle_t,
                                               const hid_host_interface_event_t, void *);

  // Trampoline targets (run in the HID driver's background task).
  void on_driver_event(hid_host_device_handle_t handle, hid_host_driver_event_t event);
  void on_interface_event(hid_host_device_handle_t handle, hid_host_interface_event_t event);

  // The USB Host library event-handling loop (own task).
  static void lib_task_trampoline(void *arg);
  void lib_task();
  // Stop + join the lib task: signal it, unblock its event wait, and wait
  // (bounded) for it to actually exit before the library is uninstalled.
  void stop_lib_task();

  static HidDevice::Info read_info(hid_host_device_handle_t handle);
  static HidDevice::Params read_params(hid_host_device_handle_t handle);

  Config config_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> lib_task_run_{false};
  std::atomic<bool> lib_task_done_{false}; // set by the lib task as it exits (join signal)
  void *lib_task_handle_{nullptr}; // TaskHandle_t (kept type-erased to avoid a public FreeRTOS dep)

  mutable std::mutex devices_mutex_;
  std::map<hid_host_device_handle_t, std::shared_ptr<HidDevice>> devices_;
};

} // namespace espp
