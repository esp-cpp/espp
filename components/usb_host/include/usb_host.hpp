#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
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
#include "task.hpp"

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
 * **Threading model.** The HID class driver delivers its events on its own
 * background task, and that same task is the one that completes the driver's
 * synchronous control transfers (Set/Get Report, Set Protocol, ...). A control
 * transfer issued *from* that task can therefore never complete. `UsbHost`
 * handles this the way the ESP-IDF HID host example does: the driver task only
 * *enqueues* events (copying each Input report out of the driver's buffer, which
 * must happen inside the callback), and a dedicated **dispatch task** owned by
 * `UsbHost` opens/starts/closes devices and invokes every user callback. So it
 * is safe to call `HidDevice::send_output_report()` (and the other device
 * methods) from within the callbacks, and callbacks never stall the USB stack.
 * Events for a device are delivered in order (connected → inputs → disconnected).
 * Device methods may also be called from any application task; each device
 * serializes its driver calls internally.
 *
 * The class is idiomatic espp: it does not throw, reports failures via
 * `std::error_code`, and marshals the USB-host driver's C callbacks into
 * per-device `std::function`s.
 *
 * @note Only one `espp::UsbHost` may exist at a time: the USB Host library and
 *       the HID class driver are global singletons. USB-OTG **host** mode is
 *       only available on the ESP32-S2, ESP32-S3 and ESP32-P4, and the board
 *       must be able to source VBUS to the attached device (a self-powered hub
 *       or a board with a VBUS switch); the host does not manage board power.
 *
 * @note Callbacks run on the dispatch task. Keep them reasonably short: a
 *       callback that blocks delays every later event (and `deinitialize()`).
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
   * disconnected (methods then fail with `std::errc::no_such_device`; the
   * identity accessors keep returning the values captured at connect time).
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
      uint16_t vid{0}; ///< idVendor
      uint16_t pid{0}; ///< idProduct
      std::string
          manufacturer{};    ///< iManufacturer string (UTF-8, converted from the device's UTF-16)
      std::string product{}; ///< iProduct string (UTF-8, converted from the device's UTF-16)
      std::string
          serial_number{}; ///< iSerialNumber string (UTF-8, converted from the device's UTF-16)
    };

    /// @brief HID interface parameters.
    struct Params {
      uint8_t address{0};          ///< USB device address
      uint8_t interface_number{0}; ///< bInterfaceNumber of this HID interface
      uint8_t sub_class{0};        ///< bInterfaceSubClass (1 = boot interface)
      uint8_t protocol{0};         ///< bInterfaceProtocol (1 = keyboard, 2 = mouse, 0 = none)
    };

    /// @brief The identity of the connected device (captured at connect time,
    ///        so it stays valid after a disconnect).
    const Info &info() const { return info_; }
    /// @brief The parameters of this HID interface (captured at connect time).
    const Params &params() const { return params_; }

    /// @brief The device's HID report descriptor (captured at connect time; a
    ///        copy owned by this object, not a view into driver memory).
    const std::vector<uint8_t> &report_descriptor() const { return report_descriptor_; }

    /// @brief Install the callback invoked with each Input report. Install it
    ///        from the connect callback: `UsbHost` invokes that *before* it
    ///        starts the device, so no report is missed.
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
    /// @param report_type The HID report type (HID_REPORT_TYPE_INPUT / _OUTPUT / _FEATURE).
    /// @param report_id The report ID.
    /// @param buffer Buffer that receives the report.
    /// @param out_length Number of bytes written into @p buffer.
    /// @param ec Set on failure.
    bool get_report(hid_report_type_t report_type, uint8_t report_id, std::span<uint8_t> buffer,
                    size_t &out_length, std::error_code &ec);

    /// @brief Set the device's idle rate (HID class Set_Idle).
    bool set_idle(uint8_t duration, uint8_t report_id, std::error_code &ec);
    /// @brief Set the device's HID protocol (boot vs report; HID class Set_Protocol).
    bool set_protocol(hid_report_protocol_t protocol, std::error_code &ec);

    /// @brief Whether the device is still connected/usable.
    bool is_connected() const { return connected_.load(); }

    /// @brief The underlying driver handle (for advanced use; only valid while
    ///        is_connected()).
    hid_host_device_handle_t handle() const { return handle_; }

  private:
    friend class UsbHost;
    HidDevice(hid_host_device_handle_t handle, Info info, Params params,
              std::vector<uint8_t> report_descriptor)
        : handle_(handle)
        , info_(std::move(info))
        , params_(std::move(params))
        , report_descriptor_(std::move(report_descriptor)) {}

    // Called by UsbHost (on the dispatch task) with a copy of an Input report.
    void deliver_input(std::span<const uint8_t> data);
    // Called by UsbHost to retire the device: marks it inert and closes the
    // driver handle, serialized against any in-flight driver call.
    void retire();

    hid_host_device_handle_t handle_{nullptr};
    const Info info_;
    const Params params_;
    const std::vector<uint8_t> report_descriptor_;
    std::atomic<bool> connected_{true};
    std::atomic<bool> started_{false};
    // Serializes every driver call made through this object against the close
    // performed on disconnect, so a control transfer in flight on an app task
    // can't race the driver freeing the interface.
    mutable std::mutex io_mutex_;
    mutable std::mutex cb_mutex_;
    input_callback_fn on_input_{nullptr};
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
    bool auto_start{true};            ///< start receiving Input reports as soon as a device opens
    size_t task_priority{5};          ///< priority of the internal tasks
    int task_core_id{-1};             ///< core for the internal tasks (-1 = no affinity)
    size_t lib_task_stack_size{4096}; ///< stack for the USB-host-library event task
    size_t hid_task_stack_size{4096}; ///< stack for the HID class driver's task (it only enqueues)
    /// @brief Stack for the dispatch task that runs the user callbacks (size it
    ///        for what your callbacks do -- logging with fmt, protocol work, ...).
    size_t dispatch_task_stack_size{6 * 1024};
    /// @brief Per-device Input-report copy size. A report larger than this is
    ///        truncated (the driver copies at most this many bytes); raise it if
    ///        your device sends larger reports. 64 covers full-speed HID.
    size_t max_input_report_size{64};
    /// @brief Hard bound on queued-but-undispatched events. When full, a new Input
    ///        report is dropped, and a lifecycle event evicts the oldest queued Input
    ///        report to make room, so the queue never blocks the USB stack and
    ///        lifecycle events are never lost. Drops are counted and logged at a
    ///        rate-limited cadence.
    size_t max_queued_events{32};
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
  ///        Attached devices are closed (their disconnect callbacks fire, on the
  ///        calling task) and the root port is powered down so the driver can
  ///        release them. Must not be called from within a `UsbHost` callback.
  /// @param ec Set on failure. If any step of the teardown fails (a device the
  ///        driver cannot release, or the library refusing to uninstall) the host
  ///        stays initialized (is_initialized() remains true) and false is
  ///        returned, rather than tearing down under a live driver. Destroying
  ///        a UsbHost in that state aborts (see the destructor).
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

  // An event queued by the HID driver task for the dispatch task.
  struct Event {
    enum class Type { NewDevice, Input, Disconnected } type;
    hid_host_device_handle_t handle{nullptr};
    std::vector<uint8_t> data{}; // Input: the report bytes (copied on the driver task)
  };

  // Trampoline targets: run on the HID driver's background task. They only
  // enqueue (plus the Input-report copy that must happen inside the callback).
  void on_driver_event(hid_host_device_handle_t handle, hid_host_driver_event_t event);
  void on_interface_event(hid_host_device_handle_t handle, hid_host_interface_event_t event);
  void enqueue(Event &&ev);

  // The dispatch task: drains the queue and does the real work / user callbacks.
  bool dispatch_task_fn(std::mutex &m, std::condition_variable &cv);
  void handle_new_device(hid_host_device_handle_t handle);
  void handle_input(hid_host_device_handle_t handle, std::span<const uint8_t> data);
  void handle_disconnected(hid_host_device_handle_t handle);
  std::shared_ptr<HidDevice> find_device(hid_host_device_handle_t handle) const;
  void stop_dispatch_task();

  // The USB Host library event-handling loop (own task).
  bool lib_task_fn(std::mutex &m, std::condition_variable &cv);
  void stop_lib_task();

  static HidDevice::Info read_info(hid_host_device_handle_t handle);
  static HidDevice::Params read_params(hid_host_device_handle_t handle);

  Config config_;
  std::atomic<bool> initialized_{false};

  // USB Host library task.
  std::atomic<bool> lib_task_run_{false};
  std::unique_ptr<espp::Task> lib_task_;

  // Event queue (driver task -> dispatch task) + dispatch task.
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Event> queue_;
  uint32_t dropped_inputs_{0}; // guarded by queue_mutex_; rate-limits the drop log
  std::atomic<bool> dispatch_run_{false};
  std::unique_ptr<espp::Task> dispatch_task_;

  mutable std::mutex devices_mutex_;
  std::map<hid_host_device_handle_t, std::shared_ptr<HidDevice>> devices_;
};

} // namespace espp
