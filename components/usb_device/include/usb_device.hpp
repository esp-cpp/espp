#pragma once

#include <atomic>
#include <chrono>
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
#include "sd_protocol_types.h" // sdmmc_card_t, for MscMedium::sd_card
#include "tinyusb.h"           // for tinyusb_event_t (esp_tinyusb is already a REQUIRES dependency)
#include "xinput.hpp"          // X-Input (Xbox 360) gamepad state + descriptor helpers

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
 * It can also enable an **MSC** (mass storage) function exposing up to two
 * media -- an SD card and/or a wear-levelled FAT partition in flash -- as USB
 * drives. Each medium is owned by one side at a time: the application reads and
 * writes files through the VFS at its `base_path`, or the USB host sees the FAT
 * volume; ownership moves to the host when it mounts the device and back to the
 * application when it ejects or disconnects (see `MscFunction`).
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

// Forward-declare the extern "C" trampoline (defined in usb_device.cpp, inside
// `namespace espp`) so the in-class friend declaration below refers to this
// existing C-linkage declaration instead of introducing a conflicting
// C++-linkage espp::espp_usb_device_event_cb.
extern "C" void espp_usb_device_event_cb(tinyusb_event_t *event, void *arg);

class UsbDevice : public BaseComponent {
public:
  /**
   * @brief Callback invoked with received bytes.
   * @param data Span of received bytes (valid only for the duration of the call).
   */
  using receive_callback_fn = std::function<void(std::span<const uint8_t> data)>;

  /// @brief Callback for a device lifecycle event (mount / unmount). Invoked in
  ///        the TinyUSB device-task context.
  using event_callback_fn = std::function<void()>;

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

    /**
     * @brief Route the ESP console to this CDC interface once `initialize()`
     *        succeeds (equivalent to calling `route_console_to_cdc()` yourself).
     *
     * The native USB port is often handed to TinyUSB for a vendor / HID / XInput
     * interface, which on the ESP32-S3 means the console can no longer live on
     * USB-Serial-JTAG (it shares that USB PHY). Enabling this redirects the
     * console (stdout — `printf`, `ESP_LOG`, and `espp::Logger`'s `fmt::print` all
     * default there) to this CDC interface, so a single native USB cable carries
     * both the logs and the other interface(s). Writes are non-blocking and are
     * dropped when no host is draining the CDC endpoint.
     */
    bool route_console{false};

    /**
     * @brief When `route_console` (or `route_console_to_cdc()`) redirects the
     *        console, also keep writing it to the ORIGINAL console (a tee), so
     *        `idf.py monitor` on the primary UART keeps working and nothing is
     *        lost when no CDC host is attached. Best-effort: teeing is only done
     *        when the primary console is a UART (it has an independent port);
     *        with a USB-Serial-JTAG or no console there is nothing to tee to.
     */
    bool tee_console{true};
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
    /**
     * @brief Callback invoked with received HID OUTPUT / SET_REPORT bytes
     *        (host -> device). Enables request/response HID protocols (e.g. the
     *        Nintendo Switch Pro controller handshake): reply by sending an INPUT
     *        report with `write_hid_report()`. When the report descriptor uses
     *        report IDs, byte 0 of the delivered span is the report id. Delivered
     *        from the TinyUSB device task; `write_hid_report()` is safe to call
     *        from within it. Requires `has_out_endpoint` for interrupt-OUT reports
     *        (control SET_REPORT is delivered regardless).
     */
    receive_callback_fn on_receive{nullptr};
  };

  /**
   * @brief X-Input (Xbox 360 wired controller) function.
   *
   * Presents a vendor-specific interface (bInterfaceClass 0xFF / SubClass 0x5D /
   * Protocol 0x01) with one interrupt IN endpoint (20-byte input reports, sent
   * with `UsbDevice::update_xinput_state()`) and one interrupt OUT endpoint (8-byte
   * rumble / LED reports, delivered to `on_rumble`). Unlike HID it is served by a
   * small custom TinyUSB application class driver built into this component (no
   * `CFG_TUD_*` count is required).
   *
   * A PC's XUSB driver only binds a device whose VID/PID is a recognized Xbox 360
   * controller, so `vid` / `pid` default to Microsoft's identifiers
   * (`0x045E:0x028E`) -- for emulation / testing of your own device only. When
   * the XInput function is the ONLY enabled function these identifiers (and a
   * 0xFF/0xFF/0xFF device class) override the top-level Config vid/pid so the
   * host recognizes it; combine XInput with other functions only if you do not
   * need XUSB to bind (the built-in vendor/WebUSB class also claims class 0xFF).
   *
   * Consumes 1 interrupt IN + 1 interrupt OUT endpoint.
   */
  struct XInputFunction {
    std::string interface_name{"espp XInput"}; /**< XInput interface string descriptor. */
    uint16_t vid{espp::xinput::kDefaultVid};   /**< Xbox 360 controller VID (Microsoft). */
    uint16_t pid{espp::xinput::kDefaultPid};   /**< Xbox 360 controller PID. */
    /** @brief Callback invoked with received rumble / LED report bytes (8-byte
     *  reports on the interrupt OUT endpoint). Runs in the TinyUSB device task. */
    receive_callback_fn on_rumble{nullptr};
  };

  /// @brief Which side currently has an MSC medium. A medium belongs to exactly
  ///        one side: while the host has it, the application's `base_path` is
  ///        unmounted (open files there become invalid); while the application
  ///        has it, the host sees the drive as "no medium".
  enum class MscOwner : uint8_t {
    Host = 0, ///< Exposed to the USB host as a drive.
    App,      ///< Mounted at MscMedium::base_path for the application (fopen, std::filesystem).
  };

  /// @brief Storage events reported through MscFunction::on_event /
  ///        set_msc_event_callback().
  enum class MscEvent : uint8_t {
    OwnerChangeStarted, ///< A hand-over between application and host is starting.
    OwnerChanged,       ///< The hand-over completed; `owner` is the new owner.
    OwnerChangeFailed,  ///< The hand-over failed (e.g. the FAT volume could not be mounted).
    FormatRequired,     ///< The medium has no FAT filesystem and format_if_unformatted is off.
    FormatFailed,       ///< Formatting the medium failed.
  };

  /// @brief MSC storage event callback: the medium index (LUN), the event, and
  ///        the owner at the time of the event: the previous owner for
  ///        OwnerChangeStarted and OwnerChangeFailed (the side that still has
  ///        the medium), the new one for OwnerChanged. Runs in the TinyUSB device task for
  ///        host-driven hand-overs (mount / eject / disconnect) and in the
  ///        caller's task for set_msc_owner(); keep it short and do not call
  ///        set_msc_owner() from it.
  using msc_event_callback_fn = std::function<void(size_t lun, MscEvent event, MscOwner owner)>;

  /**
   * @brief One medium exposed by the MSC function (one LUN).
   *
   * The host only understands FAT, so the medium carries a FAT volume: an SD
   * card, or a FAT data partition in flash (accessed through wear levelling).
   * esp_tinyusb supports at most one medium of each type.
   */
  struct MscMedium {
    /// @brief The kind of storage behind this LUN.
    enum class Type : uint8_t {
      SdCard,         ///< An already-initialized SD/MMC card (SDMMC or SDSPI host): `sd_card`.
      FlashPartition, ///< A FAT data partition in flash, by label: `partition_label`.
    };
    Type type{Type::FlashPartition}; /**< Which storage backs this LUN. */
    /** For Type::SdCard: a caller-owned card initialized with sdmmc_card_init() on
     *  an SDMMC or SDSPI host, e.g. espp::SdCard::card() with its volume unmounted.
     *  Must outlive the UsbDevice. Do not pass the card from
     *  esp_vfs_fat_sdmmc_mount() / esp_vfs_fat_sdspi_mount(): the matching
     *  esp_vfs_fat_sdcard_unmount() frees it. Requires a target with an SDMMC host
     *  peripheral (e.g. ESP32-S3, ESP32-P4), even when the card is on SPI. */
    sdmmc_card_t *sd_card{nullptr};
    /** For Type::FlashPartition: label of a `data, fat` partition. The device
     *  mounts wear levelling on it and unmounts it on destruction. */
    std::string partition_label{"storage"};
    /** VFS path where the application sees the files while it owns the medium.
     *  Must be unique per medium, and must not already be mounted by the app
     *  (unmount your own esp_vfs_fat mount of the card first). */
    std::string base_path{"/msc"};
    int max_files{5}; /**< Files the application may keep open at once. */
    /** FAT volume label: the name the host shows for the drive (up to 11
     *  characters; FAT stores it upper-case). Written at initialize() and after
     *  format_msc_medium() when it differs from the medium's current label.
     *  Empty = leave the label alone. Requires CONFIG_FATFS_USE_LABEL=y. */
    std::string volume_label{};
    /** Format the medium as FAT when it is handed to the application and has no
     *  filesystem. Off by default: an unformatted medium raises
     *  MscEvent::FormatRequired instead.
     *  @warning esp_tinyusb formats FatFs drive 0 rather than this medium's own
     *           drive. Only enable this (or call format_msc_medium()) when no
     *           other FAT volume is mounted on the device, or it may format that
     *           volume instead. */
    bool format_if_unformatted{false};
    /** Owner right after initialize(). With auto_handover a host that is (or
     *  becomes) connected takes the medium when it mounts the device. */
    MscOwner initial_owner{MscOwner::App};
  };

  /**
   * @brief MSC (USB mass storage) function: exposes up to two media as USB drives.
   *
   * Consumes 1 bulk IN + 1 bulk OUT endpoint. Built on esp_tinyusb's MSC storage
   * backend, which provides the SCSI handling, so it requires
   * `CONFIG_TINYUSB_MSC_ENABLED=y`; a flash partition additionally needs
   * `CONFIG_TINYUSB_MSC_BUFSIZE >= CONFIG_WL_SECTOR_SIZE`. Prefer 4096-byte wear
   * levelling sectors with a 4096-byte MSC buffer: each host write is then one
   * flash erase + write, while 512-byte sectors need a read-modify-erase of the
   * 4 KiB block per sector (slow in the power-safe mode, and
   * `CONFIG_WL_SECTOR_MODE_PERF` loses the block on a reset mid-erase). SD cards
   * are written directly, with no wear-levelling layer.
   *
   * Ownership: with `auto_handover` (the default) the media move to the host when
   * the host mounts (configures) the device, and back to the application when the
   * host ejects a drive or the device is detached. The hand-over is for ALL media
   * at once: esp_tinyusb ignores which drive was ejected, so ejecting either one
   * returns both to the application (the other drive disappears from the host too). Turn it off to
   * decide yourself with set_msc_owner() (e.g. only expose the card while a "USB drive mode" screen
   * is shown). Either way, never let the application and the host write the same volume at once --
   * that is what the ownership model prevents.
   */
  struct MscFunction {
    std::string interface_name{"espp MSC"}; /**< MSC interface string descriptor. */
    std::vector<MscMedium> media{};         /**< One or two media (LUN 0, LUN 1). */
    bool auto_handover{true}; /**< Host takes all media on mount; the app gets all of them
                                   back on any eject / detach. */
    msc_event_callback_fn on_event{nullptr}; /**< Optional storage event callback. */
  };

  /// @brief Size of an MSC medium.
  struct MscCapacity {
    uint32_t sector_count{0}; ///< Number of sectors.
    uint32_t sector_size{0};  ///< Bytes per sector.
    /// @brief Total size in bytes.
    uint64_t bytes() const { return static_cast<uint64_t>(sector_count) * sector_size; }
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
    // Descriptor details some hosts check (e.g. a Switch expects a Pro Controller
    // to report bcdDevice 0x0210 and 500 mA).
    uint16_t bcd_device{0x0100}; /**< bcdDevice (device release, BCD) in the device descriptor.
                                     Ignored for an XInput-only device (which reports the Xbox
                                     360 value). */
    uint16_t max_power_ma{100};  /**< bMaxPower in the configuration descriptor, in mA; clamped
                                     to 500 and rounded up to the next 2 mA unit. */
    bool remote_wakeup{true};    /**< Advertise remote wakeup in the configuration attributes. */
    /** Attach to the bus (enable the D+ pull-up) at the end of initialize(). Set
     *  false to stay invisible to the host until connect() -- e.g. to finish
     *  application file I/O on an MSC medium before a host can take it. */
    bool connect_on_initialize{true};
    /** USB peripheral port to use, as esp_tinyusb's `tinyusb_port_t` (0 = the
     *  USB-OTG 1.1 full-speed port, 1 = the USB-OTG 2.0 high-speed port on
     *  targets that have one). -1 = TinyUSB's default for the target: the
     *  high-speed port on the ESP32-P4, the full-speed port elsewhere. Boards
     *  do not always route the high-speed port to a device-capable connector
     *  (the M5Stack Tab5 wires it to its USB-A host jack; its USB-C carries the
     *  full-speed port, shared with the USB-Serial-JTAG console), so this lets
     *  the application pick the connector. */
    int port{-1};

    std::optional<CdcFunction> cdc{};       /**< Enable a CDC-ACM function. */
    std::optional<VendorFunction> vendor{}; /**< Enable a vendor-specific / WebUSB function. */
    std::optional<HidFunction> hid{};       /**< Enable a HID function. */
    std::optional<XInputFunction> xinput{}; /**< Enable an X-Input (Xbox 360) function. */
    std::optional<MscFunction> msc{};       /**< Enable an MSC (mass storage) function. */

    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Construct a UsbDevice. Does not touch hardware until initialize().
   * @param config Configuration parameters.
   */
  explicit UsbDevice(const Config &config);

  /**
   * @brief Uninstalls the enabled functions and the TinyUSB driver if initialized.
   * @note MSC media are released too: an application-owned medium's `base_path`
   *       is unmounted, flash partitions are unmounted from wear levelling, and
   *       an SD card is left initialized (the caller owns it) but not mounted.
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
   * @note Same backpressure contract as write_vendor(). A frame that fits in the
   *       TX FIFO (CONFIG_TINYUSB_CDC_TX_BUFSIZE) is written ALL-OR-NOTHING: the
   *       call sleep-waits (bounded, 250 ms) for room for the WHOLE frame and
   *       then enqueues it in a single write, so a drain-timeout or a mid-write
   *       disconnect returns false WITHOUT leaving a truncated prefix on the wire
   *       (a partial frame would poison the host-side framing parser). When
   *       called from TinyUSB-callback context (e.g. inside a receive callback,
   *       which runs on the TinyUSB task) the drain can never happen while this
   *       call blocks, so it fails fast with `no_buffer_space` if the whole frame
   *       does not ALREADY fit - again without enqueueing anything. A frame
   *       LARGER than the FIFO cannot be atomic and is streamed across drains
   *       (a mid-stream timeout may leave a prefix on the wire); keep framed
   *       payloads within the FIFO, or send large replies from your own task
   *       rather than a receive callback, for atomic writes.
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
   * @note A frame that fits in the TX FIFO (CONFIG_TINYUSB_VENDOR_TX_BUFSIZE) is
   *       written ALL-OR-NOTHING: the call sleep-waits (bounded, 250 ms) for room
   *       for the WHOLE frame and then enqueues it in a single write, so a
   *       drain-timeout or a mid-write unmount returns false WITHOUT leaving a
   *       truncated prefix on the wire (a partial frame would poison the
   *       host-side framing parser). When called from TinyUSB-callback context
   *       (e.g. inside a receive callback, which runs on the TinyUSB task) the
   *       drain can never happen while this call blocks, so it fails fast with
   *       `no_buffer_space` if the whole frame does not ALREADY fit - again
   *       without enqueueing anything. A frame LARGER than the FIFO cannot be
   *       atomic and is streamed across drains (a mid-stream timeout may leave a
   *       prefix on the wire); keep framed payloads within the FIFO, or send
   *       large replies from your own task rather than a receive callback, for
   *       atomic writes.
   */
  bool write_vendor(std::span<const uint8_t> data, std::error_code &ec);

  /// @brief Convenience overload of write_vendor() that ignores errors.
  bool write_vendor(std::span<const uint8_t> data);

  /// @brief Bytes of free space currently in the vendor TX FIFO.
  /// @return How many bytes write_vendor() can accept right now without
  ///         blocking, or 0 if not initialized / no vendor interface / not
  ///         mounted. A point-in-time hint: with a single serialized writer it
  ///         is stable, otherwise treat it as advisory. Use it to skip or defer
  ///         a streaming frame when the host has stopped draining the endpoint,
  ///         instead of building the frame and having write_vendor() drop it.
  size_t vendor_write_available() const;

  /// @brief Bytes of free space currently in the CDC TX FIFO.
  /// @return How many bytes write_cdc() can accept right now, or 0 if not
  ///         initialized / no CDC interface / not mounted. See
  ///         vendor_write_available() for usage notes.
  size_t cdc_write_available() const;

  /// @brief Discard any bytes queued in the vendor TX FIFO that have not been
  ///        sent yet. Call this when the host goes away (e.g. on a detected
  ///        disconnect / stream stall) so a stale backlog (queued telemetry) is
  ///        not delivered to the next host that connects and mis-parsed as a
  ///        reply to its first command.
  void vendor_write_clear();

  /// @brief Discard any bytes queued in the CDC TX FIFO that have not been sent
  ///        yet. See vendor_write_clear() for usage notes.
  void cdc_write_clear();

  /**
   * @brief Redirect the ESP console (stdout) to the CDC interface, so the device's
   *        logs travel over the same native USB cable as the other USB
   *        interface(s) (vendor / HID / XInput). Call this AFTER a successful
   *        `initialize()`; or just set `CdcFunction::route_console` and it is done
   *        for you at the end of `initialize()`.
   *
   * `printf`, `ESP_LOG` (via its default vprintf), and `espp::Logger` (which uses
   * `fmt::print`) all write to `stdout`, so redirecting stdout captures them all.
   * A small write-only VFS device is registered and `stdout` is `freopen`ed onto
   * it; its writes forward to `write_cdc()` only when the CDC TX FIFO can take the
   * whole chunk right now, so logging NEVER blocks on an absent or slow reader
   * (dropped console bytes are harmless). When `CdcFunction::tee_console` is set
   * (the default) and the primary console is a UART, writes are also teed to that
   * UART so `idf.py monitor` keeps working.
   *
   * Idempotent (a second call is a no-op). Requires the CDC function to be enabled
   * and the device initialized.
   *
   * @note Lifetime: routing points `stdout` at this device. On destruction the
   *       device detaches itself (later stdout writes degrade to the UART tee),
   *       but a write already in flight can still race destruction -- so a
   *       console-routed UsbDevice must outlive concurrent logging. This is
   *       normally trivial: it is a program-lifetime singleton.
   *
   * @param[out] ec Set on failure (CDC not enabled / not initialized, or the VFS
   *        device could not be registered / stdout could not be reopened).
   * @return true if the console is now routed to CDC (or already was).
   */
  bool route_console_to_cdc(std::error_code &ec);

  /// @brief Convenience overload of route_console_to_cdc() that ignores errors.
  bool route_console_to_cdc();

  /// @brief Whether the console is currently routed to the CDC interface.
  bool is_console_routed_to_cdc() const;

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

  /**
   * @brief Send a fresh X-Input (Xbox 360) input report from a gamepad state.
   * @param state Buttons / triggers / sticks to serialize into the 20-byte report.
   * @param[out] ec Set on failure (XInput not enabled / not initialized, host not
   *        ready / a previous report still in flight, or a transfer error).
   * @return true if the report was queued for transmission, false otherwise.
   * @note Single-writer: call from one task. The report bytes are held in an
   *       internal buffer for the duration of the (asynchronous) transfer.
   */
  bool update_xinput_state(const espp::xinput::GamepadState &state, std::error_code &ec);

  /// @brief Convenience overload of update_xinput_state() that ignores errors.
  bool update_xinput_state(const espp::xinput::GamepadState &state);

  /// @brief Whether the XInput function is enabled, mounted and ready to accept a
  ///        new input report (no report in flight).
  bool is_xinput_ready() const;

  /**
   * @brief Hand an MSC medium to the application or the USB host.
   * @param lun Medium index (position in MscFunction::media).
   * @param owner New owner. Handing it to the App mounts the FAT volume at the
   *        medium's `base_path`; handing it to the Host unmounts it there first.
   * @param[out] ec Set on failure: MSC not enabled / not initialized
   *        (`not_connected`), bad index (`invalid_argument`), the medium has no
   *        FAT filesystem (`no_such_device`, see format_msc_medium()), the host
   *        is attached and still has the medium (`device_or_resource_busy`, see
   *        below), or the volume could not be mounted / unmounted (`io_error`).
   * @return true if `owner` now has the medium.
   * @note Taking a medium from an attached host is refused: esp_tinyusb accepts
   *       host writes and runs them later, without re-checking ownership, so a
   *       write already queued could land under the application's mounted FAT
   *       volume. Have the host eject the drive (auto_handover then returns it),
   *       or detach / destroy the device, first. Handing a medium to the host is
   *       always allowed.
   * @note Blocks for the mount / unmount. Call it from an application task, not
   *       from a USB callback. With auto_handover, the next host mount / eject /
   *       detach still moves the medium automatically -- and a host mount or
   *       eject that happens during this call races it, so turn auto_handover
   *       off if the application drives ownership itself.
   */
  bool set_msc_owner(size_t lun, MscOwner owner, std::error_code &ec);

  /// @brief Convenience overload of set_msc_owner() that ignores errors.
  bool set_msc_owner(size_t lun, MscOwner owner);

  /// @brief Who currently has an MSC medium (nullopt if MSC is not enabled /
  ///        initialized or the index is out of range). An unformatted medium
  ///        waiting for format_msc_medium() reports App, with nothing mounted.
  std::optional<MscOwner> msc_owner(size_t lun) const;

  /// @brief Size of an MSC medium (nullopt if MSC is not enabled / initialized
  ///        or the index is out of range).
  std::optional<MscCapacity> msc_capacity(size_t lun) const;

  /// @brief Number of MSC media (LUNs); 0 if MSC is not enabled / initialized.
  size_t msc_lun_count() const;

  /**
   * @brief Create a FAT filesystem on an MSC medium that has none (e.g. after
   *        MscEvent::FormatRequired), and mount it for the application.
   * @param lun Medium index.
   * @param[out] ec Set on failure: MSC not enabled / not initialized, bad index,
   *        the application does not own the medium (`operation_not_permitted`), a
   *        filesystem already exists (`file_exists`), every FatFs drive slot is in
   *        use (`device_or_resource_busy`), or formatting failed (`io_error`).
   * @return true if the medium was formatted.
   * @warning See MscMedium::format_if_unformatted: esp_tinyusb formats FatFs
   *          drive 0, so only use this when no other FAT volume is mounted.
   * @note With auto_handover, the USB connection is dropped for the duration of
   *       the format (and restored after) so a host attaching mid-format cannot
   *       take the medium while esp_tinyusb is formatting it.
   */
  bool format_msc_medium(size_t lun, std::error_code &ec);

  /// @brief Set or replace the MSC storage event callback (nullptr to detach).
  void set_msc_event_callback(const msc_event_callback_fn &cb);

  /// @brief Set or replace the CDC receive callback (nullptr to detach).
  void set_cdc_receive_callback(const receive_callback_fn &cb);

  /// @brief Set or replace the vendor receive callback (nullptr to detach).
  void set_vendor_receive_callback(const receive_callback_fn &cb);

  /// @brief Set or replace the HID receive callback (received OUTPUT / SET_REPORT
  ///        bytes, host -> device; nullptr to detach).
  void set_hid_receive_callback(const receive_callback_fn &cb);

  /// @brief Register a callback invoked when the device is mounted (the host has
  ///        configured it). Runs in the TinyUSB device-task context; nullptr
  ///        detaches. esp_tinyusb owns the raw tud_mount_cb, so applications
  ///        should register here rather than defining that callback themselves.
  void set_mount_callback(const event_callback_fn &cb);

  /// @brief Register a callback invoked when the device is unmounted (detached /
  ///        re-enumerated). The component clears the vendor + CDC TX FIFOs before
  ///        invoking it. Runs in the TinyUSB device-task context; nullptr
  ///        detaches. Register here instead of defining tud_umount_cb
  ///        (esp_tinyusb already defines it).
  void set_unmount_callback(const event_callback_fn &cb);

  /// @brief Whether initialize() has completed successfully.
  bool is_initialized() const;

  /// @brief Attach to the bus (enable the D+ pull-up) so a host can enumerate the
  ///        device. Only needed after Config::connect_on_initialize = false or a
  ///        disconnect(). @return false if not initialized.
  bool connect();

  /// @brief Detach from the bus (disable the D+ pull-up): the host sees the
  ///        device unplugged. @return false if not initialized.
  bool disconnect();

  /// @brief Whether the CDC function is enabled and a host has asserted DTR.
  bool is_cdc_connected() const;

  /// @brief Whether the vendor function is enabled and the device is mounted.
  bool is_vendor_connected() const;

  /// @brief Opaque bridge letting the TinyUSB C callback trampolines reach the
  ///        device-task-only methods below (defined in usb_device.cpp). An
  ///        implementation detail: it is incomplete here, with nothing callable
  ///        from application code.
  struct Callbacks;

protected:
  //
  // Internal: invoked from the TinyUSB device task via C trampolines / weak
  // overrides (through the Callbacks bridge, or the friended event trampoline).
  // Not part of the public API; not intended to be called by application code.
  //

  /// @brief Internal: drain the CDC RX FIFO and dispatch to the CDC callback.
  void handle_cdc_rx();

  /// @brief Internal: dispatch received vendor bytes to the vendor callback.
  /// @param buffer When non-null (TinyUSB zero-copy RX variant, RX_BUFSIZE==0),
  ///        the just-received bytes to dispatch directly. When null (the FIFO
  ///        variant), the FIFO is drained via `tud_vendor_read()` instead.
  /// @param bufsize Number of bytes at @p buffer (0 when @p buffer is null).
  void handle_vendor_rx(const uint8_t *buffer = nullptr, size_t bufsize = 0);

  /// @brief Internal: dispatch a received HID OUTPUT / SET_REPORT to the HID
  ///        receive callback. `report_id` is the SET_REPORT report id (0 for an
  ///        interrupt-OUT report, whose report id, if any, is buffer[0]); the
  ///        callback always receives the report id as byte 0 of its span.
  void handle_hid_rx(uint8_t report_id, const uint8_t *buffer, size_t bufsize);

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

  /// @brief Internal: dispatch received X-Input rumble / LED report bytes to the
  ///        on_rumble callback. Called from the XInput class driver's OUT
  ///        transfer-complete callback (TinyUSB device task context).
  void handle_xinput_out(const uint8_t *buffer, size_t bufsize);

  /// @brief Internal: the allocated X-Input IN endpoint address (0 if the XInput
  ///        function is not enabled). Used by the write path / readiness check.
  uint8_t xinput_in_endpoint() const;

  /// @brief Internal: route an esp_tinyusb MSC storage event (from the TinyUSB
  ///        task or the task calling set_msc_owner()) to the event callback.
  /// @param storage The esp_tinyusb storage handle the event refers to.
  /// @param event The translated event.
  /// @param owner The owner the event refers to.
  void handle_msc_event(const void *storage, MscEvent event, MscOwner owner);

  /// @brief Internal: hand an MSC medium over and confirm the result against the
  ///        VFS (esp_tinyusb's setter records the requested owner even when the
  ///        mount / unmount failed, and not every failure raises an event).
  bool hand_over_msc(size_t index, MscOwner owner, std::error_code &ec);

  /// @brief Internal: write MscMedium::volume_label to an application-mounted
  ///        medium if it differs from the current label.
  void apply_msc_volume_label(size_t index);

  /// @brief Internal: install the MSC driver and create the storage objects for
  ///        the configured media (before the TinyUSB driver is installed, so a
  ///        host that is already connected finds them on its first mount).
  bool init_msc(std::error_code &ec);

  /// @brief Internal: tear down the MSC media (storage objects, wear levelling,
  ///        MSC driver). Safe to call when none were set up. Call it while no
  ///        TinyUSB task is running (before the driver is installed, or after
  ///        quiesce_msc_before_uninstall() + tinyusb_driver_uninstall()). A
  ///        storage object with host writes still queued cannot be deleted;
  ///        resources behind it are left in place, not freed.
  /// @return true if every medium and the MSC driver were released.
  bool deinit_msc();

  /// @brief Internal: pass barriers through the TinyUSB task until everything
  ///        queued before the call (unplug / auto-hand-over callbacks, deferred
  ///        MSC writes and the writes they queue) has run. @return false on
  ///        timeout (the task did not get through its queue).
  bool drain_tinyusb_task();

  /// @brief Internal: undo what a failed tinyusb_msc_format_storage() left
  ///        registered (VFS path, FatFs mount, diskio drive @p pdrv).
  void clean_up_failed_msc_format(size_t index, uint8_t pdrv);

  /// @brief Internal: detach from the host and wait until the TinyUSB task has
  ///        run everything already queued (deferred MSC writes, a detach /
  ///        auto-hand-over callback), so the MSC media can be deleted once the
  ///        task is stopped. Call before tinyusb_driver_uninstall(). @return false
  ///        if the task did not get through its queue in time.
  bool quiesce_msc_before_uninstall();

  /// @brief Internal: the singleton instance handling the global USB callbacks.
  static UsbDevice *instance();

private:
  // Trampoline registered as tinyusb_config_t::event_cb; routes
  // TINYUSB_EVENT_ATTACHED/DETACHED to the private handlers below.
  friend void espp_usb_device_event_cb(tinyusb_event_t *event, void *arg);

  /// @brief Internal: mount / unmount handling driven by esp_tinyusb's event_cb
  ///        (clears the TX FIFOs on unmount, then invokes the app callback).
  void handle_usb_mount();
  void handle_usb_unmount();

  struct Impl; // holds TinyUSB descriptors, kept alive for driver lifetime
  std::unique_ptr<Impl> impl_;

  Config config_;
  std::atomic<bool> initialized_{false}; // read from the TinyUSB task via the write paths
  // Whether the application wants the device attached (pull-up on): set by
  // initialize() / connect() / disconnect(), so internal detaches (formatting)
  // restore the caller's choice instead of forcing the device visible.
  std::atomic<bool> attached_{false};

  std::mutex cb_mutex_;
  receive_callback_fn on_cdc_receive_;
  receive_callback_fn on_vendor_receive_;
  receive_callback_fn on_xinput_rumble_;
  receive_callback_fn on_hid_receive_;
  event_callback_fn on_mount_;
  event_callback_fn on_unmount_;
  msc_event_callback_fn on_msc_event_;

  // Preallocated RX scratch buffers (sized in initialize()) so the TinyUSB-task
  // RX handlers stay allocation-free (no heap churn on the hot path).
  std::vector<uint8_t> cdc_rx_buf_;
  std::vector<uint8_t> vendor_rx_buf_;
};

} // namespace espp
