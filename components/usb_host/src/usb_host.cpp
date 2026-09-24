#include "usb_host.hpp"

#include <sdkconfig.h>
#include <soc/soc_caps.h>
#if SOC_USB_OTG_SUPPORTED && defined(SOC_USB_UTMI_PHY_NUM) && SOC_USB_UTMI_PHY_NUM > 0
// a high-speed capable host controller: the full-speed-only mode applies
#include <hal/usb_dwc_ll.h>
#define ESPP_USB_HOST_HAS_HS_CONTROLLER 1
#else
#define ESPP_USB_HOST_HAS_HS_CONTROLLER 0
#endif

#include <esp_err.h> // esp_err_to_name()
#include <esp_log.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/soc_caps.h"
#include "usb/usb_helpers.h"
#include "usb/usb_host.h"

using namespace std::chrono_literals;

namespace espp {

// ---------------------------------------------------------------------------
// C-linkage trampolines the HID host driver calls (from its background task).
// `arg` is the owning UsbHost.
// ---------------------------------------------------------------------------
extern "C" void espp_usb_host_driver_event_cb(hid_host_device_handle_t handle,
                                              const hid_host_driver_event_t event, void *arg) {
  if (arg) {
    static_cast<UsbHost *>(arg)->on_driver_event(handle, event);
  }
}

extern "C" void espp_usb_host_interface_event_cb(hid_host_device_handle_t handle,
                                                 const hid_host_interface_event_t event,
                                                 void *arg) {
  if (arg) {
    static_cast<UsbHost *>(arg)->on_interface_event(handle, event);
  }
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
namespace {
// Map an esp_err_t to a std::error_code (the generic category is close enough
// for the intent: callers branch on "did it work", and the log carries detail).
std::error_code make_ec(esp_err_t err) {
  switch (err) {
  case ESP_OK:
    return {};
  case ESP_ERR_INVALID_ARG:
    return std::make_error_code(std::errc::invalid_argument);
  case ESP_ERR_INVALID_STATE:
    return std::make_error_code(std::errc::operation_not_permitted);
  case ESP_ERR_TIMEOUT:
    return std::make_error_code(std::errc::timed_out);
  case ESP_ERR_NOT_FOUND:
    return std::make_error_code(std::errc::no_such_device);
  case ESP_ERR_NOT_SUPPORTED:
    return std::make_error_code(std::errc::not_supported);
  case ESP_ERR_NO_MEM:
    return std::make_error_code(std::errc::not_enough_memory);
  default:
    return std::make_error_code(std::errc::io_error);
  }
}

std::string wchars_to_utf8(const wchar_t *ws) {
  // The HID host driver stores string descriptors as wchar_t code units carrying
  // the device's UTF-16 (USB string descriptors are UTF-16LE). Encode to UTF-8,
  // combining surrogate pairs; a lone/invalid surrogate becomes U+FFFD.
  std::string out;
  if (!ws) {
    return out;
  }
  auto put = [&out](uint32_t cp) {
    if (cp < 0x80) {
      out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
      out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
      out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
      out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  };
  for (; *ws; ++ws) {
    uint32_t cu = static_cast<uint32_t>(*ws) & 0xFFFF;
    if (cu >= 0xD800 && cu <= 0xDBFF) { // high surrogate: needs a low surrogate next
      uint32_t lo = static_cast<uint32_t>(ws[1]) & 0xFFFF;
      if (lo >= 0xDC00 && lo <= 0xDFFF) {
        put(0x10000 + (((cu - 0xD800) << 10) | (lo - 0xDC00)));
        ++ws;
      } else {
        put(0xFFFD);
      }
    } else if (cu >= 0xDC00 && cu <= 0xDFFF) { // stray low surrogate
      put(0xFFFD);
    } else {
      put(cu);
    }
  }
  return out;
}

constexpr uint8_t kHidSubclassBoot = 1;
} // namespace

// ---------------------------------------------------------------------------
// UsbHost::HidDevice
// ---------------------------------------------------------------------------
void UsbHost::HidDevice::set_input_callback(input_callback_fn cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  on_input_ = std::move(cb);
}

bool UsbHost::HidDevice::start(std::error_code &ec) {
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = hid_host_device_start(handle_);
  ec = make_ec(err);
  if (!ec) {
    started_.store(true);
  }
  return !ec;
}

bool UsbHost::HidDevice::stop(std::error_code &ec) {
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = hid_host_device_stop(handle_);
  ec = make_ec(err);
  if (!ec) {
    started_.store(false);
  }
  return !ec;
}

bool UsbHost::HidDevice::send_output_report(uint8_t report_id, std::span<const uint8_t> data,
                                            std::error_code &ec) {
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  // hid_class_request_set_report() takes a non-const buffer. Rather than cast
  // away const (the caller's bytes may live in read-only memory), copy into a
  // stack buffer -- Output reports are small -- and only fall back to the heap
  // for an unusually large one.
  uint8_t stack_buf[Event::kInlineBytes];
  std::vector<uint8_t> heap_buf;
  uint8_t *buf = stack_buf;
  if (data.size() <= sizeof(stack_buf)) {
    std::memcpy(stack_buf, data.data(), data.size());
  } else {
    heap_buf.assign(data.begin(), data.end());
    buf = heap_buf.data();
  }
  esp_err_t err =
      hid_class_request_set_report(handle_, HID_REPORT_TYPE_OUTPUT, report_id, buf, data.size());
  ec = make_ec(err);
  return !ec;
}

bool UsbHost::HidDevice::get_report(hid_report_type_t report_type, uint8_t report_id,
                                    std::span<uint8_t> buffer, size_t &out_length,
                                    std::error_code &ec) {
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  size_t len = buffer.size();
  esp_err_t err = hid_class_request_get_report(handle_, static_cast<uint8_t>(report_type),
                                               report_id, buffer.data(), &len);
  ec = make_ec(err);
  out_length = ec ? 0 : len;
  return !ec;
}

bool UsbHost::HidDevice::set_idle(uint8_t duration, uint8_t report_id, std::error_code &ec) {
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = hid_class_request_set_idle(handle_, duration, report_id);
  ec = make_ec(err);
  return !ec;
}

bool UsbHost::HidDevice::set_protocol(hid_report_protocol_t protocol, std::error_code &ec) {
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = hid_class_request_set_protocol(handle_, protocol);
  ec = make_ec(err);
  return !ec;
}

void UsbHost::HidDevice::deliver_input(std::span<const uint8_t> data) {
  // Intentionally not gated on connected_: this runs on the dispatch task, and
  // reports queued before the Disconnected event are delivered even though the
  // driver task has already marked the device inert for outbound calls.
  input_callback_fn cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = on_input_;
  }
  if (cb) {
    cb(data);
  }
}

void UsbHost::HidDevice::close_interface(const char *where) {
  // Exactly one task may be inside hid_host_device_close() for this interface:
  // the driver task's close on disconnect and a retire() from deinitialize()
  // can otherwise overlap (the disconnect callback runs while the app task is
  // retiring the same device), and the second close would be freeing what the
  // first is still tearing down. The flag is released again afterwards, so a
  // close the driver refused can still be retried by whoever comes next.
  bool expected = false;
  if (!closing_.compare_exchange_strong(expected, true)) {
    return; // another task owns the close of this interface
  }
  if (!closed_.load()) {
    const esp_err_t err = hid_host_device_close(handle_);
    if (err == ESP_OK) {
      closed_.store(true);
    } else if (err == ESP_ERR_INVALID_STATE) {
      // The driver refusing a busy interface (a call in flight) or one it has
      // already let go of. Expected on both paths: whoever closes next gets
      // it, and failing that the driver frees the interface itself once the
      // device is gone, so nothing leaks.
      ESP_LOGD("UsbHost", "hid_host_device_close (%s): interface busy or already gone", where);
    } else {
      ESP_LOGW("UsbHost", "hid_host_device_close (%s): %s", where, esp_err_to_name(err));
    }
  }
  closing_.store(false);
}

void UsbHost::HidDevice::close_on_driver_task() {
  // Runs inside the driver's DISCONNECTED callback, i.e. on the driver task
  // before it continues its own teardown of the device. Closing here keeps
  // the interface release ordered with the driver's pipe handling: closing
  // from another task races the driver's disconnect processing, which
  // ends in hcd_urb_dequeue asserting on a pipe the close already flushed.
  // No io_mutex_: an app-task driver call in flight on this device is
  // completed by this very task, so waiting for it here would deadlock;
  // the driver rejects a close of a busy interface instead, and retire()
  // (dispatch task, later) closes it under the mutex in that case.
  if (!connected_.exchange(false)) {
    return;
  }
  close_interface("disconnect");
}

void UsbHost::HidDevice::retire() {
  // Taking io_mutex_ here waits for any driver call in flight on another task
  // to finish before the interface is closed (and its resources freed).
  std::lock_guard<std::mutex> lk(io_mutex_);
  connected_.store(false);
  if (retired_) {
    return; // idempotent: a second retire() must not re-attempt the close
  }
  retired_ = true;
  close_interface("retire"); // a no-op if the driver task already closed it
}

// ---------------------------------------------------------------------------
// UsbHost
// ---------------------------------------------------------------------------
UsbHost::UsbHost(const Config &config)
    : BaseComponent("UsbHost", config.log_level)
    , config_(config) {}

UsbHost::~UsbHost() {
  if (initialized_.load()) {
    std::error_code ec;
    if (!deinitialize(ec)) {
      // The USB host driver still holds a pointer to this object and would call
      // into freed memory on the next device event. Freeing it anyway would be a
      // silent use-after-free; failing loudly is the only safe option.
      logger_.error("USB host stack could not be released ({}); aborting rather than freeing an "
                    "object the driver still references",
                    ec.message());
      abort();
    }
  }
}

UsbHost::HidDevice::Info UsbHost::read_info(hid_host_device_handle_t handle) {
  HidDevice::Info info;
  hid_host_dev_info_t dev_info{};
  if (hid_host_get_device_info(handle, &dev_info) == ESP_OK) {
    info.vid = dev_info.VID;
    info.pid = dev_info.PID;
    info.manufacturer = wchars_to_utf8(dev_info.iManufacturer);
    info.product = wchars_to_utf8(dev_info.iProduct);
    info.serial_number = wchars_to_utf8(dev_info.iSerialNumber);
  }
  return info;
}

UsbHost::HidDevice::Params UsbHost::read_params(hid_host_device_handle_t handle) {
  HidDevice::Params params;
  hid_host_dev_params_t dev_params{};
  if (hid_host_device_get_params(handle, &dev_params) == ESP_OK) {
    params.address = dev_params.addr;
    params.interface_number = dev_params.iface_num;
    params.sub_class = dev_params.sub_class;
    params.protocol = dev_params.proto;
  }
  return params;
}

bool UsbHost::initialize(std::error_code &ec) {
  if (initialized_.load()) {
    logger_.warn("already initialized");
    ec = std::make_error_code(std::errc::operation_in_progress);
    return false;
  }

  // 1) Install the USB Host library.
  usb_host_config_t host_config = {};
  host_config.skip_phy_setup = false;
  host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
  // Keep the root port OFF until the HID class driver has registered its client:
  // the library only tells clients about devices that enumerate AFTER they
  // register (the HID driver never scans for existing ones), so a device that
  // is already plugged in at boot would otherwise enumerate before anyone is
  // listening and never be opened.
  host_config.root_port_unpowered = true;
#if ESPP_USB_HOST_HAS_PORT_SELECT
  if (config_.port < -1 || config_.port >= static_cast<int>(SOC_USB_OTG_PERIPH_NUM)) {
    logger_.error("Invalid USB port {} (-1 = default, or 0..{})", config_.port,
                  static_cast<int>(SOC_USB_OTG_PERIPH_NUM) - 1);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (config_.port >= 0)
    host_config.peripheral_map = 1u << config_.port;
#else
  // single-controller target, or a USB Host library without peripheral_map
  // (IDF's built-in one on ESP-IDF 5.x): only the default port exists
  if (config_.port != -1) {
    logger_.error("USB port {} requested, but peripheral selection is not available on this "
                  "target / USB Host library (see ESPP_USB_HOST_HAS_PORT_SELECT); use -1",
                  config_.port);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
#endif
  esp_err_t err = usb_host_install(&host_config);
  if (err != ESP_OK) {
    logger_.error("usb_host_install failed: {}", esp_err_to_name(err));
    ec = make_ec(err);
    return false;
  }
  apply_full_speed_only();
#if ESPP_USB_HOST_HAS_HS_CONTROLLER
  // Only meaningful where the periodic re-assert actually runs: on a
  // full-speed-only controller the interval is never read, so warning about it
  // would only mislead.
  if (config_.full_speed_only && config_.full_speed_reassert_interval.count() <= 0) {
    logger_.warn("full_speed_reassert_interval must be > 0 ({}ms given); re-asserting once per "
                 "RTOS tick instead",
                 config_.full_speed_reassert_interval.count());
  }
#endif

  // 2) Start the USB-host-library event task.
  lib_task_run_.store(true);
  lib_event_errors_ = 0;
  lib_task_ = espp::Task::make_unique({
      .callback = [this](std::mutex &m, std::condition_variable &cv) { return lib_task_fn(m, cv); },
      .task_config =
          {
              .name = "usb_host_lib",
              .stack_size_bytes = config_.lib_task_stack_size,
              .priority = config_.task_priority,
              .core_id = config_.task_core_id,
              .stack_alloc_caps = config_.task_stack_alloc_caps,
          },
      .log_level = Logger::Verbosity::WARN,
  });
  if (!lib_task_->start()) {
    logger_.error("failed to start usb host lib task");
    lib_task_run_.store(false);
    lib_task_.reset();
    usb_host_uninstall();
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }

  // 3) Start the dispatch task (runs device open/close + all user callbacks off
  //    the driver task, so callbacks may issue control transfers).
  dispatch_run_.store(true);
  dispatch_task_ = espp::Task::make_unique({
      .callback = [this](std::mutex &m,
                         std::condition_variable &cv) { return dispatch_task_fn(m, cv); },
      .task_config =
          {
              .name = "usb_host_cb",
              .stack_size_bytes = config_.dispatch_task_stack_size,
              .priority = config_.task_priority,
              .core_id = config_.task_core_id,
              .stack_alloc_caps = config_.task_stack_alloc_caps,
          },
      .log_level = Logger::Verbosity::WARN,
  });
  if (!dispatch_task_->start()) {
    logger_.error("failed to start usb host dispatch task");
    dispatch_run_.store(false);
    dispatch_task_.reset();
    stop_lib_task();
    usb_host_uninstall();
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }
  accepting_.store(true); // driver callbacks may now enqueue

  // 4) Install the HID class driver. Its event loop runs on a task of ours
  //    (create_background_task = false) so the stack placement follows
  //    task_stack_alloc_caps like the other two. The driver then creates no
  //    task and ignores the priority / stack / core fields below -- they are
  //    left at our configured values only so the struct reads consistently;
  //    start_hid_task() is where those settings actually take effect.
  const hid_host_driver_config_t hid_config = {
      .create_background_task = false,
      .task_priority = config_.task_priority,
      .stack_size = config_.hid_task_stack_size,
      .core_id = config_.task_core_id < 0 ? tskNO_AFFINITY : config_.task_core_id,
      .callback = &espp_usb_host_driver_event_cb,
      .callback_arg = this,
  };
  err = hid_host_install(&hid_config);
  if (err != ESP_OK) {
    logger_.error("hid_host_install failed: {}", esp_err_to_name(err));
    stop_dispatch_task();
    stop_lib_task(); // join the lib task before uninstalling the library
    usb_host_uninstall();
    ec = make_ec(err);
    return false;
  }
  if (!start_hid_task()) {
    // Without the pump nothing dispatches the driver's events, so the host
    // would look installed and never report a device. Unwind as for a failed
    // install (the driver's uninstall needs no pump when it tracks nothing).
    logger_.error("could not start the HID event pump task");
    accepting_.store(false); // no callback may enqueue into a queue being torn down
    if (esp_err_t uerr = hid_host_uninstall(); uerr != ESP_OK) {
      logger_.error("hid_host_uninstall during unwind: {}", esp_err_to_name(uerr));
    }
    stop_dispatch_task();
    stop_lib_task();
    usb_host_uninstall();
    ec = std::make_error_code(std::errc::resource_unavailable_try_again);
    return false;
  }

  // Now that the HID client exists, power the root port: an already-attached
  // device enumerates from here and is reported to the driver.
  if (config_.root_port_power_on_delay.count() > 0) {
    logger_.debug("waiting {} ms before powering the root port",
                  config_.root_port_power_on_delay.count());
    std::this_thread::sleep_for(config_.root_port_power_on_delay);
  }
  err = usb_host_lib_set_root_port_power(true);
  if (err != ESP_OK) {
    logger_.error("usb_host_lib_set_root_port_power failed: {}", esp_err_to_name(err));
    accepting_.store(false); // no callback may enqueue into a queue being torn down
    // With the pump running, a successful uninstall returns only after the pump
    // has seen ESP_FAIL; a failure here (nothing is tracked yet, so it would be
    // the driver already mid-uninstall) is logged, and the pump is joined
    // either way so the next initialize() starts a fresh one.
    if (esp_err_t uerr = hid_host_uninstall(); uerr != ESP_OK) {
      logger_.error("hid_host_uninstall during unwind: {}", esp_err_to_name(uerr));
    }
    stop_hid_task();
    stop_dispatch_task();
    stop_lib_task();
    usb_host_uninstall();
    ec = make_ec(err);
    return false;
  }

  if (config_.vbus_control)
    config_.vbus_control(true); // the host is listening: now let the jack power the device

  initialized_.store(true);
  logger_.info("USB host installed on USB-OTG peripheral {} of {}",
               config_.port >= 0 ? config_.port : 0, static_cast<int>(SOC_USB_OTG_PERIPH_NUM));
  ec.clear();
  return true;
}

bool UsbHost::deinitialize(std::error_code &ec) {
  if (!initialized_.load()) {
    ec.clear();
    return true;
  }
  logger_.info("uninstalling USB host");

  // 1) Stop the dispatch task first, so no further driver operations or user
  //    callbacks are issued from it (a callback in flight finishes; the HID
  //    driver is still installed, so an in-flight control transfer completes).
  stop_dispatch_task();

  // 2) Retire every device we opened (serialized against app-task I/O) and let
  //    the application know, since the dispatch task is no longer around to.
  std::vector<std::shared_ptr<HidDevice>> devices;
  {
    std::lock_guard<std::mutex> lk(devices_mutex_);
    for (auto &[handle, dev] : devices_) {
      (void)handle;
      devices.push_back(dev);
    }
    devices_.clear();
  }
  for (auto &dev : devices) {
    dev->retire();
    if (config_.on_device_disconnected) {
      config_.on_device_disconnected(dev);
    }
  }

  // 3) The HID driver only forgets a device when the USB stack reports it gone,
  //    and it refuses to uninstall while it still tracks one. Power down the
  //    root port so any attached device is reported gone and let our event
  //    pump run until the driver has released every device we opened
  //    (bounded), then uninstall while the pump keeps running: the driver's
  //    uninstall waits for one more hid_host_handle_events() return, which
  //    then reports ESP_FAIL and the pump exits by itself (hid_task_fn).
  usb_host_lib_set_root_port_power(false);
  if (config_.vbus_control)
    config_.vbus_control(false); // and the board's jack, if it switches it
  // One budget for the whole of this step: the wait for the driver to release
  // the devices and the uninstall retries that follow share it, so a driver
  // that never releases costs ~kTeardownBudget, not that twice over. The
  // retries keep a small floor of their own because the driver's bookkeeping
  // lags its disconnect callbacks by a little even when the wait succeeded
  // immediately. (In practice, with the gate signalled from the driver task,
  // this whole step completes in tens of milliseconds.)
  static constexpr auto kTeardownBudget = 2s;
  static constexpr auto kUninstallRetryDelay = 10ms;
  static constexpr auto kUninstallRetryFloor = 200ms;
  const auto deadline = std::chrono::steady_clock::now() + kTeardownBudget;
  if (!wait_for_untracked(kTeardownBudget)) {
    logger_.warn("driver still tracks {} device(s) after {} ms; uninstalling anyway",
                 num_tracked_devices(),
                 std::chrono::duration_cast<std::chrono::milliseconds>(kTeardownBudget).count());
  }
  const auto retry_until =
      std::max(deadline, std::chrono::steady_clock::now() + kUninstallRetryFloor);
  esp_err_t err = ESP_FAIL;
  do {
    err = hid_host_uninstall();
    if (err == ESP_OK) {
      break;
    }
    std::this_thread::sleep_for(kUninstallRetryDelay);
  } while (std::chrono::steady_clock::now() < retry_until);
  if (err != ESP_OK) {
    // Tearing down under a driver that still references us would be a
    // use-after-free waiting to happen; stay initialized and report it. The
    // root port deliberately stays powered OFF: powering it back up would make
    // the driver re-enumerate (and track) the attached device again, which is
    // exactly what a retry of deinitialize() needs to have gone away. Event
    // delivery is already stopped, so the only valid next steps are retrying
    // deinitialize() or destroying the object (see the header).
    // ESP_ERR_INVALID_STATE is what the driver returns while it still tracks a
    // device; anything else is reported as-is rather than guessed at.
    // The HID pump is intentionally NOT stopped here: hid_host_uninstall()
    // only completes while a task pumps its events, so a retry needs it alive.
    // The destructor aborts rather than freeing a host the driver still
    // references, so leaving it running cannot become a use-after-free.
    logger_.error("hid_host_uninstall failed: {}{}; root port left powered off, retry "
                  "deinitialize()",
                  esp_err_to_name(err),
                  err == ESP_ERR_INVALID_STATE ? " (the driver still tracks a device)" : "");
    ec = make_ec(err);
    return false;
  }
  stop_hid_task();         // exited on its own (ESP_FAIL from the pump); join it
  clear_tracked_devices(); // the driver is gone: no disconnect can arrive to do it

  // 4) The HID driver is gone. Free any remaining devices so the library can
  //    be uninstalled, then stop + join the lib task and uninstall.
  usb_host_device_free_all();
  stop_lib_task();

  err = usb_host_uninstall();
  if (err != ESP_OK) {
    // The library is still installed: stay initialized so the object is never
    // freed under a live stack (and a retry of deinitialize() is possible).
    logger_.error("usb_host_uninstall failed: {}", esp_err_to_name(err));
    ec = make_ec(err);
    return false;
  }

  initialized_.store(false);
  ec.clear();
  return true;
}

bool UsbHost::print_usb_devices() {
  if (!initialized_.load())
    return false;
  // a throw-away asynchronous client: opening a device needs one, and the HID
  // driver's is private to it
  usb_host_client_config_t client_config = {};
  client_config.is_synchronous = false;
  client_config.max_num_event_msg = 4;
  client_config.async.client_event_callback = [](const usb_host_client_event_msg_t *, void *) {};
  client_config.async.callback_arg = nullptr;
  usb_host_client_handle_t client = nullptr;
  esp_err_t err = usb_host_client_register(&client_config, &client);
  if (err != ESP_OK) {
    logger_.error("usb_host_client_register failed: {}", esp_err_to_name(err));
    return false;
  }
  // Size the list from the library's own count so a hub full of devices is
  // never silently capped (usb_host_device_addr_list_fill() reports at most
  // list_len entries and returns ESP_OK either way). That count also includes
  // devices still being enumerated, which the address list omits, so print
  // both: the difference is a device stuck in enumeration.
  usb_host_lib_info_t lib_info = {};
  const int counted = usb_host_lib_info(&lib_info) == ESP_OK ? lib_info.num_devices : 0;
  std::vector<uint8_t> addresses(static_cast<size_t>(std::max(counted, 1)), 0);
  int count = 0;
  err =
      usb_host_device_addr_list_fill(static_cast<int>(addresses.size()), addresses.data(), &count);
  if (err == ESP_OK) {
    printf("USB devices: %d counted by the library, %d fully enumerated (listed below)\n", counted,
           count);
    for (int i = 0; i < count; ++i) {
      usb_device_handle_t dev = nullptr;
      if (usb_host_device_open(client, addresses[i], &dev) != ESP_OK) {
        printf("  address %d: open failed\n", addresses[i]);
        continue;
      }
      usb_device_info_t info = {};
      if (usb_host_device_info(dev, &info) == ESP_OK)
        printf("--- address %d: speed %s, bConfigurationValue %d\n", addresses[i],
               info.speed == USB_SPEED_LOW    ? "low"
               : info.speed == USB_SPEED_FULL ? "full"
                                              : "high",
               info.bConfigurationValue);
      const usb_device_desc_t *dev_desc = nullptr;
      if (usb_host_get_device_descriptor(dev, &dev_desc) == ESP_OK)
        usb_print_device_descriptor(dev_desc);
      const usb_config_desc_t *cfg_desc = nullptr;
      if (usb_host_get_active_config_descriptor(dev, &cfg_desc) == ESP_OK)
        usb_print_config_descriptor(cfg_desc, nullptr);
      usb_host_device_close(client, dev);
    }
  } else {
    logger_.error("usb_host_device_addr_list_fill failed: {}", esp_err_to_name(err));
  }
  usb_host_client_deregister(client);
  return err == ESP_OK;
}

size_t UsbHost::num_usb_devices() const {
  if (!initialized_.load())
    return 0;
  usb_host_lib_info_t info = {};
  if (usb_host_lib_info(&info) != ESP_OK)
    return 0;
  return static_cast<size_t>(info.num_devices);
}

std::vector<std::shared_ptr<UsbHost::HidDevice>> UsbHost::devices() const {
  std::vector<std::shared_ptr<HidDevice>> out;
  std::lock_guard<std::mutex> lk(devices_mutex_);
  out.reserve(devices_.size());
  for (const auto &[handle, dev] : devices_) {
    (void)handle;
    out.push_back(dev);
  }
  return out;
}

std::shared_ptr<UsbHost::HidDevice> UsbHost::find_device(hid_host_device_handle_t handle) const {
  std::lock_guard<std::mutex> lk(devices_mutex_);
  auto it = devices_.find(handle);
  return it == devices_.end() ? nullptr : it->second;
}

// ---------------------------------------------------------------------------
// USB Host library task
// ---------------------------------------------------------------------------
void UsbHost::apply_full_speed_only() {
#if ESPP_USB_HOST_HAS_HS_CONTROLLER
  // The LL setter is idempotent (it writes the bit), and there is no LL
  // getter, so it is simply re-applied rather than read back through the
  // register struct: a root port recovery soft-resets the controller and
  // clears the bit, and no event reports that.
  if (config_.full_speed_only) {
    usb_dwc_ll_hcfg_set_fsls_supp_only(USB_DWC_LL_GET_HW(0));
  } else {
    // Clear it explicitly instead of trusting whatever ran before: IDF never
    // touches this bit, so a host installed earlier in this boot with
    // full_speed_only set would otherwise leave the port full-speed-only for a
    // host that did not ask for it. There is no LL clear, so the bitfield is
    // written the same way usb_dwc_ll_hcfg_set_fsls_supp_only() writes it.
    USB_DWC_LL_GET_HW(0)->hcfg_reg.fslssupp = 0;
  }
#endif
}

TickType_t UsbHost::reassert_wait_ticks(std::chrono::milliseconds interval) {
  // Clamped to a real block: pdMS_TO_TICKS() truncates, so an interval shorter
  // than one tick (the default tick is 10ms) would otherwise round to 0 and
  // turn usb_host_lib_handle_events() into a non-blocking call, i.e. a busy
  // loop. The top end is capped just below portMAX_DELAY so a very long
  // interval cannot wrap into "wait forever" (or into 0).
  constexpr TickType_t kMaxWait = portMAX_DELAY - 1;
  const int64_t ms = interval.count();
  if (ms <= 0) {
    return 1;
  }
  const int64_t ticks = static_cast<int64_t>(pdMS_TO_TICKS(ms));
  if (ticks <= 0) {
    return 1;
  }
  return ticks >= static_cast<int64_t>(kMaxWait) ? kMaxWait : static_cast<TickType_t>(ticks);
}

bool UsbHost::lib_task_fn(std::mutex & /*m*/, std::condition_variable & /*cv*/) {
  uint32_t event_flags = 0;
  // Block until the library has an event (stop_lib_task() unblocks it); with
  // full_speed_only on a high-speed capable controller the loop wakes
  // periodically to re-assert the mode, since a root port recovery (after a
  // transfer error / unplug) soft-resets the controller and clears it.
  constexpr bool kPeriodic = ESPP_USB_HOST_HAS_HS_CONTROLLER != 0;
  // (written as one expression: naming the intermediate makes static analysis
  // report a condition that is always false on a full-speed-only target)
  const TickType_t wait = (kPeriodic && config_.full_speed_only)
                              ? reassert_wait_ticks(config_.full_speed_reassert_interval)
                              : portMAX_DELAY;
  const esp_err_t err = usb_host_lib_handle_events(wait, &event_flags);
  if (err != ESP_OK) {
    // only ESP_OK writes event_flags; anything else leaves whatever the call
    // decided not to report, so do not act on it
    event_flags = 0;
  }
  if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
    // not expected while installed; rate-limited so a persistent failure does
    // not flood the log from this loop (per instance, reset when the task
    // starts, so the cadence follows this host's lifecycle)
    if (++lib_event_errors_ == 1 || lib_event_errors_ % 100 == 0) {
      logger_.error("usb_host_lib_handle_events: {} ({} so far)", esp_err_to_name(err),
                    lib_event_errors_);
    }
  }
  apply_full_speed_only();
  if (event_flags)
    logger_.debug("USB host lib event flags {:#x}", event_flags);
  if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
    // No registered clients: it is safe to release the devices.
    usb_host_device_free_all();
  }
  if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
    logger_.debug("all USB devices freed");
  }
  return !lib_task_run_.load(); // true = stop the task
}

bool UsbHost::start_hid_task() {
  if (hid_task_) {
    return true;
  }
  hid_task_run_.store(true);
  hid_event_errors_ = 0;
  hid_task_ = espp::Task::make_unique({
      .callback = [this](std::mutex &m, std::condition_variable &cv) { return hid_task_fn(m, cv); },
      .task_config =
          {
              .name = "usb_host_hid",
              .stack_size_bytes = config_.hid_task_stack_size,
              .priority = config_.task_priority,
              .core_id = config_.task_core_id,
              .stack_alloc_caps = config_.task_stack_alloc_caps,
          },
      .log_level = Logger::Verbosity::WARN,
  });
  if (!hid_task_ || !hid_task_->start()) {
    hid_task_run_.store(false);
    hid_task_.reset();
    return false;
  }
  return true;
}

bool UsbHost::hid_task_fn(std::mutex & /*m*/, std::condition_variable & /*cv*/) {
  // The driver's event pump; a bounded wait so the stop flag is observed.
  // ESP_FAIL means hid_host_uninstall() is in progress and was waiting for
  // this return: the pump must not call the driver again. ESP_OK and a
  // timeout are the normal returns; anything else (e.g. the driver gone:
  // ESP_ERR_INVALID_STATE) is logged, rate-limited, and the pump keeps
  // going so a transient error does not silently kill event delivery.
  // How long the pump blocks in the driver before looking at the stop flag.
  // It is not a poll interval: hid_host_handle_events() returns as soon as the
  // driver has an event, so this only bounds how quickly stop_hid_task() is
  // noticed (and the idle wakeup rate), not event latency.
  static constexpr auto kPumpWait = pdMS_TO_TICKS(100);
  const esp_err_t err = hid_host_handle_events(kPumpWait);
  if (err == ESP_FAIL) {
    hid_task_run_.store(false);
  } else if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
    // per instance, reset when the pump starts, so the log cadence follows
    // this host's lifecycle rather than the process's
    if (++hid_event_errors_ == 1 || hid_event_errors_ % 100 == 0) {
      logger_.error("hid_host_handle_events: {} ({} so far)", esp_err_to_name(err),
                    hid_event_errors_);
    }
  }
  return !hid_task_run_.load(); // true = stop the task
}

void UsbHost::stop_hid_task() {
  if (!hid_task_) {
    return;
  }
  hid_task_run_.store(false);
  hid_task_->stop();
  hid_task_.reset();
}

void UsbHost::track_opened_device(hid_host_device_handle_t handle) {
  std::lock_guard<std::mutex> lk(tracked_mutex_);
  driver_tracked_.insert(handle);
}

void UsbHost::drop_tracked_device(hid_host_device_handle_t handle) {
  // Silent counterpart of release_tracked_device(): used when an open fails, so
  // there is nothing noteworthy about the interface no longer being tracked.
  bool empty = false;
  {
    std::lock_guard<std::mutex> lk(tracked_mutex_);
    if (driver_tracked_.erase(handle) == 0) {
      return; // a disconnect already took it out
    }
    empty = driver_tracked_.empty();
  }
  if (empty) {
    tracked_cv_.notify_all();
  }
}

void UsbHost::release_tracked_device(hid_host_device_handle_t handle) {
  bool empty = false;
  bool untracked = false;
  {
    std::lock_guard<std::mutex> lk(tracked_mutex_);
    untracked = driver_tracked_.erase(handle) == 0;
    empty = driver_tracked_.empty();
  }
  // Both of these are done with the mutex released: the logger takes locks of
  // its own, and this runs on the driver's callback task.
  if (untracked) {
    // A disconnect for an interface we never opened (the filter rejected it,
    // or the open failed): nothing to release. Not an error, but worth a line
    // while tracing teardown.
    logger_.debug("disconnect for an interface this host did not open");
    return;
  }
  if (empty) {
    tracked_cv_.notify_all();
  }
}

void UsbHost::clear_tracked_devices() {
  {
    std::lock_guard<std::mutex> lk(tracked_mutex_);
    driver_tracked_.clear();
  }
  tracked_cv_.notify_all();
}

size_t UsbHost::num_tracked_devices() const {
  std::lock_guard<std::mutex> lk(tracked_mutex_);
  return driver_tracked_.size();
}

bool UsbHost::wait_for_untracked(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lk(tracked_mutex_);
  return tracked_cv_.wait_for(lk, timeout, [this] { return driver_tracked_.empty(); });
}

void UsbHost::stop_lib_task() {
  if (!lib_task_) {
    return;
  }
  lib_task_run_.store(false);
  // Unblock the task so it observes the stop flag and returns, then join it.
  // Without full_speed_only it is parked in usb_host_lib_handle_events()
  // indefinitely and this is the only thing that wakes it; with the periodic
  // re-assert it would also wake on its own within one
  // full_speed_reassert_interval, so this only shortens the shutdown.
  usb_host_lib_unblock();
  lib_task_->stop();
  lib_task_.reset();
}

// ---------------------------------------------------------------------------
// HID driver task side: only enqueue
// ---------------------------------------------------------------------------
void UsbHost::enqueue(Event &&ev) {
  if (!accepting_.load()) {
    return; // tearing down (or not yet up): there is no consumer, so keep nothing
  }
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    // Re-check under the lock: stop_dispatch_task() clears accepting_ and then
    // clears the queue under this same mutex, so an enqueue that passed the
    // unlocked check can't slip a stale event in after the clear.
    if (!accepting_.load()) {
      return;
    }
    if (queue_.size() >= config_.max_queued_events) {
      // Never block the USB driver task, and keep the queue bounded (see the
      // Config::max_queued_events doc for the exact bound).
      if (ev.type == Event::Type::Input) {
        // The consumer is behind: drop this report. Rate-limit the log so a
        // sustained backlog doesn't spend the driver task's time logging.
        if (++dropped_inputs_ == 1 || dropped_inputs_ % 100 == 0) {
          logger_.warn("event queue full; {} input report(s) dropped so far", dropped_inputs_);
        }
        return;
      }
      // A lifecycle event: make room by evicting the oldest queued Input report.
      auto victim = std::find_if(queue_.begin(), queue_.end(),
                                 [](const Event &e) { return e.type == Event::Type::Input; });
      if (victim != queue_.end()) {
        queue_.erase(victim);
      } else if (ev.type == Event::Type::NewDevice) {
        // Only lifecycle events are queued and the consumer is overloaded: leave
        // this device unopened rather than grow without bound. A Disconnected
        // event is always kept -- it can only follow an opened device, so those
        // are bounded by the open-device count.
        logger_.warn("event queue full; not opening newly attached HID device");
        return;
      }
    }
    queue_.push_back(std::move(ev));
  }
  queue_cv_.notify_one();
}

void UsbHost::on_driver_event(hid_host_device_handle_t handle, hid_host_driver_event_t event) {
  if (event != HID_HOST_DRIVER_EVENT_CONNECTED) {
    return;
  }
  // Everything else (filter, open, set-protocol, start, user callback) needs
  // the dispatch task: opening / configuring a device involves control
  // transfers that this task is responsible for completing.
  enqueue(Event{.type = Event::Type::NewDevice, .handle = handle});
}

void UsbHost::on_interface_event(hid_host_device_handle_t handle,
                                 hid_host_interface_event_t event) {
  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
    // The report lives in the driver's transfer buffer, which is reused as soon
    // as this callback returns -- so copy it out here, then hand the copy to
    // the dispatch task.
    Event ev{.type = Event::Type::Input, .handle = handle};
    uint8_t *buf = ev.inline_data.data();
    size_t cap = std::min(config_.max_input_report_size, Event::kInlineBytes);
    if (config_.max_input_report_size > Event::kInlineBytes) {
      // Opt-in larger reports: reuse a recycled buffer (its capacity already
      // covers max_input_report_size, so resize() does not reallocate) rather
      // than allocating on every report.
      {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (!overflow_pool_.empty()) {
          ev.overflow = std::move(overflow_pool_.back());
          overflow_pool_.pop_back();
        }
      }
      ev.overflow.resize(config_.max_input_report_size);
      buf = ev.overflow.data();
      cap = ev.overflow.size();
    }
    size_t len = 0;
    esp_err_t err = hid_host_device_get_raw_input_report_data(handle, buf, cap, &len);
    if (err != ESP_OK) {
      return;
    }
    if (len > cap) {
      // The driver copies at most `cap` bytes, so this only happens if it ever
      // reports the report's full length rather than the copied length; never
      // let it turn into an out-of-bounds span.
      logger_.warn("input report of {} bytes truncated to {} (max_input_report_size)", len, cap);
      len = cap;
    }
    ev.len = len;
    enqueue(std::move(ev));
    break;
  }
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    // Close the interface now, on the driver task (see close_on_driver_task),
    // and only then count it out of the tracked set, so "untracked" means the
    // close has been made and deinitialize() does not race the driver's own
    // teardown of the interface. Membership in that set -- not a lookup in
    // devices_ -- decides: deinitialize() clears devices_ before it powers the
    // root port down, so the disconnects it provokes would otherwise never be
    // counted out. The dispatch task then retires the device object in order,
    // after any queued inputs, and tells the application.
    if (auto device = find_device(handle)) {
      device->close_on_driver_task();
    }
    release_tracked_device(handle);
    enqueue(Event{.type = Event::Type::Disconnected, .handle = handle});
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    logger_.warn("HID transfer error");
    break;
  default:
    break;
  }
}

// ---------------------------------------------------------------------------
// Dispatch task side: the real work + user callbacks
// ---------------------------------------------------------------------------
bool UsbHost::dispatch_task_fn(std::mutex & /*m*/, std::condition_variable & /*cv*/) {
  std::deque<Event> batch;
  {
    std::unique_lock<std::mutex> lk(queue_mutex_);
    // Bounded wait so Task::stop() is never held up for long; the stop path
    // also notifies queue_cv_ directly.
    queue_cv_.wait_for(lk, 100ms, [this] { return !queue_.empty() || !dispatch_run_.load(); });
    batch.swap(queue_);
  }
  for (auto &ev : batch) {
    if (!dispatch_run_.load()) {
      break;
    }
    switch (ev.type) {
    case Event::Type::NewDevice:
      handle_new_device(ev.handle);
      break;
    case Event::Type::Input:
      handle_input(ev.handle, ev.data());
      if (!ev.overflow.empty()) { // recycle the large-report buffer
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (overflow_pool_.size() < config_.max_queued_events) {
          overflow_pool_.push_back(std::move(ev.overflow));
        }
      }
      break;
    case Event::Type::Disconnected:
      handle_disconnected(ev.handle);
      break;
    }
  }
  return !dispatch_run_.load(); // true = stop the task
}

void UsbHost::stop_dispatch_task() {
  accepting_.store(false); // driver callbacks that race teardown enqueue nothing
  dispatch_run_.store(false);
  queue_cv_.notify_all();
  if (dispatch_task_) {
    dispatch_task_->stop();
    dispatch_task_.reset();
  }
  std::lock_guard<std::mutex> lk(queue_mutex_);
  queue_.clear();
  overflow_pool_.clear();
}

void UsbHost::handle_new_device(hid_host_device_handle_t handle) {
  // Identity + interface parameters are readable before the interface is
  // opened (the driver has already enumerated the device); snapshot them now,
  // they are static for the life of the connection.
  HidDevice::Info info = read_info(handle);
  HidDevice::Params params = read_params(handle);
  logger_.info("HID device connected: VID={:#06x} PID={:#06x} iface={} proto={}", info.vid,
               info.pid, params.interface_number, params.protocol);

  if (config_.should_open && !config_.should_open(info, params)) {
    logger_.debug("filter rejected device; not opening");
    return;
  }

  // Open the HID interface, routing its events back to us (on the driver task).
  // The interface is counted as tracked *before* the open, not after: the open
  // registers the callback below, so a disconnect can be delivered on the
  // driver task while the open is still returning. Counting it in afterwards
  // would let that disconnect find nothing to release and leave the interface
  // in the tracked set forever, which is what gates teardown. An open that
  // fails takes it back out (and so does a disconnect that beat us to it --
  // the erase is a no-op then).
  const hid_host_device_config_t dev_config = {
      .callback = &espp_usb_host_interface_event_cb,
      .callback_arg = this,
  };
  track_opened_device(handle);
  esp_err_t err = hid_host_device_open(handle, &dev_config);
  if (err != ESP_OK) {
    drop_tracked_device(handle);
    logger_.error("hid_host_device_open failed: {}", esp_err_to_name(err));
    return;
  }

  // Boot-subclass interfaces may come up in boot protocol; ask those for report
  // protocol so we always get the full report-descriptor'd reports (only such
  // interfaces are required to support the request). Safe here: we are on the
  // dispatch task, not the driver task.
  if (params.sub_class == kHidSubclassBoot) {
    esp_err_t perr = hid_class_request_set_protocol(handle, HID_REPORT_PROTOCOL_REPORT);
    if (perr != ESP_OK) {
      logger_.debug("set_protocol(report) not honored: {}", esp_err_to_name(perr));
    }
  }

  // Snapshot the report descriptor (driver-owned memory, valid only while the
  // interface is open) into the device object.
  std::vector<uint8_t> descriptor;
  {
    size_t len = 0;
    uint8_t *desc = hid_host_get_report_descriptor(handle, &len);
    if (desc && len > 0) {
      descriptor.assign(desc, desc + len);
    }
  }

  auto device = std::shared_ptr<HidDevice>(
      new HidDevice(handle, std::move(info), std::move(params), std::move(descriptor)));
  {
    std::lock_guard<std::mutex> lk(devices_mutex_);
    devices_[handle] = device;
  }

  // Let the application install its input callback *before* reports flow.
  if (config_.on_device_connected) {
    config_.on_device_connected(device);
  }

  if (config_.auto_start) {
    std::error_code sec;
    if (!device->start(sec)) {
      logger_.warn("hid_host_device_start failed: {}", sec.message());
    }
  }
}

void UsbHost::handle_input(hid_host_device_handle_t handle, std::span<const uint8_t> data) {
  if (auto dev = find_device(handle)) {
    dev->deliver_input(data);
  }
}

void UsbHost::handle_disconnected(hid_host_device_handle_t handle) {
  std::shared_ptr<HidDevice> device;
  {
    std::lock_guard<std::mutex> lk(devices_mutex_);
    auto it = devices_.find(handle);
    if (it != devices_.end()) {
      device = it->second;
      devices_.erase(it);
    }
  }
  if (!device) {
    return; // not one we opened (or already torn down) -- nothing to close
  }
  logger_.info("HID device disconnected");
  device->retire();
  if (config_.on_device_disconnected) {
    config_.on_device_disconnected(device);
  }
}

} // namespace espp
