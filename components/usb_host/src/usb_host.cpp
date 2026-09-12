#include "usb_host.hpp"

#include <cstring>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb/usb_host.h"

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
  std::string out;
  if (!ws) {
    return out;
  }
  for (; *ws; ++ws) {
    // The HID host driver stores string descriptors as UCS-2; keep ASCII and
    // approximate the rest (device identity strings are informational).
    wchar_t c = *ws;
    out.push_back(c < 0x80 ? static_cast<char>(c) : '?');
  }
  return out;
}
} // namespace

// ---------------------------------------------------------------------------
// UsbHost::HidDevice
// ---------------------------------------------------------------------------
UsbHost::HidDevice::Info UsbHost::HidDevice::info() const { return UsbHost::read_info(handle_); }

UsbHost::HidDevice::Params UsbHost::HidDevice::params() const {
  return UsbHost::read_params(handle_);
}

std::vector<uint8_t> UsbHost::HidDevice::report_descriptor() const {
  if (!connected_.load()) {
    return {};
  }
  size_t len = 0;
  // The driver returns a pointer into memory it owns, valid only while the
  // device is connected. Copy it out so the caller can't be left with a dangling
  // reference if the device disconnects concurrently.
  uint8_t *desc = hid_host_get_report_descriptor(handle_, &len);
  if (!desc || len == 0) {
    return {};
  }
  return std::vector<uint8_t>(desc, desc + len);
}

void UsbHost::HidDevice::set_input_callback(input_callback_fn cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  on_input_ = std::move(cb);
}

bool UsbHost::HidDevice::start(std::error_code &ec) {
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
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  // hid_class_request_set_report's signature is non-const, but a SET_REPORT is a
  // host->device transfer: the driver only reads the buffer, it does not write
  // it. const_cast avoids an allocation + copy on every output report (hot path
  // for e.g. WDI feedback).
  esp_err_t err = hid_class_request_set_report(handle_, HID_REPORT_TYPE_OUTPUT, report_id,
                                               const_cast<uint8_t *>(data.data()), data.size());
  ec = make_ec(err);
  return !ec;
}

bool UsbHost::HidDevice::get_report(uint8_t report_type, uint8_t report_id,
                                    std::span<uint8_t> buffer, size_t &out_length,
                                    std::error_code &ec) {
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  size_t len = buffer.size();
  esp_err_t err =
      hid_class_request_get_report(handle_, report_type, report_id, buffer.data(), &len);
  ec = make_ec(err);
  out_length = ec ? 0 : len;
  return !ec;
}

bool UsbHost::HidDevice::set_idle(uint8_t duration, uint8_t report_id, std::error_code &ec) {
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = hid_class_request_set_idle(handle_, duration, report_id);
  ec = make_ec(err);
  return !ec;
}

bool UsbHost::HidDevice::set_protocol(hid_report_protocol_t protocol, std::error_code &ec) {
  if (!connected_.load()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = hid_class_request_set_protocol(handle_, protocol);
  ec = make_ec(err);
  return !ec;
}

void UsbHost::HidDevice::deliver_input() {
  // Copy the raw report into our buffer, then invoke the user callback.
  size_t len = 0;
  esp_err_t err = hid_host_device_get_raw_input_report_data(handle_, rx_buffer_.data(),
                                                            rx_buffer_.size(), &len);
  if (err != ESP_OK) {
    return;
  }
  input_callback_fn cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = on_input_;
  }
  if (cb) {
    cb(std::span<const uint8_t>(rx_buffer_.data(), len));
  }
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
    deinitialize(ec);
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
  esp_err_t err = usb_host_install(&host_config);
  if (err != ESP_OK) {
    logger_.error("usb_host_install failed: {}", esp_err_to_name(err));
    ec = make_ec(err);
    return false;
  }

  // 2) Spawn the USB-host-library event task.
  lib_task_run_.store(true);
  lib_task_done_.store(false);
  BaseType_t core = config_.task_core_id < 0 ? tskNO_AFFINITY : config_.task_core_id;
  TaskHandle_t task = nullptr;
  BaseType_t created =
      xTaskCreatePinnedToCore(&UsbHost::lib_task_trampoline, "usb_host_lib",
                              config_.task_stack_size, this, config_.task_priority, &task, core);
  if (created != pdPASS) {
    logger_.error("failed to create usb host lib task");
    lib_task_run_.store(false);
    usb_host_uninstall();
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }
  lib_task_handle_ = task;

  // 3) Install the HID class driver (with its own background task).
  const hid_host_driver_config_t hid_config = {
      .create_background_task = true,
      .task_priority = config_.task_priority,
      .stack_size = config_.task_stack_size,
      .core_id = core,
      .callback = &espp_usb_host_driver_event_cb,
      .callback_arg = this,
  };
  err = hid_host_install(&hid_config);
  if (err != ESP_OK) {
    logger_.error("hid_host_install failed: {}", esp_err_to_name(err));
    stop_lib_task(); // join the lib task before uninstalling the library
    usb_host_uninstall();
    ec = make_ec(err);
    return false;
  }

  initialized_.store(true);
  logger_.info("USB host installed");
  ec.clear();
  return true;
}

bool UsbHost::deinitialize(std::error_code &ec) {
  if (!initialized_.load()) {
    ec.clear();
    return true;
  }
  logger_.info("uninstalling USB host");

  // Collect the device handles under the lock, then close them *outside* it: the
  // driver's close path can run callbacks that also take devices_mutex_, so
  // closing while holding it risks lock inversion.
  std::vector<hid_host_device_handle_t> handles;
  {
    std::lock_guard<std::mutex> lk(devices_mutex_);
    handles.reserve(devices_.size());
    for (auto &[handle, dev] : devices_) {
      dev->mark_disconnected();
      handles.push_back(handle);
    }
    devices_.clear();
  }
  for (auto handle : handles) {
    hid_host_device_close(handle);
  }

  // Uninstall the HID class driver (stops its background task).
  esp_err_t err = hid_host_uninstall();
  if (err != ESP_OK) {
    logger_.warn("hid_host_uninstall: {}", esp_err_to_name(err));
  }

  // Free any remaining devices so the library can be uninstalled, then stop +
  // join the lib task (unblocking it so it observes the stop flag promptly
  // rather than relying on a fixed delay).
  usb_host_device_free_all();
  stop_lib_task();

  err = usb_host_uninstall();
  if (err != ESP_OK) {
    logger_.warn("usb_host_uninstall: {}", esp_err_to_name(err));
  }

  initialized_.store(false);
  ec = make_ec(err);
  return !ec;
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

void UsbHost::lib_task_trampoline(void *arg) { static_cast<UsbHost *>(arg)->lib_task(); }

void UsbHost::lib_task() {
  while (lib_task_run_.load()) {
    uint32_t event_flags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      // No registered clients: it is safe to release the devices.
      usb_host_device_free_all();
    }
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      logger_.debug("all USB devices freed");
      if (!lib_task_run_.load()) {
        break;
      }
    }
  }
  lib_task_done_.store(true); // signal stop_lib_task() that we have exited
  vTaskDelete(nullptr);
}

void UsbHost::stop_lib_task() {
  if (lib_task_handle_ == nullptr) {
    return;
  }
  lib_task_run_.store(false);
  // The task blocks in usb_host_lib_handle_events(portMAX_DELAY); unblock it so
  // it observes the stop flag and returns instead of waiting for an event.
  usb_host_lib_unblock();
  // Join: wait (bounded) for the task to actually exit before the caller
  // uninstalls the library out from under it.
  for (int i = 0; i < 100 && !lib_task_done_.load(); ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (!lib_task_done_.load()) {
    logger_.warn("usb host lib task did not exit in time");
  }
  lib_task_handle_ = nullptr;
}

void UsbHost::on_driver_event(hid_host_device_handle_t handle, hid_host_driver_event_t event) {
  if (event != HID_HOST_DRIVER_EVENT_CONNECTED) {
    return;
  }
  HidDevice::Info info = read_info(handle);
  HidDevice::Params params = read_params(handle);
  logger_.info("HID device connected: VID={:#06x} PID={:#06x} iface={} proto={}", info.vid,
               info.pid, params.interface_number, params.protocol);

  if (config_.should_open && !config_.should_open(info, params)) {
    logger_.debug("filter rejected device; not opening");
    return;
  }

  // Open the HID interface, routing its events back to us.
  const hid_host_device_config_t dev_config = {
      .callback = &espp_usb_host_interface_event_cb,
      .callback_arg = this,
  };
  esp_err_t err = hid_host_device_open(handle, &dev_config);
  if (err != ESP_OK) {
    logger_.error("hid_host_device_open failed: {}", esp_err_to_name(err));
    return;
  }

  // Some devices report a boot protocol; force report protocol so we always get
  // the full report-descriptor'd reports (ignore errors -- not all devices
  // support the request).
  hid_class_request_set_protocol(handle, HID_REPORT_PROTOCOL_REPORT);

  auto device = std::shared_ptr<HidDevice>(new HidDevice(handle, config_.max_input_report_size));
  {
    std::lock_guard<std::mutex> lk(devices_mutex_);
    devices_[handle] = device;
  }

  if (config_.auto_start) {
    esp_err_t serr = hid_host_device_start(handle);
    if (serr != ESP_OK) {
      logger_.warn("hid_host_device_start failed: {}", esp_err_to_name(serr));
    } else {
      device->started_.store(true);
    }
  }

  if (config_.on_device_connected) {
    config_.on_device_connected(device);
  }
}

void UsbHost::on_interface_event(hid_host_device_handle_t handle,
                                 hid_host_interface_event_t event) {
  std::shared_ptr<HidDevice> device;
  {
    std::lock_guard<std::mutex> lk(devices_mutex_);
    auto it = devices_.find(handle);
    if (it != devices_.end()) {
      device = it->second;
    }
  }

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    if (device) {
      device->deliver_input();
    }
    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    logger_.info("HID device disconnected");
    if (device) {
      device->mark_disconnected();
    }
    hid_host_device_close(handle);
    {
      std::lock_guard<std::mutex> lk(devices_mutex_);
      devices_.erase(handle);
    }
    if (device && config_.on_device_disconnected) {
      config_.on_device_disconnected(device);
    }
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    logger_.warn("HID transfer error");
    break;
  default:
    break;
  }
}

} // namespace espp
