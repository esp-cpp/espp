#include "usb_host.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
  input_callback_fn cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = on_input_;
  }
  if (cb) {
    cb(data);
  }
}

void UsbHost::HidDevice::retire() {
  // Taking io_mutex_ here waits for any driver call in flight on another task
  // to finish before the interface is closed (and its resources freed).
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (!connected_.exchange(false)) {
    return; // already retired
  }
  hid_host_device_close(handle_);
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
  esp_err_t err = usb_host_install(&host_config);
  if (err != ESP_OK) {
    logger_.error("usb_host_install failed: {}", esp_err_to_name(err));
    ec = make_ec(err);
    return false;
  }

  // 2) Start the USB-host-library event task.
  lib_task_run_.store(true);
  lib_task_ = espp::Task::make_unique({
      .callback = [this](std::mutex &m, std::condition_variable &cv) { return lib_task_fn(m, cv); },
      .task_config =
          {
              .name = "usb_host_lib",
              .stack_size_bytes = config_.lib_task_stack_size,
              .priority = config_.task_priority,
              .core_id = config_.task_core_id,
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

  // 4) Install the HID class driver (with its own background task).
  const hid_host_driver_config_t hid_config = {
      .create_background_task = true,
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
  //    root port so any attached device is reported gone, then wait (bounded)
  //    for the driver to release it and uninstall to succeed.
  usb_host_lib_set_root_port_power(false);
  esp_err_t err = ESP_FAIL;
  for (int i = 0; i < 200; ++i) { // up to ~2 s
    err = hid_host_uninstall();
    if (err == ESP_OK) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
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
    logger_.error("hid_host_uninstall failed: {}{}; root port left powered off, retry "
                  "deinitialize()",
                  esp_err_to_name(err),
                  err == ESP_ERR_INVALID_STATE ? " (the driver still tracks a device)" : "");
    ec = make_ec(err);
    return false;
  }

  // 4) Free any remaining devices so the library can be uninstalled, then stop
  //    + join the lib task and uninstall.
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
bool UsbHost::lib_task_fn(std::mutex & /*m*/, std::condition_variable & /*cv*/) {
  uint32_t event_flags = 0;
  usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
  if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
    // No registered clients: it is safe to release the devices.
    usb_host_device_free_all();
  }
  if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
    logger_.debug("all USB devices freed");
  }
  return !lib_task_run_.load(); // true = stop the task
}

void UsbHost::stop_lib_task() {
  if (!lib_task_) {
    return;
  }
  lib_task_run_.store(false);
  // The task blocks in usb_host_lib_handle_events(portMAX_DELAY); unblock it so
  // it observes the stop flag and returns, then join it.
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
    // The dispatch task retires the device (in order, after any queued inputs).
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
  const hid_host_device_config_t dev_config = {
      .callback = &espp_usb_host_interface_event_cb,
      .callback_arg = this,
  };
  esp_err_t err = hid_host_device_open(handle, &dev_config);
  if (err != ESP_OK) {
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
