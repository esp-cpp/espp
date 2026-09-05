#include "usb_device.hpp"

#include <array>
#include <atomic>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

namespace {

// Only a single USB device exists on the chip; the BOS descriptor and the vendor
// RX / control-request callbacks are global (no user pointer), so we route them
// through a file-scope pointer to the active instance. It is atomic because the
// TinyUSB task reads it concurrently with initialize()/~UsbDevice() writes on
// the caller's thread; each callback loads it ONCE into a local. Teardown
// safety additionally relies on clearing it BEFORE tinyusb_driver_uninstall()
// (which quiesces the TinyUSB task) so no callback can begin using a
// destructing instance.
std::atomic<espp::UsbDevice *> s_device{nullptr};

// The CDC port this component uses. A single dedicated CDC-ACM interface.
constexpr tinyusb_cdcacm_itf_t kCdcPort = TINYUSB_CDC_ACM_0;

// Backpressure tuning shared by write_cdc() and write_vendor() so the two TX
// paths stay consistent. kUsbWriteTimeoutTicks bounds how long a blocking write
// sleep-waits for the host to drain a full TX FIFO before dropping the frame.
// kUsbWriteDrainPollTicks is the poll interval while waiting - never less than
// one tick (pdMS_TO_TICKS(1) is 0 when the tick rate is below 1 kHz, and
// vTaskDelay(0) would not block at all).
constexpr TickType_t kUsbWriteTimeoutTicks = pdMS_TO_TICKS(250);
constexpr TickType_t kUsbWriteDrainPollTicks = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;

// ESP32-S3 / -S2 USB-OTG (DWC2, full-speed) endpoint budget: besides the control
// endpoint EP0, there are ~5 usable data IN endpoints and ~5 usable data OUT
// endpoints. See the README endpoint-budget table for which class combinations
// fit.
constexpr uint8_t kMaxInEndpoints = 5;
constexpr uint8_t kMaxOutEndpoints = 5;

// The MS OS 2.0 descriptor set length used below (fixed by the registry-property
// payload; identical to TinyUSB's webusb_serial example).
constexpr uint16_t kMsOs20DescLen = 0xB2;

// Handle of the task that runs tud_task() (created inside esp_tinyusb's
// tinyusb_driver_install(); esp_tinyusb does not expose it). Every TinyUSB
// class/descriptor callback below runs on that task, so each one records the
// current task handle here before dispatching. write_vendor() uses it to detect
// that it is being called from TinyUSB-callback context (e.g. from inside a
// receive callback), where sleep-waiting for the TX FIFO to drain would block
// the very task that processes the TX-complete events doing the draining.
std::atomic<TaskHandle_t> s_tinyusb_task{nullptr};

void note_tinyusb_task() {
  s_tinyusb_task.store(xTaskGetCurrentTaskHandle(), std::memory_order_relaxed);
}

bool on_tinyusb_task() {
  return xTaskGetCurrentTaskHandle() == s_tinyusb_task.load(std::memory_order_relaxed);
}

} // namespace

namespace espp {

// Storage for the descriptors that TinyUSB references by pointer for the lifetime
// of the driver. These must outlive tinyusb_driver_install().
struct UsbDevice::Impl {
  tusb_desc_device_t device_desc{};
  std::vector<uint8_t> config_desc;    // full-speed configuration (64-byte bulk)
  std::vector<uint8_t> hs_config_desc; // high-speed configuration (512-byte bulk), HS builds only
  tusb_desc_device_qualifier_t qualifier_desc{}; // device qualifier, HS builds only
  std::vector<uint8_t> bos_desc;                 // BOS (WebUSB + MS OS 2.0), empty if unused
  std::vector<uint8_t> ms_os_20_desc;            // MS OS 2.0 descriptor set, empty if unused
  std::vector<uint8_t> webusb_url_desc;          // WebUSB URL descriptor, empty if unused
  std::vector<uint8_t> hid_report_desc;          // HID report descriptor bytes, empty if unused

  // Owning strings + the pointer table TinyUSB reads (index 0 is the LANGID).
  std::array<uint8_t, 2> langid{{0x09, 0x04}};
  std::vector<std::string> owned_strings;
  std::vector<const char *> strings;

  // Allocated interface / endpoint identifiers, filled in during initialize().
  uint8_t vendor_itf{0xFF};
  uint8_t hid_itf{0xFF};
};

UsbDevice *UsbDevice::instance() { return s_device; }

UsbDevice::UsbDevice(const Config &config)
    : BaseComponent("UsbDevice", config.log_level)
    , impl_(std::make_unique<Impl>())
    , config_(config)
    , on_cdc_receive_(config.cdc ? config.cdc->on_receive : nullptr)
    , on_vendor_receive_(config.vendor ? config.vendor->on_receive : nullptr) {}

UsbDevice::~UsbDevice() {
  if (initialized_) {
    // Detach the global callback routing BEFORE tearing down the driver so a
    // TinyUSB callback that fires during deinit cannot dereference this
    // destructing instance (use-after-free). Atomic compare_exchange so the
    // clear only happens if we still own the slot (mirrors the claim in
    // initialize()).
    UsbDevice *expected = this;
    s_device.compare_exchange_strong(expected, nullptr);
    if (config_.cdc)
      tinyusb_cdcacm_deinit(kCdcPort);
    tinyusb_driver_uninstall();
    initialized_ = false;
  }
}

// ---------------------------------------------------------------------------
// TinyUSB C callbacks (global; routed to the active instance).
// ---------------------------------------------------------------------------

// CDC RX trampoline registered with esp_tinyusb; runs in the TinyUSB task.
static void cdc_rx_trampoline(int itf, cdcacm_event_t *event) {
  (void)event;
  note_tinyusb_task();
  if (itf != (int)kCdcPort)
    return;
  // load once: the pointer must not be re-read between check and use
  auto *dev = s_device.load();
  if (dev)
    dev->handle_cdc_rx();
}

extern "C" {

// BOS descriptor (weak in TinyUSB core). Returns our WebUSB/MS-OS BOS when the
// vendor+WebUSB function is enabled, otherwise NULL (no BOS).
uint8_t const *tud_descriptor_bos_cb(void) {
  note_tinyusb_task();
  auto *dev = s_device.load();
  return dev ? dev->bos_descriptor() : nullptr;
}

#if (CFG_TUD_VENDOR > 0)

// Vendor RX callback: drain the FIFO and dispatch to the user callback.
#if CFG_TUD_API_V0_19_COMPAT
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize) {
#else
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint32_t bufsize) {
#endif
  (void)itf;
  note_tinyusb_task();
  // The FIFO variant calls this with buffer==NULL, bufsize==0 (drain via
  // tud_vendor_read); the zero-copy variant passes the received bytes directly.
  auto *dev = s_device.load();
  if (dev)
    dev->handle_vendor_rx(buffer, static_cast<size_t>(bufsize));
}

// Vendor control-transfer callback: answer the WebUSB URL and MS OS 2.0
// descriptor requests, and the WebUSB "connect" class request (0x22).
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  note_tinyusb_task();
  if (stage != CONTROL_STAGE_SETUP)
    return true; // nothing to do on DATA / ACK stages
  auto *dev = s_device.load();
  if (!dev || !dev->vendor_config().has_value())
    return false;
  const auto &vendor = *dev->vendor_config();

  switch (request->bmRequestType_bit.type) {
  case TUSB_REQ_TYPE_VENDOR:
    // wIndex 2 == WEBUSB_REQUEST_GET_URL; qualifying on it keeps this branch
    // from shadowing the MS-OS request if the two vendor codes are configured
    // to the same value.
    if (request->bRequest == vendor.webusb_vendor_code && request->wIndex == 2) {
      // Return the WebUSB landing-page URL descriptor.
      uint8_t len = 0;
      const uint8_t *url = dev->webusb_url_descriptor(len);
      if (!url)
        return false;
      return tud_control_xfer(rhport, request, (void *)(uintptr_t)url, len);
    }
    if (request->bRequest == vendor.ms_os_vendor_code && request->wIndex == 7) {
      // Return the MS OS 2.0 descriptor set.
      uint16_t total_len = 0;
      const uint8_t *ms = dev->ms_os_20_descriptor(total_len);
      if (!ms)
        return false;
      return tud_control_xfer(rhport, request, (void *)(uintptr_t)ms, total_len);
    }
    return false;

  case TUSB_REQ_TYPE_CLASS:
    if (request->bRequest == 0x22) {
      // WebUSB simulates CDC SET_CONTROL_LINE_STATE (0x22) to signal connect.
      if (request->wValue == 0)
        tud_vendor_write_clear();
      return tud_control_status(rhport, request);
    }
    return false;

  default:
    return false;
  }
}

#endif // CFG_TUD_VENDOR > 0

// NOTE: the TinyUSB device lifecycle callbacks (tud_mount_cb / tud_umount_cb /
// tud_suspend_cb / tud_resume_cb) are defined by esp_tinyusb itself, which
// forwards them to the tinyusb_config_t::event_cb we register in initialize().
// Do NOT define tud_umount_cb here -- it would be a duplicate symbol. The
// unmount TX-FIFO clear + the app mount/unmount hooks live in the handlers
// below, driven by this event callback.
// cppcheck-suppress constParameterCallback // signature must match tinyusb_event_cb_t
extern "C" void espp_usb_device_event_cb(tinyusb_event_t *event, void *arg) {
  auto *dev = static_cast<espp::UsbDevice *>(arg);
  if (!dev || !event)
    return;
  if (event->id == TINYUSB_EVENT_ATTACHED)
    dev->handle_usb_mount();
  else if (event->id == TINYUSB_EVENT_DETACHED)
    dev->handle_usb_unmount();
}

#if (CFG_TUD_HID > 0)

// HID: return the application-supplied report descriptor for the given instance.
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  (void)instance;
  auto *dev = s_device.load();
  return dev ? dev->hid_report_descriptor() : nullptr;
}

// HID GET_REPORT control request: this device is input-only, so nothing to do.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}

// HID SET_REPORT control request (and OUT endpoint data): unused / ignored.
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)bufsize;
}

#endif // CFG_TUD_HID > 0

} // extern "C"

// ---------------------------------------------------------------------------
// Descriptor accessors used by the global callbacks.
// ---------------------------------------------------------------------------

const uint8_t *UsbDevice::bos_descriptor() const {
  return impl_->bos_desc.empty() ? nullptr : impl_->bos_desc.data();
}

const uint8_t *UsbDevice::ms_os_20_descriptor(uint16_t &total_len) const {
  if (impl_->ms_os_20_desc.empty())
    return nullptr;
  total_len = static_cast<uint16_t>(impl_->ms_os_20_desc.size());
  return impl_->ms_os_20_desc.data();
}

const uint8_t *UsbDevice::webusb_url_descriptor(uint8_t &length) const {
  if (impl_->webusb_url_desc.empty())
    return nullptr;
  length = static_cast<uint8_t>(impl_->webusb_url_desc.size());
  return impl_->webusb_url_desc.data();
}

const uint8_t *UsbDevice::hid_report_descriptor() const {
  return impl_->hid_report_desc.empty() ? nullptr : impl_->hid_report_desc.data();
}

// ---------------------------------------------------------------------------
// RX handling.
// ---------------------------------------------------------------------------

void UsbDevice::handle_cdc_rx() {
#if (CFG_TUD_CDC > 0)
  receive_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_cdc_receive_;
  }
  if (!config_.cdc)
    return;
  // NOTE: even with no callback attached we still drain (and discard) the FIFO
  // below; leaving bytes in it would back-pressure/stall the host.
  std::vector<uint8_t> &buf = cdc_rx_buf_;
  size_t rx_size = 0;
  do {
    rx_size = 0;
    esp_err_t err = tinyusb_cdcacm_read(kCdcPort, buf.data(), buf.size(), &rx_size);
    if (err != ESP_OK) {
      logger_.error("CDC read error: {}", esp_err_to_name(err));
      break;
    }
    if (rx_size > 0 && cb)
      cb(std::span<const uint8_t>(buf.data(), rx_size));
  } while (rx_size == buf.size());
#endif
}

void UsbDevice::handle_vendor_rx(const uint8_t *buffer, size_t bufsize) {
#if (CFG_TUD_VENDOR > 0)
  receive_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_vendor_receive_;
  }
  if (!config_.vendor)
    return;
  // TinyUSB zero-copy RX variant (CFG_TUD_VENDOR_RX_BUFSIZE==0): the received
  // bytes are delivered directly via the callback buffer and are NOT in a FIFO,
  // so dispatch them here. Otherwise (FIFO variant, the esp_tinyusb default)
  // buffer is null and we drain the FIFO via tud_vendor_read(). With no
  // callback attached, bytes are still consumed (discarded) so the FIFO cannot
  // fill up and stall the host.
  if (buffer != nullptr && bufsize > 0) {
    if (cb)
      cb(std::span<const uint8_t>(buffer, bufsize));
    return;
  }
  std::vector<uint8_t> &buf = vendor_rx_buf_;
  while (tud_vendor_available()) {
    uint32_t count = tud_vendor_read(buf.data(), buf.size());
    if (count == 0)
      break;
    if (cb)
      cb(std::span<const uint8_t>(buf.data(), count));
  }
#endif
}

// ---------------------------------------------------------------------------
// Initialization: build descriptors from the selected functions.
// ---------------------------------------------------------------------------

bool UsbDevice::initialize(std::error_code &ec) {
  ec.clear();
  if (initialized_) {
    logger_.warn("Already initialized");
    return true;
  }
  // Fast-fail when another instance is already active. This check alone is
  // check-then-act racy; the AUTHORITATIVE claim is the compare_exchange just
  // before tinyusb_driver_install() below.
  if (s_device != nullptr) {
    logger_.error("Another UsbDevice/UsbCdc instance is already active");
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }
  if (!config_.cdc && !config_.vendor && !config_.hid) {
    logger_.error("No USB function enabled (enable cdc, vendor and/or hid)");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (config_.msc) {
    // Reserved extension point; not implemented yet (see README endpoint table).
    logger_.error("MSC function is not implemented yet");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
  }
  if (config_.vendor) {
#if (CFG_TUD_VENDOR == 0)
    logger_.error("Vendor function requested but CFG_TUD_VENDOR==0. Set "
                  "CONFIG_TINYUSB_VENDOR_COUNT>0 in sdkconfig.");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  }
  if (config_.hid) {
#if (CFG_TUD_HID == 0)
    logger_.error("HID function requested but CFG_TUD_HID==0. Set "
                  "CONFIG_TINYUSB_HID_COUNT>0 in sdkconfig.");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#else
    if (config_.hid->report_descriptor.empty()) {
      // A default-constructed HidFunction has no report descriptor; proceeding
      // would emit a HID interface with wDescriptorLength == 0 (and a null
      // report callback) -- an invalid HID interface that "succeeds" here and
      // then confuses the host. Reject it as an invalid configuration.
      logger_.error("HID function enabled but report_descriptor is empty; supply the HID "
                    "report descriptor bytes (e.g. built with the hid-rp component).");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
#endif
  }

  // A zero-length RX scratch buffer would make the RX drain loops spin without
  // making progress (e.g. handle_cdc_rx()'s `while (rx_size == buf.size())`
  // becomes `while (0 == 0)`), so require a positive chunk size.
  if (config_.cdc && config_.cdc->rx_chunk_size == 0) {
    logger_.error("CDC rx_chunk_size must be > 0");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (config_.vendor && config_.vendor->rx_chunk_size == 0) {
    logger_.error("Vendor rx_chunk_size must be > 0");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  // --- Sequentially allocate interface numbers, endpoint addresses, strings ---
  uint8_t next_itf = 0;
  uint8_t next_ep = 1; // endpoint number (1..); IN uses 0x80|n, OUT uses n
  uint8_t in_used = 0, out_used = 0;

  // Preallocate the RX scratch buffers now so the TinyUSB-task RX handlers never
  // allocate on the hot path.
  if (config_.cdc)
    cdc_rx_buf_.assign(config_.cdc->rx_chunk_size, 0);
  if (config_.vendor)
    vendor_rx_buf_.assign(config_.vendor->rx_chunk_size, 0);

  // String table: 0=LANGID, 1=manufacturer, 2=product, 3=serial, then per-itf.
  impl_->owned_strings = {config_.manufacturer, config_.product, config_.serial_number};
  uint8_t next_str = 4;

  uint8_t cdc_itf = 0, cdc_str = 0, cdc_notif = 0, cdc_out = 0, cdc_in = 0;
  if (config_.cdc) {
    cdc_itf = next_itf;
    next_itf = static_cast<uint8_t>(next_itf + 2); // comm + data interfaces
    cdc_str = next_str++;
    impl_->owned_strings.push_back(config_.cdc->interface_name);
    cdc_notif = static_cast<uint8_t>(0x80 | next_ep++); // interrupt IN (notification)
    in_used++;
    const uint8_t data_ep = next_ep++;
    cdc_out = data_ep;                             // bulk OUT
    cdc_in = static_cast<uint8_t>(0x80 | data_ep); // bulk IN
    in_used++;
    out_used++;
  }

  uint8_t vendor_itf = 0, vendor_str = 0, vendor_out = 0, vendor_in = 0;
  if (config_.vendor) {
    vendor_itf = next_itf++;
    vendor_str = next_str++;
    impl_->owned_strings.push_back(config_.vendor->interface_name);
    const uint8_t v_ep = next_ep++;
    vendor_out = v_ep;                             // bulk OUT
    vendor_in = static_cast<uint8_t>(0x80 | v_ep); // bulk IN
    in_used++;
    out_used++;
    impl_->vendor_itf = vendor_itf;
  }

  uint8_t hid_itf = 0, hid_str = 0, hid_in = 0, hid_out = 0;
  if (config_.hid) {
    hid_itf = next_itf++;
    hid_str = next_str++;
    impl_->owned_strings.push_back(config_.hid->interface_name);
    const uint8_t h_ep = next_ep++;
    hid_in = static_cast<uint8_t>(0x80 | h_ep); // interrupt IN
    in_used++;
    if (config_.hid->has_out_endpoint) {
      hid_out = h_ep; // interrupt OUT (shares the endpoint number with IN)
      out_used++;
    }
    impl_->hid_itf = hid_itf;
    // Keep our own copy of the report descriptor alive for the driver lifetime.
    impl_->hid_report_desc = config_.hid->report_descriptor;
  }

  // --- Endpoint budget check ---
  if (in_used > kMaxInEndpoints || out_used > kMaxOutEndpoints) {
    logger_.error("Endpoint budget exceeded: IN={} (max {}), OUT={} (max {})", in_used,
                  kMaxInEndpoints, out_used, kMaxOutEndpoints);
    ec = std::make_error_code(std::errc::value_too_large);
    return false;
  }

  // --- Build the string pointer table TinyUSB reads ---
  impl_->strings.clear();
  impl_->strings.push_back(reinterpret_cast<const char *>(impl_->langid.data()));
  for (const auto &s : impl_->owned_strings)
    impl_->strings.push_back(s.c_str());

  // --- Device descriptor ---
  const bool webusb = config_.vendor && config_.vendor->webusb;
  impl_->device_desc = tusb_desc_device_t{};
  impl_->device_desc.bLength = sizeof(tusb_desc_device_t);
  impl_->device_desc.bDescriptorType = TUSB_DESC_DEVICE;
  // BOS/WebUSB requires bcdUSB >= 2.1.
  impl_->device_desc.bcdUSB = webusb ? 0x0210 : 0x0200;
  // Advertise the IAD-based composite class (0xEF/0x02/0x01) only when CDC is
  // enabled, since CDC is the function that emits an Interface Association
  // Descriptor. For a vendor-only and/or HID-only device there is no IAD, so use
  // 0x00/0x00/0x00 and let the interface descriptors declare the class(es).
  if (config_.cdc) {
    impl_->device_desc.bDeviceClass = TUSB_CLASS_MISC;
    impl_->device_desc.bDeviceSubClass = MISC_SUBCLASS_COMMON;
    impl_->device_desc.bDeviceProtocol = MISC_PROTOCOL_IAD;
  } else {
    impl_->device_desc.bDeviceClass = 0x00;
    impl_->device_desc.bDeviceSubClass = 0x00;
    impl_->device_desc.bDeviceProtocol = 0x00;
  }
  impl_->device_desc.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE;
  impl_->device_desc.idVendor = config_.vid;
  impl_->device_desc.idProduct = config_.pid;
  impl_->device_desc.bcdDevice = 0x0100;
  impl_->device_desc.iManufacturer = 0x01;
  impl_->device_desc.iProduct = 0x02;
  impl_->device_desc.iSerialNumber = 0x03;
  impl_->device_desc.bNumConfigurations = 0x01;

  // --- Configuration descriptor ---
  uint8_t itf_count = 0;
  uint16_t total_len = TUD_CONFIG_DESC_LEN;
  if (config_.cdc) {
    itf_count = static_cast<uint8_t>(itf_count + 2);
    total_len = static_cast<uint16_t>(total_len + TUD_CDC_DESC_LEN);
  }
  if (config_.vendor) {
    itf_count = static_cast<uint8_t>(itf_count + 1);
    total_len = static_cast<uint16_t>(total_len + TUD_VENDOR_DESC_LEN);
  }
  if (config_.hid) {
    itf_count = static_cast<uint8_t>(itf_count + 1);
    total_len = static_cast<uint16_t>(
        total_len + (config_.hid->has_out_endpoint ? TUD_HID_INOUT_DESC_LEN : TUD_HID_DESC_LEN));
  }

  // Build one configuration descriptor for a given bus speed. Bulk endpoints
  // are 64 bytes at full speed and 512 at high speed; the HID interrupt
  // bInterval is in 1-ms frames at FS but exponent-encoded (2^(n-1) x 125 us
  // microframes) at HS. On HS-capable parts (e.g. ESP32-P4) BOTH descriptors
  // are installed so the device is valid whichever speed the host negotiates.
  auto build_config_desc = [&](std::vector<uint8_t> &desc, int bulk_ep_size,
                               uint8_t hid_binterval) {
    desc.clear();
    auto append = [&](const uint8_t *p, size_t n) { desc.insert(desc.end(), p, p + n); };
    {
      const uint8_t hdr[] = {
          // config number, interface count, string index, total length, attribute, power (mA)
          TUD_CONFIG_DESCRIPTOR(1, itf_count, 0, total_len, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
                                100),
      };
      append(hdr, sizeof(hdr));
    }
    if (config_.cdc) {
      const uint8_t d[] = {
          TUD_CDC_DESCRIPTOR(cdc_itf, cdc_str, cdc_notif, 8, cdc_out, cdc_in, bulk_ep_size),
      };
      append(d, sizeof(d));
    }
    if (config_.vendor) {
      const uint8_t d[] = {
          TUD_VENDOR_DESCRIPTOR(vendor_itf, vendor_str, vendor_out, vendor_in, bulk_ep_size),
      };
      append(d, sizeof(d));
    }
    if (config_.hid) {
      const uint16_t report_len = static_cast<uint16_t>(impl_->hid_report_desc.size());
      // Interrupt endpoints are <=64 byte packets at either speed; a 64-byte
      // endpoint buffer comfortably fits the gamepad report.
      constexpr uint8_t kHidEpSize = 64;
      if (config_.hid->has_out_endpoint) {
        // NOTE: TinyUSB's parameter order here is (..., _epout, _epin, ...) --
        // OUT before IN (see TUD_HID_INOUT_DESCRIPTOR in usbd.h). hid_out is
        // the plain endpoint number and hid_in carries the 0x80 direction bit.
        const uint8_t d[] = {
            TUD_HID_INOUT_DESCRIPTOR(hid_itf, hid_str, HID_ITF_PROTOCOL_NONE, report_len, hid_out,
                                     hid_in, kHidEpSize, hid_binterval),
        };
        append(d, sizeof(d));
      } else {
        const uint8_t d[] = {
            TUD_HID_DESCRIPTOR(hid_itf, hid_str, HID_ITF_PROTOCOL_NONE, report_len, hid_in,
                               kHidEpSize, hid_binterval),
        };
        append(d, sizeof(d));
      }
    }
  };

  const uint8_t hid_poll_ms = config_.hid ? config_.hid->poll_interval_ms : 0;
  // Full-speed configuration: 64-byte bulk endpoints, bInterval in ms frames.
  build_config_desc(impl_->config_desc, 64, hid_poll_ms);
#if (TUD_OPT_HIGH_SPEED)
  // High-speed configuration: 512-byte bulk endpoints; HID bInterval is the
  // exponent n in 2^(n-1) x 125 us microframes. Choose the largest n whose
  // period does not exceed the requested ms (i.e. poll at least as often).
  uint8_t hs_hid_binterval = 1;
  {
    const uint32_t microframes = static_cast<uint32_t>(hid_poll_ms) * 8; // 125 us units
    while (hs_hid_binterval < 16 && (1u << hs_hid_binterval) <= microframes)
      ++hs_hid_binterval; // exits with 2^(n-1) <= microframes < 2^n
  }
  build_config_desc(impl_->hs_config_desc, 512, hs_hid_binterval);
  // Device qualifier: required for a high-speed-capable device so the host can
  // query the other-speed characteristics.
  impl_->qualifier_desc = {
      .bLength = sizeof(tusb_desc_device_qualifier_t),
      .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
      .bcdUSB = impl_->device_desc.bcdUSB,
      .bDeviceClass = impl_->device_desc.bDeviceClass,
      .bDeviceSubClass = impl_->device_desc.bDeviceSubClass,
      .bDeviceProtocol = impl_->device_desc.bDeviceProtocol,
      .bMaxPacketSize0 = impl_->device_desc.bMaxPacketSize0,
      .bNumConfigurations = 1,
      .bReserved = 0,
  };
#endif

  // --- WebUSB / MS OS 2.0 descriptors (only when the vendor+WebUSB is enabled) ---
  if (webusb) {
    const auto &v = *config_.vendor;

    // The WebUSB URL descriptor encodes its total length in a single byte
    // (bLength = 3 header bytes + URL bytes). Reject a URL that would overflow
    // that byte and produce an invalid descriptor (which can break enumeration).
    if (v.landing_page_url.size() > (0xFF - 3)) {
      logger_.error("WebUSB landing_page_url too long ({} bytes); max {} so bLength (3+url) fits a "
                    "uint8_t",
                    v.landing_page_url.size(), 0xFF - 3);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }

    // WebUSB URL descriptor: bLength, bDescriptorType(3), bScheme, url...
    impl_->webusb_url_desc.clear();
    impl_->webusb_url_desc.push_back(static_cast<uint8_t>(3 + v.landing_page_url.size()));
    impl_->webusb_url_desc.push_back(3); // WEBUSB URL descriptor type
    impl_->webusb_url_desc.push_back(v.url_scheme);
    impl_->webusb_url_desc.insert(impl_->webusb_url_desc.end(), v.landing_page_url.begin(),
                                  v.landing_page_url.end());

    // MS OS 2.0 descriptor set (identical layout to TinyUSB's webusb example, with
    // the function-subset "first interface" byte set to our vendor interface).
    const uint8_t ms_os_20[] = {
        // Set header: length, type, windows version, total length
        U16_TO_U8S_LE(0x000A),
        U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
        U32_TO_U8S_LE(0x06030000),
        U16_TO_U8S_LE(kMsOs20DescLen),
        // Configuration subset header: length, type, config index, reserved, total length
        U16_TO_U8S_LE(0x0008),
        U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
        0,
        0,
        U16_TO_U8S_LE(kMsOs20DescLen - 0x0A),
        // Function subset header: length, type, first interface, reserved, subset length
        U16_TO_U8S_LE(0x0008),
        U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
        vendor_itf,
        0,
        U16_TO_U8S_LE(kMsOs20DescLen - 0x0A - 0x08),
        // MS OS 2.0 compatible ID: length, type, compatible ID, sub compatible ID
        U16_TO_U8S_LE(0x0014),
        U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
        'W',
        'I',
        'N',
        'U',
        'S',
        'B',
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        // MS OS 2.0 registry property: length, type
        U16_TO_U8S_LE(kMsOs20DescLen - 0x0A - 0x08 - 0x08 - 0x14),
        U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
        U16_TO_U8S_LE(0x0007),
        U16_TO_U8S_LE(0x002A),
        'D',
        0x00,
        'e',
        0x00,
        'v',
        0x00,
        'i',
        0x00,
        'c',
        0x00,
        'e',
        0x00,
        'I',
        0x00,
        'n',
        0x00,
        't',
        0x00,
        'e',
        0x00,
        'r',
        0x00,
        'f',
        0x00,
        'a',
        0x00,
        'c',
        0x00,
        'e',
        0x00,
        'G',
        0x00,
        'U',
        0x00,
        'I',
        0x00,
        'D',
        0x00,
        's',
        0x00,
        0x00,
        0x00,
        U16_TO_U8S_LE(0x0050),
        // bPropertyData: "{975F44D9-0D08-43FD-8B3E-127CA8AFFF9D}"
        '{',
        0x00,
        '9',
        0x00,
        '7',
        0x00,
        '5',
        0x00,
        'F',
        0x00,
        '4',
        0x00,
        '4',
        0x00,
        'D',
        0x00,
        '9',
        0x00,
        '-',
        0x00,
        '0',
        0x00,
        'D',
        0x00,
        '0',
        0x00,
        '8',
        0x00,
        '-',
        0x00,
        '4',
        0x00,
        '3',
        0x00,
        'F',
        0x00,
        'D',
        0x00,
        '-',
        0x00,
        '8',
        0x00,
        'B',
        0x00,
        '3',
        0x00,
        'E',
        0x00,
        '-',
        0x00,
        '1',
        0x00,
        '2',
        0x00,
        '7',
        0x00,
        'C',
        0x00,
        'A',
        0x00,
        '8',
        0x00,
        'A',
        0x00,
        'F',
        0x00,
        'F',
        0x00,
        'F',
        0x00,
        '9',
        0x00,
        'D',
        0x00,
        '}',
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    };
    static_assert(sizeof(ms_os_20) == kMsOs20DescLen, "MS OS 2.0 descriptor size mismatch");
    impl_->ms_os_20_desc.assign(ms_os_20, ms_os_20 + sizeof(ms_os_20));

    // BOS descriptor: WebUSB + MS OS 2.0 platform capabilities.
    const uint16_t bos_total =
        TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN;
    const uint8_t bos[] = {
        TUD_BOS_DESCRIPTOR(bos_total, 2),
        TUD_BOS_WEBUSB_DESCRIPTOR(v.webusb_vendor_code, 1),
        TUD_BOS_MS_OS_20_DESCRIPTOR(kMsOs20DescLen, v.ms_os_vendor_code),
    };
    impl_->bos_desc.assign(bos, bos + sizeof(bos));
  }

  // --- Install the TinyUSB driver with our descriptors ---
  tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
  tusb_cfg.descriptor.device = &impl_->device_desc;
  tusb_cfg.descriptor.string = impl_->strings.data();
  tusb_cfg.descriptor.string_count = static_cast<int>(impl_->strings.size());
  tusb_cfg.descriptor.full_speed_config = impl_->config_desc.data();
#if (TUD_OPT_HIGH_SPEED)
  tusb_cfg.descriptor.high_speed_config = impl_->hs_config_desc.data();
  tusb_cfg.descriptor.qualifier = &impl_->qualifier_desc;
#endif

  // Route esp_tinyusb's device lifecycle events (mount / unmount) to us so we
  // can clear the TX FIFOs on unmount and invoke any app-registered callbacks.
  tusb_cfg.event_cb = espp_usb_device_event_cb;
  tusb_cfg.event_arg = this;

  // Register before installing so the BOS / vendor callbacks can find us.
  // Claim the singleton slot ATOMICALLY: the null check at the top of
  // initialize() is only a fast-fail, so two threads (or two instances) that
  // both passed it must be arbitrated here — exactly one compare_exchange
  // wins and installs the driver; the loser backs out with "busy".
  UsbDevice *expected = nullptr;
  if (!s_device.compare_exchange_strong(expected, this)) {
    logger_.error("Another UsbDevice/UsbCdc instance is already active");
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }

  esp_err_t err = tinyusb_driver_install(&tusb_cfg);
  if (err != ESP_OK) {
    logger_.error("tinyusb_driver_install failed: {}", esp_err_to_name(err));
    s_device = nullptr;
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  // --- Initialize the CDC-ACM function (vendor needs no explicit init) ---
  if (config_.cdc) {
    tinyusb_config_cdcacm_t acm_cfg = {};
    acm_cfg.cdc_port = kCdcPort;
    acm_cfg.callback_rx = &cdc_rx_trampoline;
    acm_cfg.callback_rx_wanted_char = nullptr;
    acm_cfg.callback_line_state_changed = nullptr;
    acm_cfg.callback_line_coding_changed = nullptr;
    err = tinyusb_cdcacm_init(&acm_cfg);
    if (err != ESP_OK) {
      logger_.error("tinyusb_cdcacm_init failed: {}", esp_err_to_name(err));
      s_device = nullptr;
      tinyusb_driver_uninstall();
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
  }

  initialized_ = true;
  logger_.info(
      "Initialized native USB device (VID=0x{:04x} PID=0x{:04x}) cdc={} vendor={} hid={}{}",
      config_.vid, config_.pid, config_.cdc.has_value(), config_.vendor.has_value(),
      config_.hid.has_value(), webusb ? " webusb" : "");
  return true;
}

// ---------------------------------------------------------------------------
// Write paths.
// ---------------------------------------------------------------------------

bool UsbDevice::write_cdc(std::span<const uint8_t> data, std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_CDC > 0)
  if (!initialized_ || !config_.cdc) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  // A frame is written ALL-OR-NOTHING when it fits in the TX FIFO
  // (CONFIG_TINYUSB_CDC_TX_BUFSIZE): we wait (bounded) for room for the WHOLE
  // frame and only then enqueue it in a SINGLE write, so a drain-timeout or a
  // mid-write disconnect can never leave a truncated prefix on the wire (a
  // partial frame is useless to the host - its parser discards it on the
  // length/CRC check). Uses the raw tud_cdc_n_* API (not the esp_tinyusb TX
  // ringbuffer) so the whole frame can be sized up front, exactly as
  // write_vendor() uses tud_vendor_*.
  //
  // In TinyUSB-callback context (e.g. from inside a receive callback, which is
  // dispatched on the TinyUSB task) we cannot wait for a drain: tud_task() is
  // below us on this very stack, so the TX-complete events that refill the
  // endpoint cannot be processed while we sleep. There we fail fast if the whole
  // frame does not ALREADY fit, again without enqueueing anything.
  //
  // A frame LARGER than the whole TX FIFO cannot be enqueued atomically, so it
  // is streamed across drains and is NOT all-or-nothing (a mid-stream timeout
  // may leave a prefix on the wire). Keep framed payloads within the FIFO for
  // atomic writes.
  const bool in_tinyusb_task = on_tinyusb_task();
  const TickType_t start_tick = xTaskGetTickCount();

  if (data.size() <= CFG_TUD_CDC_TX_BUFSIZE) {
    // Atomic path: wait until the whole frame fits, then write it in one shot.
    while (tud_cdc_n_write_available(kCdcPort) < data.size()) {
      if (in_tinyusb_task) {
        logger_.warn_rate_limited("CDC TX FIFO cannot hold the whole {}-byte frame in "
                                  "TinyUSB-callback context (cannot wait for a drain here), "
                                  "dropping it - send from a separate task instead",
                                  data.size());
        ec = std::make_error_code(std::errc::no_buffer_space);
        return false;
      }
      // Host closing the port (DTR cleared) is a different condition from
      // backpressure: report not_connected so callers do not treat it like a
      // full FIFO.
      if (!tud_cdc_n_connected(kCdcPort)) {
        logger_.warn_rate_limited("CDC host disconnected before a {}-byte frame could be sent",
                                  data.size());
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      // Unsigned tick subtraction stays correct across tick-count wraparound.
      if ((xTaskGetTickCount() - start_tick) >= kUsbWriteTimeoutTicks) {
        logger_.warn_rate_limited("CDC TX FIFO full, dropping a {}-byte frame", data.size());
        ec = std::make_error_code(std::errc::no_buffer_space);
        return false;
      }
      vTaskDelay(kUsbWriteDrainPollTicks);
    }
    // Room for the whole frame is guaranteed, so this single write takes all of
    // it - no prefix/truncation is possible.
    tud_cdc_n_write(kCdcPort, data.data(), data.size());
    tud_cdc_n_write_flush(kCdcPort);
    return true;
  }

  // Streaming path: frame larger than the FIFO (NOT atomic - see note above).
  size_t offset = 0;
  while (offset < data.size()) {
    uint32_t queued = tud_cdc_n_write(kCdcPort, data.data() + offset, data.size() - offset);
    tud_cdc_n_write_flush(kCdcPort);
    offset += queued;
    if (queued == 0) {
      if (in_tinyusb_task) {
        logger_.warn_rate_limited("CDC TX buffer full in TinyUSB-callback context (cannot wait "
                                  "for a drain here), dropping {} bytes",
                                  data.size() - offset);
        ec = std::make_error_code(std::errc::no_buffer_space);
        break;
      }
      if (!tud_cdc_n_connected(kCdcPort)) {
        logger_.warn_rate_limited("CDC host disconnected mid-write, dropping {} bytes",
                                  data.size() - offset);
        ec = std::make_error_code(std::errc::not_connected);
        break;
      }
      if ((xTaskGetTickCount() - start_tick) >= kUsbWriteTimeoutTicks) {
        logger_.warn_rate_limited("CDC TX buffer full, dropping {} bytes", data.size() - offset);
        ec = std::make_error_code(std::errc::no_buffer_space);
        break;
      }
      vTaskDelay(kUsbWriteDrainPollTicks);
    }
  }
  return offset == data.size();
#else
  (void)data;
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

bool UsbDevice::write_cdc(std::span<const uint8_t> data) {
  std::error_code ec;
  return write_cdc(data, ec);
}

bool UsbDevice::write_vendor(std::span<const uint8_t> data, std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_VENDOR > 0)
  if (!initialized_ || !config_.vendor) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  // Same all-or-nothing contract as write_cdc(): a frame that fits in the vendor
  // TX FIFO (CONFIG_TINYUSB_VENDOR_TX_BUFSIZE) is written atomically - wait
  // (bounded) for room for the WHOLE frame, then enqueue it in a SINGLE write,
  // so a drain-timeout or a mid-write unmount can never leave a truncated prefix
  // on the wire (a partial frame is useless to the host - its parser discards it
  // on the length/CRC check).
  //
  // In TinyUSB-callback context (e.g. from inside a receive callback, dispatched
  // on the TinyUSB task) we cannot wait for a drain: tud_task() is below us on
  // this very stack, so the TX-complete events that refill the endpoint cannot
  // run while we sleep. There we fail fast if the whole frame does not ALREADY
  // fit, again without enqueueing anything.
  //
  // A frame LARGER than the whole TX FIFO cannot be enqueued atomically, so it
  // is streamed across drains and is NOT all-or-nothing (a mid-stream timeout
  // may leave a prefix on the wire). Keep framed payloads within the FIFO for
  // atomic writes.
  const bool in_tinyusb_task = on_tinyusb_task();
  static constexpr TickType_t kVendorWriteTimeoutTicks = pdMS_TO_TICKS(250);
  // Poll at ~1 ms, but never less than one tick (pdMS_TO_TICKS(1) is 0 when
  // the tick rate is below 1 kHz, and vTaskDelay(0) would not block at all).
  static constexpr TickType_t kVendorDrainPollTicks = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
  const TickType_t start_tick = xTaskGetTickCount();

  if (data.size() <= CFG_TUD_VENDOR_TX_BUFSIZE) {
    // Atomic path: wait until the whole frame fits, then write it in one shot.
    while (tud_vendor_write_available() < data.size()) {
      if (in_tinyusb_task) {
        logger_.warn_rate_limited("Vendor TX FIFO cannot hold the whole {}-byte frame in "
                                  "TinyUSB-callback context (cannot wait for a drain here), "
                                  "dropping it - send from a separate task instead",
                                  data.size());
        ec = std::make_error_code(std::errc::no_buffer_space);
        return false;
      }
      // Unplug/disconnect is a different condition from backpressure: report
      // not_connected so callers do not treat an unmount like a full FIFO.
      if (!tud_vendor_mounted()) {
        logger_.warn_rate_limited("Vendor device unmounted before a {}-byte frame could be sent",
                                  data.size());
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      // Unsigned tick subtraction stays correct across tick-count wraparound.
      if ((xTaskGetTickCount() - start_tick) >= kVendorWriteTimeoutTicks) {
        logger_.warn_rate_limited("Vendor TX FIFO full, dropping a {}-byte frame", data.size());
        ec = std::make_error_code(std::errc::no_buffer_space);
        return false;
      }
      vTaskDelay(kVendorDrainPollTicks);
    }
    // Room for the whole frame is guaranteed, so this single write takes all of
    // it - no prefix/truncation is possible.
    tud_vendor_write(data.data(), data.size());
    tud_vendor_write_flush();
    return true;
  }

  // Streaming path: frame larger than the FIFO (NOT atomic - see note above).
  size_t offset = 0;
  while (offset < data.size()) {
    uint32_t queued = tud_vendor_write(data.data() + offset, data.size() - offset);
    tud_vendor_write_flush();
    offset += queued;
    if (queued == 0) {
      if (in_tinyusb_task) {
        logger_.warn_rate_limited("Vendor TX buffer full in TinyUSB-callback context (cannot wait "
                                  "for a drain here), dropping {} bytes",
                                  data.size() - offset);
        ec = std::make_error_code(std::errc::no_buffer_space);
        break;
      }
      if (!tud_vendor_mounted()) {
        logger_.warn_rate_limited("Vendor device unmounted mid-write, dropping {} bytes",
                                  data.size() - offset);
        ec = std::make_error_code(std::errc::not_connected);
        break;
      }
      if ((xTaskGetTickCount() - start_tick) >= kVendorWriteTimeoutTicks) {
        logger_.warn_rate_limited("Vendor TX buffer full, dropping {} bytes", data.size() - offset);
        ec = std::make_error_code(std::errc::no_buffer_space);
        break;
      }
      vTaskDelay(kVendorDrainPollTicks);
    }
  }
  return offset == data.size();
#else
  (void)data;
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

bool UsbDevice::write_vendor(std::span<const uint8_t> data) {
  std::error_code ec;
  return write_vendor(data, ec);
}

bool UsbDevice::write_hid_report(uint8_t report_id, std::span<const uint8_t> report,
                                 std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_HID > 0)
  if (!initialized_ || !config_.hid) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (!tud_mounted()) {
    // Device not mounted (host not connected / not configured).
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (!tud_hid_ready()) {
    // Mounted but a previous report is still in flight -- transient
    // backpressure, distinct from a disconnect so callers can retry.
    ec = std::make_error_code(std::errc::resource_unavailable_try_again);
    return false;
  }
  if (!tud_hid_report(report_id, report.data(), static_cast<uint16_t>(report.size()))) {
    logger_.warn_rate_limited("HID report send failed (report_id={})", report_id);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  return true;
#else
  (void)report_id;
  (void)report;
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

bool UsbDevice::write_hid_report(uint8_t report_id, std::span<const uint8_t> report) {
  std::error_code ec;
  return write_hid_report(report_id, report, ec);
}

void UsbDevice::set_cdc_receive_callback(const receive_callback_fn &cb) {
  std::scoped_lock lk(cb_mutex_);
  on_cdc_receive_ = cb;
}

void UsbDevice::set_vendor_receive_callback(const receive_callback_fn &cb) {
  std::scoped_lock lk(cb_mutex_);
  on_vendor_receive_ = cb;
}

void UsbDevice::set_mount_callback(const event_callback_fn &cb) {
  std::scoped_lock lk(cb_mutex_);
  on_mount_ = cb;
}

void UsbDevice::set_unmount_callback(const event_callback_fn &cb) {
  std::scoped_lock lk(cb_mutex_);
  on_unmount_ = cb;
}

void UsbDevice::handle_usb_mount() {
  event_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_mount_;
  }
  if (cb)
    cb(); // runs in the TinyUSB task context
}

void UsbDevice::handle_usb_unmount() {
  // Drop any bytes still queued in the TX FIFOs so the next host to mount starts
  // from an empty pipe (a departed host's unread backlog otherwise lingers in
  // the software FIFO and can be mis-parsed as a reply to the next host's first
  // command).
#if (CFG_TUD_VENDOR > 0)
  if (config_.vendor)
    tud_vendor_write_clear();
#endif
#if (CFG_TUD_CDC > 0)
  if (config_.cdc)
    tud_cdc_n_write_clear(kCdcPort);
#endif
  event_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_unmount_;
  }
  if (cb)
    cb(); // runs in the TinyUSB task context
}

bool UsbDevice::is_initialized() const { return initialized_; }

bool UsbDevice::is_cdc_connected() const {
#if (CFG_TUD_CDC > 0)
  if (!initialized_ || !config_.cdc)
    return false;
  return tud_cdc_n_connected(kCdcPort);
#else
  return false;
#endif
}

bool UsbDevice::is_vendor_connected() const {
#if (CFG_TUD_VENDOR > 0)
  if (!initialized_ || !config_.vendor)
    return false;
  return tud_mounted();
#else
  return false;
#endif
}

size_t UsbDevice::vendor_write_available() const {
#if (CFG_TUD_VENDOR > 0)
  if (!initialized_ || !config_.vendor || !tud_mounted())
    return 0;
  return tud_vendor_write_available();
#else
  return 0;
#endif
}

size_t UsbDevice::cdc_write_available() const {
#if (CFG_TUD_CDC > 0)
  if (!initialized_ || !config_.cdc || !tud_mounted())
    return 0;
  return tud_cdc_n_write_available(kCdcPort);
#else
  return 0;
#endif
}

void UsbDevice::vendor_write_clear() {
#if (CFG_TUD_VENDOR > 0)
  if (initialized_ && config_.vendor)
    tud_vendor_write_clear();
#endif
}

void UsbDevice::cdc_write_clear() {
#if (CFG_TUD_CDC > 0)
  if (initialized_ && config_.cdc)
    tud_cdc_n_write_clear(kCdcPort);
#endif
}

bool UsbDevice::is_hid_ready() const {
#if (CFG_TUD_HID > 0)
  if (!initialized_ || !config_.hid)
    return false;
  return tud_hid_ready();
#else
  return false;
#endif
}

} // namespace espp
