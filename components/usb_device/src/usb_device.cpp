#include "usb_device.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>

#include "sdkconfig.h" // CONFIG_ESP_CONSOLE_UART_NUM for the console-tee path

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"
// Only pull in the CDC-ACM helper when the CDC class is actually compiled in
// (CONFIG_TINYUSB_CDC_COUNT > 0 -> CFG_TUD_CDC). This keeps XInput-only / vendor-
// only builds from forcing CDC support. tusb.h above defines CFG_TUD_CDC.
#if (CFG_TUD_CDC > 0)
#include "tinyusb_cdc_acm.h"
// Headers for route_console_to_cdc(): a write-only VFS device that forwards
// stdout to the CDC interface (and optionally tees to the primary UART console).
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_vfs.h"
#endif
// TinyUSB private class-driver API (usbd_class_driver_t, usbd_edpt_*,
// usbd_app_driver_get_cb). `src/device` is a private include of the tinyusb
// component, but `src/` is public, so reach it via the `device/` prefix.
#include "device/usbd_pvt.h"
// MSC: esp_tinyusb's storage backend (SCSI callbacks, SD card / wear-levelled
// flash media, VFS hand-over). Compiled in only with CONFIG_TINYUSB_MSC_ENABLED.
#if (CFG_TUD_MSC > 0)
#include "diskio_impl.h" // ff_diskio_get_drive: tell "no free FatFs drive" apart when formatting
#include "esp_partition.h"
#include "soc/soc_caps.h"
#include "tinyusb_msc.h"
#include "wear_levelling.h"
#endif

#include "xinput.hpp"

namespace espp {
// Bridges the global TinyUSB C callback trampolines to UsbDevice's device-task-
// only methods, which are non-public (protected). A nested type has access to the
// enclosing class's non-public members, so these thin static forwarders keep those
// methods off the public API without a raft of friend declarations for the
// (variously file-static / extern "C" / version-conditional) callbacks.
struct UsbDevice::Callbacks {
  static void cdc_rx(UsbDevice *d) { d->handle_cdc_rx(); }
  static void vendor_rx(UsbDevice *d, const uint8_t *buf, size_t n) { d->handle_vendor_rx(buf, n); }
  static void xinput_out(UsbDevice *d, const uint8_t *buf, size_t n) {
    d->handle_xinput_out(buf, n);
  }
  static const uint8_t *bos(UsbDevice *d) { return d->bos_descriptor(); }
  static const uint8_t *ms_os_20(UsbDevice *d, uint16_t &len) {
    return d->ms_os_20_descriptor(len);
  }
  static const uint8_t *webusb_url(UsbDevice *d, uint8_t &len) {
    return d->webusb_url_descriptor(len);
  }
  static const uint8_t *hid_report(UsbDevice *d) { return d->hid_report_descriptor(); }
  static void hid_rx(UsbDevice *d, uint8_t report_id, const uint8_t *b, size_t n) {
    d->handle_hid_rx(report_id, b, n);
  }
  static const std::optional<UsbDevice::VendorFunction> &vendor_config(UsbDevice *d) {
    return d->vendor_config();
  }
  static void msc_event(UsbDevice *d, const void *storage, UsbDevice::MscEvent e,
                        UsbDevice::MscOwner o) {
    d->handle_msc_event(storage, e, o);
  }
};
} // namespace espp

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

#if (CFG_TUD_CDC > 0)
// The CDC port this component uses. A single dedicated CDC-ACM interface.
constexpr tinyusb_cdcacm_itf_t kCdcPort = TINYUSB_CDC_ACM_0;

// --- Console -> CDC routing (UsbDevice::route_console_to_cdc) ----------------
// A tiny write-only VFS device: stdout is freopen'ed onto it, and its write()
// forwards each chunk to the CDC interface (and optionally tees to the original
// UART console). There is only one USB device (s_device), so this state is
// file-scope rather than per-instance.
// Must be <= ESP_VFS_PATH_MAX (15) or esp_vfs_register() rejects it.
constexpr char kConsoleVfsPath[] = "/dev/usbcons";
// fd of the original (UART) console kept as a tee, or -1. Atomic because a
// re-route can reconcile it (open/close) while a console writer loads it.
std::atomic<int> s_console_tee_fd{-1};
bool s_console_routed = false; // whether stdout has been redirected
// The UsbDevice that owns the routed console. Loaded (not s_device) by the VFS
// write, which runs on ARBITRARY tasks doing stdout writes -- so it is cleared in
// ~UsbDevice() to stop console writes from reaching a destroyed device; after
// that the VFS degrades to the UART tee only. (Redirecting stdout to an object
// couples their lifetimes: a console-routed UsbDevice must outlive concurrent
// logging -- normally trivially true, as it is a program-lifetime singleton.)
std::atomic<espp::UsbDevice *> s_console_usb{nullptr};
// One CDC TX lock shared by BOTH the console VFS write and write_cdc(), so a
// "does the whole chunk fit?" check and the write are atomic against every CDC
// writer (an app write_cdc() cannot consume the FIFO between the console's check
// and its raw write, nor vice versa). write_cdc() takes it blocking on app tasks
// (try-lock in TinyUSB-task context, to never stall tud_task); the console takes
// it try-lock and drops the chunk if held (its non-blocking contract).
std::mutex s_cdc_tx_mutex;

// Open the primary console (a UART) so route_console_to_cdc() can tee to it, or
// return -1 when there is nothing independent to tee to. Only a UART console has a
// separate physical port; a USB-Serial-JTAG console shares the native USB PHY with
// USB-OTG (teeing to it while TinyUSB owns that port is pointless) and CONSOLE_NONE
// has none -- in those builds CONFIG_ESP_CONSOLE_UART_NUM is undefined, so the tee
// is simply compiled out.
int open_primary_console_for_tee() {
#if defined(CONFIG_ESP_CONSOLE_UART_NUM)
  char path[16];
  std::snprintf(path, sizeof(path), "/dev/uart/%d", CONFIG_ESP_CONSOLE_UART_NUM);
  return open(path, O_WRONLY);
#else
  return -1;
#endif
}

// Open or close the UART tee fd to match `want`, so a re-route reconciles the tee
// with the (possibly different) current config. Idempotent. s_console_tee_fd is
// atomic; on close we clear it BEFORE closing so a console writer that just loaded
// it at worst writes to an already-closed fd (harmless EBADF on a dropped chunk).
void reconcile_console_tee(bool want) {
  const int cur = s_console_tee_fd.load();
  if (want && cur < 0) {
    s_console_tee_fd.store(open_primary_console_for_tee());
  } else if (!want && cur >= 0) {
    s_console_tee_fd.store(-1);
    close(cur);
  }
}

int cdc_console_open(const char *, int, int) { return 0; }
int cdc_console_close(int) { return 0; }
int cdc_console_fstat(int, struct stat *st) {
  *st = {};
  st->st_mode = S_IFCHR; // a character device (console): stdio uses no/line buffering
  return 0;
}
ssize_t cdc_console_write(int, const void *data, size_t size) {
  const int tee = s_console_tee_fd.load();
  if (tee >= 0)
    ::write(tee, data, size); // keep the original console as a tee
  // Mirror to CDC, NON-BLOCKING. cdc_write_available() returns 0 unless the
  // interface is mounted, so we only emit when mounted; we do NOT gate on DTR (a
  // plain serial monitor often does not assert it -- a console should still emit,
  // and the host's CDC driver buffers until a reader attaches). Serialize console
  // writers with a try-lock so the space check and the write are atomic (no other
  // writer can consume the FIFO in between and force write_cdc()'s blocking
  // drain); if the lock is held or the whole chunk doesn't fit right now, drop it
  // -- logs are best-effort and must never block the writing task.
  auto *dev = s_console_usb.load();
  if (dev) {
    std::unique_lock<std::mutex> lk(s_cdc_tx_mutex, std::try_to_lock);
    if (lk.owns_lock() && dev->cdc_write_available() >= size) {
      // We hold the shared CDC TX lock and just checked space, so no other writer
      // can interleave: this single write takes the whole chunk (no drain, no
      // torn prefix) -- tud_cdc_n_write returns `size`.
      tud_cdc_n_write(kCdcPort, static_cast<const uint8_t *>(data), size);
      tud_cdc_n_write_flush(kCdcPort);
    }
  }
  return static_cast<ssize_t>(size);
}
#endif

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
// esp_tinyusb's MSC storage backend supports two LUNs, and at most one medium of
// each type (its SD card and wear-levelling media are singletons).
constexpr size_t kMaxMscLuns = 2;

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

// [[maybe_unused]]: only the CDC/vendor write-drain paths call this, so it is
// unused in an X-Input-only build (CFG_TUD_CDC == CFG_TUD_VENDOR == 0).
[[maybe_unused]] bool on_tinyusb_task() {
  return xTaskGetCurrentTaskHandle() == s_tinyusb_task.load(std::memory_order_relaxed);
}

// --- X-Input (Xbox 360) custom TinyUSB application class driver ---------------
// TinyUSB's built-in vendor driver only handles BULK 0xFF interfaces; X-Input
// needs INTERRUPT IN+OUT on a 0xFF/0x5D/0x01 interface, so we register this
// application class driver via the weak usbd_app_driver_get_cb() override below.
// Only one USB device exists, so the driver's endpoint state is file-scope. The
// driver is always registered but open() only claims an X-Input interface, so it
// is inert when no XInput function is enabled.
struct XInputDriver {
  // All fields are touched only on the TinyUSB task (open/reset/xfer_cb/log). The
  // app-facing update_xinput_state()/is_xinput_ready() use UsbDevice's own
  // impl_->xinput_ep_in (fixed at initialize(), immutable afterwards) instead of
  // reading these, so there is no cross-task access here to synchronize.
  uint8_t itf_num{0xFF};
  uint8_t ep_in{0};
  uint8_t ep_out{0};
  // 4-byte aligned: the DWC2 also reads/writes endpoint buffers by DMA (see the
  // note on Impl::xinput_report), so keep this on a word boundary too.
  alignas(4) std::array<uint8_t, 64> out_buf{}; // interrupt-OUT receive buffer (>= kEpSize)
};
XInputDriver s_xinput_drv;

void xinput_drv_init() {}
bool xinput_drv_deinit() { return true; }
void xinput_drv_reset(uint8_t rhport) {
  (void)rhport;
  s_xinput_drv.itf_num = 0xFF;
  s_xinput_drv.ep_in = 0;
  s_xinput_drv.ep_out = 0;
}

uint16_t xinput_drv_open(uint8_t rhport, tusb_desc_interface_t const *desc_itf, uint16_t max_len) {
  // Only claim the X-Input interface (0xFF / 0x5D / 0x01); return 0 for anything
  // else so the built-in CDC/HID/vendor drivers still handle their interfaces.
  // NOTE: application class drivers are tried BEFORE the built-in ones
  // (usbd.c get_driver / process_set_config iterate app drivers first, "to allow
  // overwriting built-in ones"), so even when CFG_TUD_VENDOR>0 this driver claims
  // the X-Input 0xFF interface before the built-in vendor (bulk) driver can.
  if (desc_itf->bInterfaceClass != espp::xinput::kInterfaceClass ||
      desc_itf->bInterfaceSubClass != espp::xinput::kInterfaceSubClass ||
      desc_itf->bInterfaceProtocol != espp::xinput::kInterfaceProtocol)
    return 0;

  note_tinyusb_task();
  const uint8_t *desc_end = reinterpret_cast<const uint8_t *>(desc_itf) + max_len;
  const uint8_t *p = tu_desc_next(desc_itf); // skip the interface descriptor
  s_xinput_drv.itf_num = desc_itf->bInterfaceNumber;
  s_xinput_drv.ep_in = 0;
  s_xinput_drv.ep_out = 0;

  // Walk to the endpoints (the XID vendor descriptor between them is skipped).
  while (tu_desc_in_bounds(p, desc_end)) {
    const uint8_t type = tu_desc_type(p);
    if (type == TUSB_DESC_INTERFACE || type == TUSB_DESC_INTERFACE_ASSOCIATION)
      break;
    if (type == TUSB_DESC_ENDPOINT) {
      const tusb_desc_endpoint_t *ep = reinterpret_cast<const tusb_desc_endpoint_t *>(p);
      if (!usbd_edpt_open(rhport, ep)) {
        // Close any endpoint already opened so we don't leave partial state.
        if (s_xinput_drv.ep_in)
          usbd_edpt_close(rhport, s_xinput_drv.ep_in);
        if (s_xinput_drv.ep_out)
          usbd_edpt_close(rhport, s_xinput_drv.ep_out);
        s_xinput_drv.ep_in = 0;
        s_xinput_drv.ep_out = 0;
        return 0;
      }
      if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN)
        s_xinput_drv.ep_in = ep->bEndpointAddress;
      else
        s_xinput_drv.ep_out = ep->bEndpointAddress;
    }
    p = tu_desc_next(p);
  }

  // Prime the interrupt-OUT endpoint to receive the first rumble / LED report.
  if (s_xinput_drv.ep_out)
    usbd_edpt_xfer(rhport, s_xinput_drv.ep_out, s_xinput_drv.out_buf.data(), espp::xinput::kEpSize,
                   false);

  ESP_LOGD("espp_xinput", "class driver open: itf=%u ep_in=0x%02x ep_out=0x%02x",
           s_xinput_drv.itf_num, s_xinput_drv.ep_in, s_xinput_drv.ep_out);
  if (s_xinput_drv.ep_in == 0)
    ESP_LOGW("espp_xinput", "no interrupt IN endpoint opened -- host will get no input reports");

  return static_cast<uint16_t>(reinterpret_cast<uintptr_t>(p) -
                               reinterpret_cast<uintptr_t>(desc_itf));
}

bool xinput_drv_control_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  if (stage != CONTROL_STAGE_SETUP)
    return true; // DATA / ACK stages: nothing to do

  ESP_LOGD("espp_xinput", "control SETUP bmReq=0x%02x bReq=0x%02x wVal=0x%04x wIdx=0x%04x wLen=%u",
           request->bmRequestType, request->bRequest, request->wValue, request->wIndex,
           request->wLength);

  // Stall XUSB's vendor control requests (return false -> TinyUSB STALLs the
  // request). In particular GET_CAPABILITIES (bmReq 0xC1, bReq 0x01, wValue
  // 0x0100) expects a real 20-byte capabilities report; answering it with zeros
  // tells XUSB the controller has no controls (so it ignores all input), and
  // returning true without completing the control transfer leaves it pending.
  // Stalling is unambiguous "not supported": XUSB falls back to full default
  // capabilities, which is what a wired 360 controller's driver does and what the
  // input path (interrupt IN reports) needs. If a specific request must be
  // answered later, handle it explicitly with tud_control_xfer/tud_control_status.
  return false;
}

bool xinput_drv_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                        uint32_t xferred_bytes) {
  note_tinyusb_task();
  if (ep_addr == s_xinput_drv.ep_out) {
    if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0) {
      auto *dev = s_device.load();
      if (dev)
        espp::UsbDevice::Callbacks::xinput_out(dev, s_xinput_drv.out_buf.data(),
                                               static_cast<size_t>(xferred_bytes));
    }
    // Re-prime the OUT endpoint for the next report.
    usbd_edpt_xfer(rhport, s_xinput_drv.ep_out, s_xinput_drv.out_buf.data(), espp::xinput::kEpSize,
                   false);
  }
  // IN completion needs no action; usbd_edpt_busy() reflects readiness.
  return true;
}

const usbd_class_driver_t s_xinput_class_driver = {
    .name = "xinput",
    .init = xinput_drv_init,
    .deinit = xinput_drv_deinit,
    .reset = xinput_drv_reset,
    .open = xinput_drv_open,
    .control_xfer_cb = xinput_drv_control_xfer,
    .xfer_cb = xinput_drv_xfer_cb,
    .xfer_isr = nullptr,
    .sof = nullptr,
};

} // namespace

// Override TinyUSB's weak app-driver hook to register the X-Input class driver.
// NOTE: usbd.c both defines this as weak AND calls it in the same translation
// unit, so this strong override only wins if the linker keeps it — the
// usb_device component CMakeLists forces it with `-u usbd_app_driver_get_cb`.
extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  ESP_LOGD("espp_xinput", "registering X-Input application class driver");
  *driver_count = 1;
  return &s_xinput_class_driver;
}

#if (CFG_TUD_MSC > 0)
namespace {
// esp_tinyusb MSC storage event -> UsbDevice. Fires in the TinyUSB task for
// host-driven hand-overs and in the calling task for set_msc_owner() / storage
// creation. Loads the teardown-guarded singleton, like the other trampolines.
// cppcheck-suppress constParameterCallback // signature must match tusb_msc_callback_t
void msc_event_trampoline(tinyusb_msc_storage_handle_t handle, tinyusb_msc_event_t *event, void *) {
  auto *dev = s_device.load();
  if (!dev || !event)
    return;
  using Event = espp::UsbDevice::MscEvent;
  Event e = Event::OwnerChangeFailed;
  switch (event->id) {
  case TINYUSB_MSC_EVENT_MOUNT_START:
    e = Event::OwnerChangeStarted;
    break;
  case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
    e = Event::OwnerChanged;
    break;
  case TINYUSB_MSC_EVENT_MOUNT_FAILED:
    e = Event::OwnerChangeFailed;
    break;
  case TINYUSB_MSC_EVENT_FORMAT_REQUIRED:
    e = Event::FormatRequired;
    break;
  case TINYUSB_MSC_EVENT_FORMAT_FAILED:
    e = Event::FormatFailed;
    break;
  default:
    return;
  }
  // event->mount_point is the medium's owner at the moment the event is emitted:
  // esp_tinyusb emits MOUNT_START before it updates the owner (the previous
  // owner) and MOUNT_COMPLETE after (the new owner) -- tinyusb_msc.c
  // msc_storage_mount() / msc_storage_unmount().
  const auto owner = event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP
                         ? espp::UsbDevice::MscOwner::App
                         : espp::UsbDevice::MscOwner::Host;
  espp::UsbDevice::Callbacks::msc_event(dev, handle, e, owner);
}
} // namespace
#endif // CFG_TUD_MSC > 0

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
  uint8_t xinput_itf{0xFF};
  uint8_t xinput_ep_in{0};  // 0x80|n, or 0 if the XInput function is disabled
  uint8_t xinput_ep_out{0}; // n, or 0 if disabled
  // Input-report TX buffer; held for the duration of the async interrupt-IN
  // transfer submitted by update_xinput_state(). MUST be 4-byte aligned: the ESP32-S3
  // DWC2 reads it by DMA and a misaligned buffer makes the controller read from
  // the aligned-down address, prepending the preceding byte to every report
  // (which shifted our "00 14 .." report by one and made XUSB reject all input).
  alignas(4) std::array<uint8_t, espp::xinput::kReportInSize> xinput_report{};

#if (CFG_TUD_MSC > 0)
  // MSC media. A fixed array (never a growing vector): esp_tinyusb keeps a raw
  // pointer to each base_path string for the storage object's lifetime.
  struct MscLun {
    tinyusb_msc_storage_handle_t storage{nullptr};
    wl_handle_t wl{WL_INVALID_HANDLE}; // flash media: our wear-levelling mount
    std::string base_path;
    // Result of the last hand-over, recorded by the event bridge so
    // set_msc_owner() can report it (esp_tinyusb's setter ignores the outcome).
    std::atomic<uint8_t> last_result{0}; // 0 ok, 1 failed, 2 no filesystem
    // Set while hand_over_msc() quietly resets the owner after a failed mount, so
    // that internal step is not reported as a user-visible hand-over.
    std::atomic<bool> reverting{false};
    // The medium has no FAT filesystem (FormatRequired). Unlike last_result it
    // survives across calls: esp_tinyusb leaves such a medium application-owned
    // with nothing mounted and emits no event when asked for that owner again.
    std::atomic<bool> no_filesystem{false};
  };
  std::array<MscLun, kMaxMscLuns> msc_luns{};
  size_t msc_lun_count{0};
  bool msc_driver_installed{false};
#endif
};

UsbDevice *UsbDevice::instance() { return s_device; }

UsbDevice::UsbDevice(const Config &config)
    : BaseComponent("UsbDevice", config.log_level)
    , impl_(std::make_unique<Impl>())
    , config_(config)
    , on_cdc_receive_(config.cdc ? config.cdc->on_receive : nullptr)
    , on_vendor_receive_(config.vendor ? config.vendor->on_receive : nullptr)
    , on_xinput_rumble_(config.xinput ? config.xinput->on_rumble : nullptr)
    , on_hid_receive_(config.hid ? config.hid->on_receive : nullptr)
    , on_msc_event_(config.msc ? config.msc->on_event : nullptr) {}

UsbDevice::~UsbDevice() {
#if (CFG_TUD_CDC > 0)
  // If this instance owns the routed console, detach it FIRST so stdout writes
  // from other tasks stop reaching this destructing instance (they degrade to the
  // UART tee). stdout stays pointed at the VFS device (its functions are
  // file-scope, not tied to this instance), so logging keeps working. NOTE: a
  // write already past this load when we clear it can still race destruction --
  // a console-routed UsbDevice must outlive concurrent logging (see
  // route_console_to_cdc): normally trivial, as it is a program-lifetime object.
  {
    UsbDevice *expected_console = this;
    s_console_usb.compare_exchange_strong(expected_console, nullptr);
  }
#endif
  if (initialized_) {
    // Detach the global callback routing BEFORE tearing down the driver so a
    // TinyUSB callback that fires during deinit cannot dereference this
    // destructing instance (use-after-free). Atomic compare_exchange so the
    // clear only happens if we still own the slot (mirrors the claim in
    // initialize()).
    UsbDevice *expected = this;
    s_device.compare_exchange_strong(expected, nullptr);
#if (CFG_TUD_CDC > 0)
    if (config_.cdc)
      tinyusb_cdcacm_deinit(kCdcPort);
#endif
#if (CFG_TUD_MSC > 0)
    if (!release_msc_before_uninstall()) {
      // A storage object is still mapped (its queued writes never completed).
      // Keep the TinyUSB driver running and deliberately leak impl_: esp_tinyusb
      // still points at its base_path strings and backing media, and uninstalling
      // would strand the queued writes. A later UsbDevice cannot initialize.
      (void)impl_.release();
      initialized_ = false;
      return;
    }
#endif
    tinyusb_driver_uninstall();
    initialized_ = false;
  }
}

// ---------------------------------------------------------------------------
// TinyUSB C callbacks (global; routed to the active instance).
// ---------------------------------------------------------------------------

#if (CFG_TUD_CDC > 0)
// CDC RX trampoline registered with esp_tinyusb; runs in the TinyUSB task.
static void cdc_rx_trampoline(int itf, cdcacm_event_t *event) {
  (void)event;
  note_tinyusb_task();
  if (itf != (int)kCdcPort)
    return;
  // load once: the pointer must not be re-read between check and use
  auto *dev = s_device.load();
  if (dev)
    UsbDevice::Callbacks::cdc_rx(dev);
}
#endif

extern "C" {

// BOS descriptor (weak in TinyUSB core). Returns our WebUSB/MS-OS BOS when the
// vendor+WebUSB function is enabled, otherwise NULL (no BOS).
uint8_t const *tud_descriptor_bos_cb(void) {
  note_tinyusb_task();
  auto *dev = s_device.load();
  return dev ? UsbDevice::Callbacks::bos(dev) : nullptr;
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
    UsbDevice::Callbacks::vendor_rx(dev, buffer, static_cast<size_t>(bufsize));
}

// Vendor control-transfer callback: answer the WebUSB URL and MS OS 2.0
// descriptor requests, and the WebUSB "connect" class request (0x22).
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  note_tinyusb_task();
  if (stage != CONTROL_STAGE_SETUP)
    return true; // nothing to do on DATA / ACK stages
  auto *dev = s_device.load();
  if (!dev || !UsbDevice::Callbacks::vendor_config(dev).has_value())
    return false;
  const auto &vendor = *UsbDevice::Callbacks::vendor_config(dev);

  switch (request->bmRequestType_bit.type) {
  case TUSB_REQ_TYPE_VENDOR:
    // wIndex 2 == WEBUSB_REQUEST_GET_URL; qualifying on it keeps this branch
    // from shadowing the MS-OS request if the two vendor codes are configured
    // to the same value.
    if (request->bRequest == vendor.webusb_vendor_code && request->wIndex == 2) {
      // Return the WebUSB landing-page URL descriptor.
      uint8_t len = 0;
      const uint8_t *url = UsbDevice::Callbacks::webusb_url(dev, len);
      if (!url)
        return false;
      return tud_control_xfer(rhport, request, (void *)(uintptr_t)url, len);
    }
    if (request->bRequest == vendor.ms_os_vendor_code && request->wIndex == 7) {
      // Return the MS OS 2.0 descriptor set.
      uint16_t total_len = 0;
      const uint8_t *ms = UsbDevice::Callbacks::ms_os_20(dev, total_len);
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
  // Runs in the TinyUSB device-task context: record it so a mount/unmount
  // callback that calls write_cdc()/write_vendor() takes the non-blocking
  // fail-fast TX path instead of vTaskDelay()-ing inside the TinyUSB task
  // (which would deadlock USB servicing).
  note_tinyusb_task();
  // Load the teardown-guarded singleton (not event_arg): a destructor that has
  // atomically detached the instance during teardown then yields nullptr here,
  // matching the other tud_*_cb trampolines.
  (void)arg;
  auto *dev = s_device.load();
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
  return dev ? UsbDevice::Callbacks::hid_report(dev) : nullptr;
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

// HID SET_REPORT control request AND interrupt-OUT endpoint data: dispatch the
// received bytes to the application's HID receive callback (host -> device),
// enabling request/response HID protocols (e.g. the Switch Pro handshake).
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
  (void)instance;
  (void)report_type;
  note_tinyusb_task();
  auto *dev = s_device.load();
  if (dev)
    UsbDevice::Callbacks::hid_rx(dev, report_id, buffer, static_cast<size_t>(bufsize));
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
  if (!config_.cdc && !config_.vendor && !config_.hid && !config_.xinput && !config_.msc) {
    logger_.error("No USB function enabled (enable cdc, vendor, hid, xinput and/or msc)");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (config_.msc) {
#if (CFG_TUD_MSC == 0)
    logger_.error("MSC function requested but CFG_TUD_MSC==0. Set "
                  "CONFIG_TINYUSB_MSC_ENABLED=y in sdkconfig.");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#else
    const auto &media = config_.msc->media;
    if (media.empty() || media.size() > kMaxMscLuns) {
      logger_.error("MSC function needs 1..{} media, got {}", kMaxMscLuns, media.size());
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    size_t sd_cards = 0, partitions = 0;
    for (size_t i = 0; i < media.size(); ++i) {
      const auto &m = media[i];
      if (m.base_path.size() < 2 || m.base_path.front() != '/') {
        logger_.error("MSC medium {}: base_path '{}' must be an absolute VFS path like '/msc'", i,
                      m.base_path);
        ec = std::make_error_code(std::errc::invalid_argument);
        return false;
      }
      if (i > 0 && media[0].base_path == m.base_path) {
        logger_.error("MSC media 0 and 1 share base_path '{}'; each needs its own", m.base_path);
        ec = std::make_error_code(std::errc::invalid_argument);
        return false;
      }
      if (m.type == MscMedium::Type::SdCard) {
        if (!m.sd_card) {
          logger_.error("MSC medium {}: type SdCard but sd_card is null", i);
          ec = std::make_error_code(std::errc::invalid_argument);
          return false;
        }
#if SOC_SDMMC_HOST_SUPPORTED
        ++sd_cards;
#else
        logger_.error("MSC medium {}: SD card media need a target with an SDMMC host "
                      "(esp_tinyusb's SD backend is not built for this target)",
                      i);
        ec = std::make_error_code(std::errc::function_not_supported);
        return false;
#endif
      } else {
        if (m.partition_label.empty()) {
          logger_.error("MSC medium {}: type FlashPartition but partition_label is empty", i);
          ec = std::make_error_code(std::errc::invalid_argument);
          return false;
        }
        ++partitions;
      }
    }
    if (sd_cards > 1 || partitions > 1) {
      logger_.error("MSC supports at most one SD card and one flash partition (esp_tinyusb's "
                    "media backends are singletons)");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
#endif
  }
  if (config_.cdc) {
#if (CFG_TUD_CDC == 0)
    logger_.error("CDC function requested but CFG_TUD_CDC==0. Set "
                  "CONFIG_TINYUSB_CDC_COUNT>0 in sdkconfig.");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
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

  // [[maybe_unused]]: these feed TUD_CDC_DESCRIPTOR, which is compiled only when
  // CFG_TUD_CDC>0; without CDC the block below never runs (config_.cdc is
  // rejected earlier) and the values are unused.
  [[maybe_unused]] uint8_t cdc_itf = 0, cdc_str = 0, cdc_notif = 0, cdc_out = 0, cdc_in = 0;
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

  // hid_str/hid_in/hid_out are consumed only in the CFG_TUD_HID-guarded
  // descriptor branch below, so they are unused when HID is not compiled in.
  [[maybe_unused]] uint8_t hid_itf = 0, hid_str = 0, hid_in = 0, hid_out = 0;
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

  uint8_t xinput_itf = 0, xinput_str = 0;
  if (config_.xinput) {
    xinput_itf = next_itf++;
    xinput_str = next_str++;
    impl_->owned_strings.push_back(config_.xinput->interface_name);
    // Use SEPARATE endpoint numbers for IN and OUT. The retail controller shares
    // number 1, but the ESP32-S3 DWC2 corrupts the interrupt-IN stream (a leading
    // 0x01 byte) when the same number is used for both directions.
    const uint8_t in_ep = next_ep++;
    const uint8_t out_ep = next_ep++;
    impl_->xinput_ep_in = static_cast<uint8_t>(0x80 | in_ep); // interrupt IN
    impl_->xinput_ep_out = out_ep;                            // interrupt OUT
    in_used++;
    out_used++;
    impl_->xinput_itf = xinput_itf;
  }

  // MSC: one interface, bulk OUT + bulk IN on one endpoint number. msc_* are only
  // consumed by the CFG_TUD_MSC-guarded descriptor branch below.
  [[maybe_unused]] uint8_t msc_itf = 0, msc_str = 0, msc_out = 0, msc_in = 0;
  if (config_.msc) {
    msc_itf = next_itf++;
    msc_str = next_str++;
    impl_->owned_strings.push_back(config_.msc->interface_name);
    const uint8_t m_ep = next_ep++;
    msc_out = m_ep;                             // bulk OUT
    msc_in = static_cast<uint8_t>(0x80 | m_ep); // bulk IN
    in_used++;
    out_used++;
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
  // When the X-Input function is the ONLY function, the device must present the
  // Xbox 360 controller's identity (VID/PID/bcdDevice) and a 0xFF/0xFF/0xFF
  // device class so a PC's XUSB driver binds it. Combining XInput with other
  // functions keeps the normal composite identity (and XUSB will not bind).
  const bool xinput_only =
      config_.xinput && !config_.cdc && !config_.vendor && !config_.hid && !config_.msc;
  impl_->device_desc = tusb_desc_device_t{};
  impl_->device_desc.bLength = sizeof(tusb_desc_device_t);
  impl_->device_desc.bDescriptorType = TUSB_DESC_DEVICE;
  // BOS/WebUSB requires bcdUSB >= 2.1.
  impl_->device_desc.bcdUSB = webusb ? 0x0210 : 0x0200;
  // Advertise the IAD-based composite class (0xEF/0x02/0x01) only when CDC is
  // enabled, since CDC is the function that emits an Interface Association
  // Descriptor. For a vendor-only and/or HID-only device there is no IAD, so use
  // 0x00/0x00/0x00 and let the interface descriptors declare the class(es). An
  // X-Input-only device declares the Xbox controller's 0xFF/0xFF/0xFF class.
  if (xinput_only) {
    impl_->device_desc.bDeviceClass = 0xFF;
    impl_->device_desc.bDeviceSubClass = 0xFF;
    impl_->device_desc.bDeviceProtocol = 0xFF;
  } else if (config_.cdc) {
    impl_->device_desc.bDeviceClass = TUSB_CLASS_MISC;
    impl_->device_desc.bDeviceSubClass = MISC_SUBCLASS_COMMON;
    impl_->device_desc.bDeviceProtocol = MISC_PROTOCOL_IAD;
  } else {
    impl_->device_desc.bDeviceClass = 0x00;
    impl_->device_desc.bDeviceSubClass = 0x00;
    impl_->device_desc.bDeviceProtocol = 0x00;
  }
  impl_->device_desc.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE;
  impl_->device_desc.idVendor = xinput_only ? config_.xinput->vid : config_.vid;
  impl_->device_desc.idProduct = xinput_only ? config_.xinput->pid : config_.pid;
  impl_->device_desc.bcdDevice = xinput_only ? espp::xinput::kDefaultBcdDevice : config_.bcd_device;
  impl_->device_desc.iManufacturer = 0x01;
  impl_->device_desc.iProduct = 0x02;
  impl_->device_desc.iSerialNumber = 0x03;
  impl_->device_desc.bNumConfigurations = 0x01;

  // --- Configuration descriptor ---
  uint8_t itf_count = 0;
  uint16_t total_len = TUD_CONFIG_DESC_LEN;
#if (CFG_TUD_CDC > 0)
  if (config_.cdc) {
    itf_count = static_cast<uint8_t>(itf_count + 2);
    total_len = static_cast<uint16_t>(total_len + TUD_CDC_DESC_LEN);
  }
#endif
  if (config_.vendor) {
    itf_count = static_cast<uint8_t>(itf_count + 1);
    total_len = static_cast<uint16_t>(total_len + TUD_VENDOR_DESC_LEN);
  }
  if (config_.hid) {
    itf_count = static_cast<uint8_t>(itf_count + 1);
    total_len = static_cast<uint16_t>(
        total_len + (config_.hid->has_out_endpoint ? TUD_HID_INOUT_DESC_LEN : TUD_HID_DESC_LEN));
  }
  if (config_.xinput) {
    itf_count = static_cast<uint8_t>(itf_count + 1);
    total_len = static_cast<uint16_t>(total_len + espp::xinput::kInterfaceDescriptorLen);
  }
#if (CFG_TUD_MSC > 0)
  if (config_.msc) {
    itf_count = static_cast<uint8_t>(itf_count + 1);
    total_len = static_cast<uint16_t>(total_len + TUD_MSC_DESC_LEN);
  }
#endif

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
      // The 9-byte configuration descriptor header, written out explicitly
      // (TUD_CONFIG_DESCRIPTOR's arithmetic on runtime values is a narrowing
      // conversion inside a braced initializer). bMaxPower is in 2 mA units:
      // clamp to the USB 2.0 maximum (500 mA) and round UP so an odd request is
      // never under-reported (1 mA -> 2 mA).
      const uint16_t power_ma = std::min<uint16_t>(config_.max_power_ma, 500);
      const uint8_t attributes = static_cast<uint8_t>(
          TU_BIT(7) | (config_.remote_wakeup ? TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP : 0));
      const uint8_t hdr[9] = {
          9,                                             // bLength
          static_cast<uint8_t>(TUSB_DESC_CONFIGURATION), // bDescriptorType
          static_cast<uint8_t>(total_len & 0xFF),        // wTotalLength (LE)
          static_cast<uint8_t>(total_len >> 8),
          static_cast<uint8_t>(itf_count),          // bNumInterfaces
          1,                                        // bConfigurationValue
          0,                                        // iConfiguration
          attributes,                               // bmAttributes
          static_cast<uint8_t>((power_ma + 1) / 2), // bMaxPower (2 mA units)
      };
      append(hdr, sizeof(hdr));
    }
#if (CFG_TUD_CDC > 0)
    if (config_.cdc) {
      const uint8_t d[] = {
          TUD_CDC_DESCRIPTOR(cdc_itf, cdc_str, cdc_notif, 8, cdc_out, cdc_in, bulk_ep_size),
      };
      append(d, sizeof(d));
    }
#endif
    if (config_.vendor) {
      const uint8_t d[] = {
          TUD_VENDOR_DESCRIPTOR(vendor_itf, vendor_str, vendor_out, vendor_in, bulk_ep_size),
      };
      append(d, sizeof(d));
    }
#if (CFG_TUD_HID > 0)
    // Guarded because the TUD_HID_* macros reference HID class constants only
    // declared when the HID class driver is compiled in. config_.hid can never be
    // set here when CFG_TUD_HID==0 (initialize() rejects it earlier), so this
    // branch is dead in that case and safe to compile out.
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
#endif
    if (config_.xinput) {
      // Hand-built interface + XID + two interrupt endpoints (the built-in TinyUSB
      // descriptor macros can't express X-Input's class triple / XID blob). The
      // bIntervals are the full-speed values; on an HS-capable part they are
      // interpreted as exponents, but X-Input is a full-speed protocol (and the
      // ESP32-S3 USB-OTG is full speed).
      const auto d = espp::xinput::interface_descriptor(xinput_itf, xinput_str, impl_->xinput_ep_in,
                                                        impl_->xinput_ep_out);
      append(d.data(), d.size());
    }
#if (CFG_TUD_MSC > 0)
    if (config_.msc) {
      const uint8_t d[] = {
          TUD_MSC_DESCRIPTOR(msc_itf, msc_str, msc_out, msc_in, bulk_ep_size),
      };
      append(d, sizeof(d));
    }
#endif
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
  // The callback loads the teardown-guarded s_device singleton itself, so no
  // event_arg is needed.
  tusb_cfg.event_cb = espp_usb_device_event_cb;

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

  // MSC media come up BEFORE the driver: a host that is already connected
  // mounts the device as soon as the driver starts, and the hand-over on that
  // first mount must find the storage objects.
#if (CFG_TUD_MSC > 0)
  if (config_.msc && !init_msc(ec)) {
    s_device = nullptr;
    return false;
  }
#endif

  esp_err_t err = tinyusb_driver_install(&tusb_cfg);
  if (err != ESP_OK) {
    logger_.error("tinyusb_driver_install failed: {}", esp_err_to_name(err));
    s_device = nullptr;
    deinit_msc();
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  // --- Initialize the CDC-ACM function (vendor needs no explicit init) ---
#if (CFG_TUD_CDC > 0)
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
      // A host may already have enumerated and queued MSC writes: release the
      // media while the TinyUSB task can still run them, like the destructor.
#if (CFG_TUD_MSC > 0)
      if (!release_msc_before_uninstall()) {
        (void)impl_.release(); // storage still mapped: keep its strings alive
        impl_ = std::make_unique<Impl>();
        ec = std::make_error_code(std::errc::io_error);
        return false;
      }
#endif
      tinyusb_driver_uninstall();
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
  }
#endif

  initialized_ = true;
  // Copy the packed descriptor fields into locals: they cannot bind to the
  // logger's const-reference parameters directly.
  const uint16_t enum_vid = impl_->device_desc.idVendor;
  const uint16_t enum_pid = impl_->device_desc.idProduct;
  logger_.info("Initialized native USB device (VID=0x{:04x} PID=0x{:04x}) cdc={} vendor={} hid={} "
               "xinput={} msc={}{}",
               enum_vid, enum_pid, config_.cdc.has_value(), config_.vendor.has_value(),
               config_.hid.has_value(), config_.xinput.has_value(),
               config_.msc ? config_.msc->media.size() : 0, webusb ? " webusb" : "");
#if (CFG_TUD_CDC > 0)
  // Opt-in: route the console to the CDC interface now that TinyUSB owns the USB
  // port. Best-effort -- a routing failure must not fail initialization (the
  // device is up; the console simply stays where it was), so log and carry on.
  if (config_.cdc && config_.cdc->route_console) {
    std::error_code route_ec;
    if (!route_console_to_cdc(route_ec))
      logger_.warn("could not route console to CDC: {}", route_ec.message());
  }
#endif
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

  // Serialize with every other CDC writer (other write_cdc() callers + the console
  // VFS sink) so the available-space check and the write below are atomic. In
  // TinyUSB-task context we must NOT block (tud_task() is below us on the stack and
  // drains the FIFO), so try-lock and fail fast if another writer holds it.
  std::unique_lock<std::mutex> tx_lock(s_cdc_tx_mutex, std::defer_lock);
  if (in_tinyusb_task) {
    if (!tx_lock.try_lock()) {
      ec = std::make_error_code(std::errc::no_buffer_space);
      return false;
    }
  } else {
    tx_lock.lock();
  }

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

void UsbDevice::set_hid_receive_callback(const receive_callback_fn &cb) {
  std::scoped_lock lk(cb_mutex_);
  on_hid_receive_ = cb;
}

void UsbDevice::handle_hid_rx(uint8_t report_id, const uint8_t *buffer, size_t bufsize) {
#if (CFG_TUD_HID > 0)
  receive_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_hid_receive_;
  }
  if (!cb || !config_.hid)
    return;
  if (report_id == 0) {
    // Interrupt-OUT (or report-id-less SET_REPORT): TinyUSB passes the report as
    // received, so byte 0 is already the report id when the descriptor uses them.
    cb(std::span<const uint8_t>(buffer, bufsize));
  } else {
    // Control SET_REPORT with a report id: TinyUSB parses the id out of wValue, so
    // prepend it to keep the callback contract "byte 0 is the report id".
    std::vector<uint8_t> framed;
    framed.reserve(bufsize + 1);
    framed.push_back(report_id);
    framed.insert(framed.end(), buffer, buffer + bufsize);
    cb(std::span<const uint8_t>(framed.data(), framed.size()));
  }
#else
  (void)report_id;
  (void)buffer;
  (void)bufsize;
#endif
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

bool UsbDevice::route_console_to_cdc(std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_CDC > 0)
  if (!initialized_ || !config_.cdc) {
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
  }
  if (s_console_routed) {
    // The VFS is already installed + stdout redirected (by this device on a repeat
    // call, or by a previous device that has since been destroyed). Re-attach this
    // instance as the console owner so CDC logging resumes on it -- otherwise a
    // freshly created device would return success without owning the sink and its
    // CDC logs would be silently dropped. Reconcile the tee with THIS device's
    // config (a prior owner may have had a different tee_console setting).
    reconcile_console_tee(config_.cdc->tee_console);
    s_console_usb.store(this);
    return true;
  }
  fflush(stdout);
  // Optionally keep the original console as a tee (best-effort). ESP-IDF's libc
  // has no dup(), so we re-open the primary console device by path rather than
  // duplicating stdout's fd. Only a UART console has an independent port to tee
  // to; a JTAG / no console has none (open_primary_console_for_tee() returns -1).
  reconcile_console_tee(config_.cdc->tee_console);
  esp_vfs_t vfs = {};
  vfs.flags = ESP_VFS_FLAG_DEFAULT;
  // The classic (context-pointer-less) esp_vfs_t members are deprecated in IDF v6
  // but are exactly right for this tiny write-only console sink; use them
  // deliberately and suppress the notice.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  vfs.open = &cdc_console_open;
  vfs.write = &cdc_console_write;
  vfs.close = &cdc_console_close;
  vfs.fstat = &cdc_console_fstat;
#pragma GCC diagnostic pop
  if (esp_vfs_register(kConsoleVfsPath, &vfs, nullptr) != ESP_OK) {
    reconcile_console_tee(false); // close the tee so a retry does not leak the fd
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  // Publish the console owner BEFORE freopen makes stdout point at the VFS, so the
  // write callback never loads a null owner while stdout already targets it.
  s_console_usb.store(this);
  if (freopen(kConsoleVfsPath, "w", stdout) == nullptr) {
    // freopen closes stdout's previous target even when opening the new one fails,
    // so stdout is now broken. Routing is best-effort (initialize() promises the
    // console stays put on failure), so restore a usable console: undo the VFS +
    // tee, then re-point stdout at the primary UART console if there is one.
    s_console_usb.store(nullptr);
    esp_vfs_unregister(kConsoleVfsPath);
    reconcile_console_tee(false);
#if defined(CONFIG_ESP_CONSOLE_UART_NUM)
    char restore[16];
    std::snprintf(restore, sizeof(restore), "/dev/uart/%d", CONFIG_ESP_CONSOLE_UART_NUM);
    // best-effort restore; freopen returns stdout (not a new resource to close),
    // and there is nothing more to do if even this fails.
    // cppcheck-suppress ignoredReturnValue
    freopen(restore, "w", stdout);
#endif
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  setvbuf(stdout, nullptr, _IONBF, 0); // push each log line to CDC promptly
  s_console_routed = true;
  logger_.info("console routed to USB-CDC{}",
               s_console_tee_fd.load() >= 0 ? " (teed to the UART console)" : "");
  return true;
#else
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

bool UsbDevice::route_console_to_cdc() {
  std::error_code ec;
  return route_console_to_cdc(ec);
}

bool UsbDevice::is_console_routed_to_cdc() const {
#if (CFG_TUD_CDC > 0)
  return s_console_routed;
#else
  return false;
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

// ---------------------------------------------------------------------------
// X-Input (Xbox 360) function.
// ---------------------------------------------------------------------------

uint8_t UsbDevice::xinput_in_endpoint() const { return impl_->xinput_ep_in; }

void UsbDevice::handle_xinput_out(const uint8_t *buffer, size_t bufsize) {
  receive_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_xinput_rumble_;
  }
  if (cb && buffer && bufsize > 0)
    cb(std::span<const uint8_t>(buffer, bufsize)); // TinyUSB task context
}

bool UsbDevice::update_xinput_state(const espp::xinput::GamepadState &state, std::error_code &ec) {
  ec.clear();
  if (!initialized_ || !config_.xinput) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  // The interrupt-IN endpoint address, fixed at initialize() and immutable after
  // (so no cross-task synchronization is needed). tud_mounted() gates on the host
  // having SET_CONFIGURATION, which is exactly when the class driver's open() runs
  // for this (only) interface — so a mounted device has its endpoint open.
  const uint8_t ep_in = impl_->xinput_ep_in;
  if (!tud_mounted() || ep_in == 0) {
    // Normal before the host mounts the device (the app may poll update_* in a
    // loop): report it via ec and let the caller decide -- don't log.
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  // update_xinput_state() runs on the caller's task, not the TinyUSB task. Follow the
  // TinyUSB endpoint contract exactly (busy-check, then claim/xfer/release):
  //  - usbd_edpt_busy() rejects submitting while a previous report is still in
  //    flight (transient backpressure) — and is required because usbd_edpt_xfer()
  //    asserts the endpoint is not busy.
  //  - usbd_edpt_claim() arbitrates against the USB task; it is released after the
  //    transfer is QUEUED (on both success and failure) so the endpoint is never
  //    left permanently claimed if a completion is missed.
  if (usbd_edpt_busy(0, ep_in)) {
    ec = std::make_error_code(std::errc::resource_unavailable_try_again);
    return false;
  }
  if (!usbd_edpt_claim(0, ep_in)) {
    ec = std::make_error_code(std::errc::resource_unavailable_try_again);
    return false;
  }
  // The buffer must outlive the (asynchronous) transfer, so it lives in Impl.
  impl_->xinput_report = state.report();
  const bool queued = usbd_edpt_xfer(0, ep_in, impl_->xinput_report.data(),
                                     static_cast<uint16_t>(impl_->xinput_report.size()), false);
  usbd_edpt_release(0, ep_in); // pair with claim(), regardless of queue result
  if (!queued) {
    logger_.warn_rate_limited("XInput report send (usbd_edpt_xfer) failed on ep 0x{:02x}", ep_in);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  return true;
}

bool UsbDevice::update_xinput_state(const espp::xinput::GamepadState &state) {
  std::error_code ec;
  return update_xinput_state(state, ec);
}

bool UsbDevice::is_xinput_ready() const {
  if (!initialized_ || !config_.xinput)
    return false;
  // Fixed at initialize(), immutable after; tud_mounted() implies the class
  // driver has opened this interface's endpoints (it is the only function).
  const uint8_t ep_in = impl_->xinput_ep_in;
  return tud_mounted() && ep_in != 0 && !usbd_edpt_busy(0, ep_in);
}

// ---------------------------------------------------------------------------
// MSC (mass storage).
// ---------------------------------------------------------------------------

bool UsbDevice::init_msc(std::error_code &ec) {
#if (CFG_TUD_MSC > 0)
  tinyusb_msc_driver_config_t driver_cfg{};
  driver_cfg.user_flags.auto_mount_off = config_.msc->auto_handover ? 0 : 1;
  driver_cfg.callback = &msc_event_trampoline;
  driver_cfg.callback_arg = nullptr;
  esp_err_t err = tinyusb_msc_install_driver(&driver_cfg);
  if (err != ESP_OK) {
    logger_.error("tinyusb_msc_install_driver failed: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  impl_->msc_driver_installed = true;

  const auto &media = config_.msc->media;
  for (size_t i = 0; i < media.size(); ++i) {
    const auto &m = media[i];
    auto &lun = impl_->msc_luns[i];
    lun.base_path = m.base_path; // esp_tinyusb keeps a pointer to this string

    tinyusb_msc_storage_config_t storage_cfg{};
    // data(), not c_str(): the esp_tinyusb field is a non-const `char *`
    storage_cfg.fat_fs.base_path = lun.base_path.data();
    storage_cfg.fat_fs.config.max_files = m.max_files;
    storage_cfg.fat_fs.do_not_format = !m.format_if_unformatted;
    storage_cfg.fat_fs.format_flags = 0; // FM_ANY
    // Always create the medium host-owned and hand it to the application below.
    // If esp_tinyusb mounts it during creation and that mount fails, it frees the
    // storage object while it is still mapped as a LUN and returns no handle, so
    // the stale LUN could never be removed (and SCSI requests would reach freed
    // memory). Created host-owned, the handle is ours before anything can fail.
    storage_cfg.mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;

    if (m.type == MscMedium::Type::SdCard) {
#if SOC_SDMMC_HOST_SUPPORTED
      storage_cfg.medium.card = m.sd_card;
      err = tinyusb_msc_new_storage_sdmmc(&storage_cfg, &lun.storage);
#endif
    } else {
      const esp_partition_t *partition = esp_partition_find_first(
          ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, m.partition_label.c_str());
      if (!partition) {
        logger_.error(
            "MSC medium {}: no 'data, fat' partition labelled '{}' in the partition table", i,
            m.partition_label);
        deinit_msc();
        ec = std::make_error_code(std::errc::no_such_device);
        return false;
      }
      err = wl_mount(partition, &lun.wl);
      if (err == ESP_OK) {
        storage_cfg.medium.wl_handle = lun.wl;
        err = tinyusb_msc_new_storage_spiflash(&storage_cfg, &lun.storage);
      } else {
        logger_.error("MSC medium {}: wear levelling mount of '{}' failed: {}", i,
                      m.partition_label, esp_err_to_name(err));
      }
    }
    if (err != ESP_OK || !lun.storage) {
      logger_.error("MSC medium {}: creating the storage failed: {}", i, esp_err_to_name(err));
      deinit_msc();
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    impl_->msc_lun_count = i + 1;

    if (m.initial_owner == MscOwner::App) {
      std::error_code hand_over_ec;
      if (!hand_over_msc(i, MscOwner::App, hand_over_ec)) {
        if (hand_over_ec == std::errc::no_such_device) {
          // No FAT filesystem: not fatal. FormatRequired has been reported and the
          // medium stays application-owned so format_msc_medium() can run.
          logger_.warn("MSC medium {}: no FAT filesystem yet; format it to use it", i);
        } else {
          logger_.error("MSC medium {}: could not mount it for the application: {}", i,
                        hand_over_ec.message());
          deinit_msc();
          ec = hand_over_ec;
          return false;
        }
      }
    }

    uint32_t sectors = 0, sector_size = 0; // best-effort, for the log line only
    tinyusb_msc_get_storage_capacity(lun.storage, &sectors);
    tinyusb_msc_get_storage_sector_size(lun.storage, &sector_size);
    logger_.info("MSC medium {}: {} ({} KiB) at '{}', owned by the {}", i,
                 m.type == MscMedium::Type::SdCard ? "SD card" : m.partition_label,
                 static_cast<uint64_t>(sectors) * sector_size / 1024, lun.base_path,
                 m.initial_owner == MscOwner::App ? "application" : "host");
  }
  return true;
#else
  (void)ec;
  return true;
#endif
}

bool UsbDevice::deinit_msc(std::chrono::milliseconds drain_timeout) {
#if (CFG_TUD_MSC > 0)
  const auto deadline = std::chrono::steady_clock::now() + drain_timeout;
  bool all_released = true;
  for (size_t i = kMaxMscLuns; i-- > 0;) {
    auto &lun = impl_->msc_luns[i];
    if (lun.storage) {
      // A medium marked application-owned with nothing mounted (unformatted, or a
      // failed mount) must be reset to the host first: esp_tinyusb's delete does
      // ESP_ERROR_CHECK(msc_storage_unmount()), and that unmount fails with
      // ESP_ERR_INVALID_STATE when no drive is registered -- aborting the device.
      // The setter records the host owner even though its own unmount fails.
      tinyusb_msc_mount_point_t current = TINYUSB_MSC_STORAGE_MOUNT_USB;
      tinyusb_msc_get_storage_mount_point(lun.storage, &current);
      uint64_t total_bytes = 0, free_bytes = 0;
      if (current == TINYUSB_MSC_STORAGE_MOUNT_APP &&
          esp_vfs_fat_info(lun.base_path.c_str(), &total_bytes, &free_bytes) != ESP_OK) {
        lun.reverting = true;
        tinyusb_msc_set_storage_mount_point(lun.storage, TINYUSB_MSC_STORAGE_MOUNT_USB);
        lun.reverting = false;
      }
      esp_err_t err = tinyusb_msc_delete_storage(lun.storage);
      // ESP_ERR_INVALID_STATE: host writes are still queued on the TinyUSB task
      while (err == ESP_ERR_INVALID_STATE && std::chrono::steady_clock::now() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
        err = tinyusb_msc_delete_storage(lun.storage);
      }
      if (err != ESP_OK) {
        // Keep the handle and the medium behind it: the storage object is still
        // mapped as a LUN, so unmounting its wear levelling here would leave it
        // pointing at an invalid handle.
        if (drain_timeout.count() > 0 || err != ESP_ERR_INVALID_STATE)
          logger_.error("MSC medium {}: deleting the storage failed ({}); leaving it in place", i,
                        esp_err_to_name(err));
        all_released = false;
        continue;
      }
      lun.storage = nullptr;
    }
    if (lun.wl != WL_INVALID_HANDLE) {
      wl_unmount(lun.wl);
      lun.wl = WL_INVALID_HANDLE;
    }
    lun.no_filesystem = false;
  }
  if (!all_released)
    return false; // the driver cannot be uninstalled while a LUN is still mapped
  impl_->msc_lun_count = 0;
  if (impl_->msc_driver_installed) {
    const esp_err_t err = tinyusb_msc_uninstall_driver();
    if (err == ESP_OK)
      impl_->msc_driver_installed = false;
    else
      logger_.error("tinyusb_msc_uninstall_driver failed: {}", esp_err_to_name(err));
  }
  return true;
#else
  (void)drain_timeout;
  return true;
#endif
}

bool UsbDevice::release_msc_before_uninstall() {
#if (CFG_TUD_MSC > 0)
  if (!config_.msc)
    return true;
  // Host writes are queued and run later on the TinyUSB task, and a storage
  // object with writes still queued cannot be deleted. Stop the host sending
  // more (drop the pull-up) and delete the media while the task still runs;
  // stopping it first would lose the queued writes and strand the storage.
  tud_disconnect();
  if (deinit_msc(std::chrono::milliseconds(1000)))
    return true;
  logger_.error("MSC media could not be released (host writes still queued); leaving the USB "
                "driver installed -- no new UsbDevice can be initialized");
  return false;
#else
  return true;
#endif
}

void UsbDevice::handle_msc_event(const void *storage, MscEvent event, MscOwner owner) {
  size_t lun_index = 0;
#if (CFG_TUD_MSC > 0)
  bool found = false;
  for (size_t i = 0; i < kMaxMscLuns; ++i) {
    if (storage && impl_->msc_luns[i].storage == storage) {
      lun_index = i;
      found = true;
      break;
    }
  }
  if (!found)
    return; // not one of ours (or already torn down)
  auto &lun = impl_->msc_luns[lun_index];
  if (lun.reverting)
    return; // hand_over_msc() resetting the owner after a failed mount
  switch (event) {
  case MscEvent::OwnerChangeStarted:
    break;
  case MscEvent::OwnerChanged:
    if (owner == MscOwner::App)
      lun.no_filesystem = false; // it mounted, so it has one
    break;
  case MscEvent::OwnerChangeFailed:
    lun.last_result = 1;
    // `owner` is the side that still has the medium: the attempt was to the other
    logger_.warn("MSC medium {}: hand-over to the {} failed (mount / unmount error)", lun_index,
                 owner == MscOwner::App ? "host" : "application");
    break;
  case MscEvent::FormatFailed:
    // esp_tinyusb reports no error code with this event; its own log (tag
    // "tinyusb_msc_storage") has the FatFs result
    lun.last_result = 1;
    logger_.warn("MSC medium {}: formatting the FAT filesystem failed", lun_index);
    break;
  case MscEvent::FormatRequired:
    lun.last_result = 2;
    lun.no_filesystem = true;
    logger_.warn("MSC medium {}: no FAT filesystem (format it, or enable format_if_unformatted)",
                 lun_index);
    break;
  }
#else
  (void)storage;
#endif
  msc_event_callback_fn cb;
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    cb = on_msc_event_;
  }
  if (cb)
    cb(lun_index, event, owner); // outside the lock: it may take its time / log
}

bool UsbDevice::hand_over_msc(size_t index, MscOwner owner, std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_MSC > 0)
  auto &lun = impl_->msc_luns[index];
  const bool to_app = owner == MscOwner::App;
  const auto app_mounted = [&lun]() {
    uint64_t total_bytes = 0, free_bytes = 0;
    return esp_vfs_fat_info(lun.base_path.c_str(), &total_bytes, &free_bytes) == ESP_OK;
  };

  if (to_app) {
    tinyusb_msc_mount_point_t current = TINYUSB_MSC_STORAGE_MOUNT_USB;
    tinyusb_msc_get_storage_mount_point(lun.storage, &current);
    if (current == TINYUSB_MSC_STORAGE_MOUNT_APP) {
      if (app_mounted())
        return true; // already the application's
      if (lun.no_filesystem) {
        ec = std::make_error_code(std::errc::no_such_device); // still waiting for a format
        return false;
      }
      // Marked application-owned with nothing mounted (an earlier hand-over
      // failed): esp_tinyusb would treat this request as a no-op, so quietly
      // reset it to the host (nothing mounted: this only resets the owner) and
      // mount it again below.
      lun.reverting = true;
      tinyusb_msc_set_storage_mount_point(lun.storage, TINYUSB_MSC_STORAGE_MOUNT_USB);
      lun.reverting = false;
    }
  }

  lun.last_result = 0; // the hand-over's events run synchronously in this call
  if (tinyusb_msc_set_storage_mount_point(lun.storage, to_app ? TINYUSB_MSC_STORAGE_MOUNT_APP
                                                              : TINYUSB_MSC_STORAGE_MOUNT_USB) !=
      ESP_OK) {
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  // esp_tinyusb's setter records the requested owner whatever the mount / unmount
  // did, and several of its failure paths raise no event, so confirm the result
  // against the VFS: the application has the medium exactly when a mounted FAT
  // volume answers at its base_path.
  if (to_app == app_mounted())
    return true;

  if (to_app) {
    if (lun.last_result == 2) {
      // No FAT filesystem. Keep the medium application-owned (esp_tinyusb's
      // format requires it) so format_msc_medium() can create one.
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    // The mount failed and nothing is mounted, but esp_tinyusb now marks the
    // medium application-owned, so the host would be told "no medium" while the
    // application has no files either. Give it back to the host: with nothing
    // mounted the unmount only resets the owner.
    logger_.error("MSC medium {}: mounting it at '{}' failed; it stays with the host", index,
                  lun.base_path);
    lun.reverting = true;
    tinyusb_msc_set_storage_mount_point(lun.storage, TINYUSB_MSC_STORAGE_MOUNT_USB);
    lun.reverting = false;
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  // Handing it to the host, but the application's volume still answers: the host
  // must not write under a mounted FAT volume. Remove the VFS registration
  // ourselves and check again before reporting success.
  logger_.warn("MSC medium {}: '{}' still mounted after the hand-over to the host; unregistering "
               "it",
               index, lun.base_path);
  esp_vfs_fat_unregister_path(lun.base_path.c_str());
  if (!app_mounted())
    return true;
  logger_.error("MSC medium {}: could not unmount '{}' for the host", index, lun.base_path);
  ec = std::make_error_code(std::errc::io_error);
  return false;
#else
  (void)index;
  (void)owner;
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

bool UsbDevice::set_msc_owner(size_t lun, MscOwner owner, std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_MSC > 0)
  if (!initialized_ || !config_.msc) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (lun >= impl_->msc_lun_count) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (owner == MscOwner::App && tud_mounted() && msc_owner(lun) == MscOwner::Host) {
    // esp_tinyusb accepts WRITE(10) data and runs the write later on the TinyUSB
    // task, without re-checking ownership, so a write the attached host already
    // queued could land after the FAT volume is mounted for the application.
    // There is no backend drain primitive: refuse, and let the host eject first.
    logger_.warn("MSC medium {}: the attached host still has it; eject it on the host (or "
                 "detach) before handing it to the application",
                 lun);
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }
  return hand_over_msc(lun, owner, ec);
#else
  (void)lun;
  (void)owner;
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

bool UsbDevice::set_msc_owner(size_t lun, MscOwner owner) {
  std::error_code ec;
  return set_msc_owner(lun, owner, ec);
}

std::optional<UsbDevice::MscOwner> UsbDevice::msc_owner(size_t lun) const {
#if (CFG_TUD_MSC > 0)
  if (!initialized_ || !config_.msc || lun >= impl_->msc_lun_count)
    return std::nullopt;
  tinyusb_msc_mount_point_t mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
  if (tinyusb_msc_get_storage_mount_point(impl_->msc_luns[lun].storage, &mount_point) != ESP_OK)
    return std::nullopt;
  return mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP ? MscOwner::App : MscOwner::Host;
#else
  (void)lun;
  return std::nullopt;
#endif
}

std::optional<UsbDevice::MscCapacity> UsbDevice::msc_capacity(size_t lun) const {
#if (CFG_TUD_MSC > 0)
  if (!initialized_ || !config_.msc || lun >= impl_->msc_lun_count)
    return std::nullopt;
  MscCapacity capacity;
  const auto storage = impl_->msc_luns[lun].storage;
  if (tinyusb_msc_get_storage_capacity(storage, &capacity.sector_count) != ESP_OK ||
      tinyusb_msc_get_storage_sector_size(storage, &capacity.sector_size) != ESP_OK)
    return std::nullopt;
  return capacity;
#else
  (void)lun;
  return std::nullopt;
#endif
}

size_t UsbDevice::msc_lun_count() const {
#if (CFG_TUD_MSC > 0)
  return initialized_ && config_.msc ? impl_->msc_lun_count : 0;
#else
  return 0;
#endif
}

bool UsbDevice::format_msc_medium(size_t lun, std::error_code &ec) {
  ec.clear();
#if (CFG_TUD_MSC > 0)
  if (!initialized_ || !config_.msc) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (lun >= impl_->msc_lun_count) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (msc_owner(lun) != MscOwner::App) {
    logger_.error("MSC medium {}: hand it to the application before formatting", lun);
    ec = std::make_error_code(std::errc::operation_not_permitted);
    return false;
  }
  auto &l = impl_->msc_luns[lun];
  {
    uint64_t total_bytes = 0, free_bytes = 0;
    if (esp_vfs_fat_info(l.base_path.c_str(), &total_bytes, &free_bytes) == ESP_OK) {
      ec = std::make_error_code(std::errc::file_exists); // mounted: it has a filesystem
      return false;
    }
  }
  {
    // esp_tinyusb reports "every FatFs drive slot is taken" with the same
    // ESP_ERR_NOT_FOUND it uses for "a filesystem already exists"
    BYTE pdrv = 0xFF;
    if (ff_diskio_get_drive(&pdrv) != ESP_OK) {
      logger_.error("MSC medium {}: no free FatFs drive to format it on (raise "
                    "CONFIG_FATFS_VOLUME_COUNT or unmount another FAT volume)",
                    lun);
      ec = std::make_error_code(std::errc::device_or_resource_busy);
      return false;
    }
  }
  // esp_tinyusb's format does not take the storage lock, and with auto_handover a
  // host attaching (or detaching) mid-format would run its mount / unmount on the
  // TinyUSB task against the same drive. Drop the connection for the format so no
  // attach can happen, let any detach finish first, and reconnect afterwards.
  const bool pause_usb = config_.msc->auto_handover;
  if (pause_usb) {
    tud_disconnect();
    for (int i = 0; i < 50 && tud_mounted(); ++i)
      vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(20)); // let a detach callback already running complete
  }
  const esp_err_t err = tinyusb_msc_format_storage(l.storage);
  if (pause_usb)
    tud_connect();
  switch (err) {
  case ESP_OK:
    l.no_filesystem = false;
    logger_.info("MSC medium {}: formatted and mounted at '{}'", lun, l.base_path);
    return true;
  case ESP_ERR_NOT_FOUND:     // a filesystem is already on the medium
  case ESP_ERR_INVALID_STATE: // ...and its VFS path is registered
    ec = std::make_error_code(std::errc::file_exists);
    return false;
  default:
    logger_.error("MSC medium {}: format failed: {}", lun, esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
#else
  (void)lun;
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#endif
}

void UsbDevice::set_msc_event_callback(const msc_event_callback_fn &cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  on_msc_event_ = cb;
}

} // namespace espp
