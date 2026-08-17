#include "usb_cdc.hpp"

#include <array>
#include <cstring>
#include <vector>

#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"

namespace espp {

// The CDC port this component uses. A single dedicated CDC-ACM interface.
static constexpr tinyusb_cdcacm_itf_t kCdcPort = TINYUSB_CDC_ACM_0;

// The TinyUSB CDC RX callback is a plain C function pointer with no user
// argument, so we keep a file-scope pointer to the active instance (per port)
// and marshal into the instance method. Only one UsbCdc per port is supported.
static UsbCdc *s_instances[TINYUSB_CDC_ACM_MAX] = {nullptr};

// Storage for the descriptors that TinyUSB references by pointer for the
// lifetime of the driver. These must outlive tinyusb_driver_install().
struct UsbCdc::Impl {
  tusb_desc_device_t device_desc{};
  std::vector<uint8_t> config_desc;
  // String descriptors: index 0 is the LANGID (0x0409), then manufacturer,
  // product, serial, and CDC interface name. We keep the owning strings and an
  // array of pointers TinyUSB can read.
  std::array<uint8_t, 2> langid{{0x09, 0x04}};
  std::string manufacturer;
  std::string product;
  std::string serial_number;
  std::string interface_name;
  std::array<const char *, 5> strings{};
};

UsbCdc::UsbCdc(const Config &config)
    : BaseComponent("UsbCdc", config.log_level)
    , impl_(std::make_unique<Impl>())
    , config_(config)
    , on_receive_(config.on_receive) {}

UsbCdc::~UsbCdc() {
  if (initialized_) {
    tinyusb_cdcacm_deinit(kCdcPort);
    tinyusb_driver_uninstall();
    s_instances[kCdcPort] = nullptr;
    initialized_ = false;
  }
}

// Static trampoline registered with TinyUSB; runs in the TinyUSB task context.
static void rx_trampoline(int itf, cdcacm_event_t *event) {
  (void)event;
  if (itf < 0 || itf >= TINYUSB_CDC_ACM_MAX)
    return;
  UsbCdc *self = s_instances[itf];
  if (self)
    self->handle_rx();
}

void UsbCdc::handle_rx() {
  // Copy the current callback under lock, then invoke it outside the lock.
  receive_callback_fn cb;
  {
    std::scoped_lock lk(cb_mutex_);
    cb = on_receive_;
  }
  if (!cb)
    return;
  std::vector<uint8_t> buf(config_.rx_chunk_size);
  size_t rx_size = 0;
  // Drain the RX FIFO; a single RX event may hold more than one chunk.
  do {
    rx_size = 0;
    esp_err_t err = tinyusb_cdcacm_read(kCdcPort, buf.data(), buf.size(), &rx_size);
    if (err != ESP_OK) {
      logger_.error("CDC read error: {}", esp_err_to_name(err));
      break;
    }
    if (rx_size > 0)
      cb(std::span<const uint8_t>(buf.data(), rx_size));
  } while (rx_size == buf.size());
}

bool UsbCdc::initialize(std::error_code &ec) {
  ec.clear();
  if (initialized_) {
    logger_.warn("Already initialized");
    return true;
  }
  if (s_instances[kCdcPort] != nullptr) {
    logger_.error("CDC port {} already in use by another UsbCdc instance", (int)kCdcPort);
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }

  // Build the string descriptor table (kept alive by impl_).
  impl_->manufacturer = config_.manufacturer;
  impl_->product = config_.product;
  impl_->serial_number = config_.serial_number;
  impl_->interface_name = config_.interface_name;
  impl_->strings[0] = reinterpret_cast<const char *>(impl_->langid.data());
  impl_->strings[1] = impl_->manufacturer.c_str();
  impl_->strings[2] = impl_->product.c_str();
  impl_->strings[3] = impl_->serial_number.c_str();
  impl_->strings[4] = impl_->interface_name.c_str();

  // Build the device descriptor with the configured VID/PID. TUSB_CLASS_MISC +
  // IAD is used so that the CDC interface association is exposed correctly.
  impl_->device_desc = tusb_desc_device_t{};
  impl_->device_desc.bLength = sizeof(tusb_desc_device_t);
  impl_->device_desc.bDescriptorType = TUSB_DESC_DEVICE;
  impl_->device_desc.bcdUSB = 0x0200;
  impl_->device_desc.bDeviceClass = TUSB_CLASS_MISC;
  impl_->device_desc.bDeviceSubClass = MISC_SUBCLASS_COMMON;
  impl_->device_desc.bDeviceProtocol = MISC_PROTOCOL_IAD;
  impl_->device_desc.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE;
  impl_->device_desc.idVendor = config_.vid;
  impl_->device_desc.idProduct = config_.pid;
  impl_->device_desc.bcdDevice = 0x0100;
  impl_->device_desc.iManufacturer = 0x01;
  impl_->device_desc.iProduct = 0x02;
  impl_->device_desc.iSerialNumber = 0x03;
  impl_->device_desc.bNumConfigurations = 0x01;

  // Build the configuration descriptor: one CDC-ACM interface (2 USB interfaces:
  // notification + data). Interface string index 4 matches the strings table.
  const uint16_t cdc_desc_len = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN;
  const uint16_t ep_size = (TUD_OPT_HIGH_SPEED ? 512 : 64);
  const uint8_t cfg_desc[] = {
      // config number, interface count, string index, total length, attribute, power (mA)
      TUD_CONFIG_DESCRIPTOR(1, 2, 0, cdc_desc_len, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
      // interface number, string index, EP notification, notif size, EP out, EP in, EP data size
      TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, ep_size),
  };
  impl_->config_desc.assign(cfg_desc, cfg_desc + sizeof(cfg_desc));

  // Install the TinyUSB driver with our descriptors.
  tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
  tusb_cfg.descriptor.device = &impl_->device_desc;
  tusb_cfg.descriptor.string = impl_->strings.data();
  tusb_cfg.descriptor.string_count = static_cast<int>(impl_->strings.size());
  tusb_cfg.descriptor.full_speed_config = impl_->config_desc.data();
#if (TUD_OPT_HIGH_SPEED)
  tusb_cfg.descriptor.high_speed_config = impl_->config_desc.data();
#endif

  esp_err_t err = tinyusb_driver_install(&tusb_cfg);
  if (err != ESP_OK) {
    logger_.error("tinyusb_driver_install failed: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  // Register this instance before initializing CDC so the RX callback can find us.
  s_instances[kCdcPort] = this;

  tinyusb_config_cdcacm_t acm_cfg = {};
  acm_cfg.cdc_port = kCdcPort;
  acm_cfg.callback_rx = &rx_trampoline;
  acm_cfg.callback_rx_wanted_char = nullptr;
  acm_cfg.callback_line_state_changed = nullptr;
  acm_cfg.callback_line_coding_changed = nullptr;

  err = tinyusb_cdcacm_init(&acm_cfg);
  if (err != ESP_OK) {
    logger_.error("tinyusb_cdcacm_init failed: {}", esp_err_to_name(err));
    s_instances[kCdcPort] = nullptr;
    tinyusb_driver_uninstall();
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  initialized_ = true;
  logger_.info("Initialized native USB CDC (VID=0x{:04x} PID=0x{:04x})", config_.vid, config_.pid);
  return true;
}

bool UsbCdc::write(std::span<const uint8_t> data, std::error_code &ec) {
  ec.clear();
  if (!initialized_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  size_t offset = 0;
  while (offset < data.size()) {
    size_t queued =
        tinyusb_cdcacm_write_queue(kCdcPort, data.data() + offset, data.size() - offset);
    if (queued == 0) {
      // The TX buffer is full; flush what we have and retry once.
      tinyusb_cdcacm_write_flush(kCdcPort, 0);
      queued = tinyusb_cdcacm_write_queue(kCdcPort, data.data() + offset, data.size() - offset);
      if (queued == 0) {
        logger_.warn_rate_limited("CDC TX buffer full, dropping {} bytes", data.size() - offset);
        ec = std::make_error_code(std::errc::no_buffer_space);
        break;
      }
    }
    offset += queued;
    // Non-blocking flush (timeout 0) - safe to call from within callbacks.
    tinyusb_cdcacm_write_flush(kCdcPort, 0);
  }
  return offset == data.size();
}

bool UsbCdc::write(std::span<const uint8_t> data) {
  std::error_code ec;
  return write(data, ec);
}

void UsbCdc::set_receive_callback(const receive_callback_fn &cb) {
  std::scoped_lock lk(cb_mutex_);
  on_receive_ = cb;
}

bool UsbCdc::is_initialized() const { return initialized_; }

bool UsbCdc::is_connected() const {
  if (!initialized_)
    return false;
  return tud_cdc_n_connected(kCdcPort);
}

} // namespace espp
