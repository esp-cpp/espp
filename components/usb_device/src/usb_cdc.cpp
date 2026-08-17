#include "usb_cdc.hpp"

namespace espp {

// Translate the CDC-only Config into a UsbDevice::Config with a single CDC function.
static UsbDevice::Config make_device_config(const UsbCdc::Config &c) {
  UsbDevice::Config dc;
  dc.vid = c.vid;
  dc.pid = c.pid;
  dc.manufacturer = c.manufacturer;
  dc.product = c.product;
  dc.serial_number = c.serial_number;
  dc.log_level = c.log_level;
  UsbDevice::CdcFunction cdc;
  cdc.interface_name = c.interface_name;
  cdc.on_receive = c.on_receive;
  cdc.rx_chunk_size = c.rx_chunk_size;
  dc.cdc = cdc;
  return dc;
}

UsbCdc::UsbCdc(const Config &config)
    : BaseComponent("UsbCdc", config.log_level)
    , config_(config)
    , device_(std::make_unique<UsbDevice>(make_device_config(config))) {}

UsbCdc::~UsbCdc() = default;

bool UsbCdc::initialize(std::error_code &ec) { return device_->initialize(ec); }

bool UsbCdc::write(std::span<const uint8_t> data, std::error_code &ec) {
  return device_->write_cdc(data, ec);
}

bool UsbCdc::write(std::span<const uint8_t> data) { return device_->write_cdc(data); }

void UsbCdc::set_receive_callback(const receive_callback_fn &cb) {
  device_->set_cdc_receive_callback(cb);
}

bool UsbCdc::is_initialized() const { return device_->is_initialized(); }

bool UsbCdc::is_connected() const { return device_->is_cdc_connected(); }

} // namespace espp
