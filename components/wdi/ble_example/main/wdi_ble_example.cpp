#include <chrono>
#include <cmath>
#include <thread>

#include "ble_gatt_server.hpp"
#include "logger.hpp"
#include "wdi_ble.hpp"

using namespace std::chrono_literals;

// WDI (Wheelchair Digital Interface) BLE peripheral example: advertise as a WDI
// device (an accessory / alternative joystick) and drive a wheelchair (the BLE
// central) over the standard WDI GATT service. The device sends Control reports +
// keepalives and receives Feedback; here we sweep a demo joystick pattern.
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "WDI BLE", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting WDI BLE peripheral example");

  // The WDI device role over BLE. Feedback / host-identity callbacks just log.
  espp::WdiBlePeripheral wdi({
      .on_feedback =
          [&](const espp::wdi::FeedbackReport &f) {
            logger.info("feedback: drive_enabled={} speed={} {:.1f} mph",
                        f.has(espp::wdi::FeedbackBit::DriveEnabled), f.speed, f.velocity_mph());
          },
      .on_keepalive_response =
          [&](const espp::wdi::HostUuid &u) {
            logger.info("host uuid: manufacturer=0x{:04x}", u.manufacturer_id());
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // Bring up the GATT server, install the WDI service, advertise it.
  espp::BleGattServer ble;
  ble.set_log_level(espp::Logger::Verbosity::WARN);
  ble.set_callbacks({
      .connect_callback = [&](NimBLEConnInfo &) { logger.info("wheelchair connected"); },
      .disconnect_callback =
          [&](NimBLEConnInfo &, espp::BleGattServer::DisconnectReason) {
            logger.info("wheelchair disconnected");
          },
  });
  const std::string device_name = "espp WDI";
  ble.init(device_name);
  wdi.make_service(ble.server());
  ble.start_services();
  wdi.start();
  ble.start();

  espp::BleGattServer::AdvertisedData adv;
  adv.setFlags(BLE_HS_ADV_F_DISC_GEN);
  adv.setName(device_name);
  adv.addServiceUUID(espp::WdiBlePeripheral::service_uuid());
  ble.set_advertisement_data(adv);
  ble.start_advertising();
  logger.info("Advertising as '{}'; connect a WDI host (wheelchair).", device_name);

  // Drive loop: sweep the joystick in a slow circle with drive enabled, poll for
  // keepalives, and ask for feedback once a second. A real accessory would map
  // physical inputs here instead.
  int step = 0;
  while (true) {
    espp::wdi::ControlReport c;
    const float angle = (step % 60) / 60.0f * 2.0f * 3.14159265f;
    c.x = static_cast<int8_t>(80.0f * std::sin(angle));  // right/left
    c.y = static_cast<int8_t>(-80.0f * std::cos(angle)); // forward/reverse
    c.set(espp::wdi::ControlBit::DriveEnable);
    wdi.send_control(c); // resets the keepalive timer
    if (step % 20 == 0)
      wdi.request_feedback();
    wdi.poll(); // send a keepalive if one is due
    ++step;
    std::this_thread::sleep_for(50ms);
  }
}
