#include "wifi_ap.hpp"
#include "wifi.hpp"

namespace espp {

void WifiAp::init(const Config &config) {
  if (netif_ == nullptr) {
    logger_.info("Creating network interface and ensuring WiFi stack is initialized");
    auto &wifi = Wifi::get();
    if (!wifi.init()) {
      logger_.error("Could not initialize WiFi stack");
      return;
    }
    netif_ = wifi.get_ap_netif();
  }

  if (!register_event_handlers()) {
    logger_.error("Could not register event handlers");
    return;
  }

  if (!reconfigure(config)) {
    logger_.error("Could not configure WiFi AP");
    return;
  }

  if (!start()) {
    logger_.error("Could not start WiFi AP");
    return;
  }

  logger_.info("WiFi AP started, SSID: '{}'", config.ssid);
}

} // namespace espp
