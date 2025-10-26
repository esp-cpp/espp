#include "wifi_sta.hpp"
#include "wifi.hpp"

namespace espp {

void WifiSta::init(const Config &config) {
  if (netif_ == nullptr) {
    logger_.info("Creating network interface and ensuring WiFi stack is initialized");
    auto &wifi = Wifi::get();
    if (!wifi.init()) {
      logger_.error("Could not initialize WiFi stack");
      return;
    }
    netif_ = wifi.get_sta_netif();
  }

  if (!register_event_handlers()) {
    logger_.error("Could not register event handlers");
  }

  if (!reconfigure(config)) {
    logger_.error("Could not configure WiFi STA");
  }

  if (!start()) {
    logger_.error("Could not start WiFi STA");
  }

  logger_.info("WiFi STA started, SSID: '{}'", config.ssid);
}

} // namespace espp
