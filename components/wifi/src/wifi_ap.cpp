#include "wifi_ap.hpp"
#include "wifi.hpp"

namespace espp {

WifiAp::WifiAp(const WifiAp::Config &config)
    : WifiAp(config, Wifi::get().get_ap_netif()) {}

} // namespace espp
