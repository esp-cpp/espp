#include "wifi_sta.hpp"
#include "wifi.hpp"

namespace espp {

WifiSta::WifiSta(const WifiSta::Config &config)
    : WifiSta(config, Wifi::get().get_sta_netif()) {}

} // namespace espp
