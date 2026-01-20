# Provisioning Component

WiFi provisioning component with embedded web interface.

## Features

- Creates temporary WiFi AP for configuration
- Web-based UI for scanning and connecting to networks
- Tests credentials before saving
- Mobile-friendly responsive design
- Automatic AP shutdown after provisioning

## Usage

```cpp
//! [provisioning example]
#include "provisioning.hpp"
#include "nvs.hpp"

// Initialize provisioning
espp::Provisioning::Config prov_config{
    .ap_ssid = "MyDevice-Setup",
    .ap_password = "setup123",  // Leave empty for open AP
    .device_name = "My ESP32 Device",
    .auto_shutdown_ap = true,
    .on_provisioned = [](const std::string& ssid, const std::string& password) {
        fmt::print("Provisioned! SSID: {}\n", ssid);
        // Save credentials to NVS
        espp::Nvs nvs;
        nvs.set_string("wifi_ssid", ssid);
        nvs.set_string("wifi_password", password);
    }
};

espp::Provisioning prov(prov_config);
prov.start();

fmt::print("Connect to WiFi: {}\n", prov_config.ap_ssid);
fmt::print("Open browser to: http://{}\n", prov.get_ip_address());

while (prov.is_active()) {
    std::this_thread::sleep_for(1s);
}
//! [provisioning example]
```

## Web Interface

Connect to the AP and navigate to `http://192.168.4.1` to:
1. Scan for available networks
2. Select a network
3. Enter password
4. Test and save configuration

The interface is mobile-friendly and provides visual feedback during scanning and connection.
