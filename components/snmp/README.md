# SNMP Component

[![Badge](https://components.espressif.com/components/espp/snmp/badge.svg)](https://components.espressif.com/components/espp/snmp)

A synchronous SNMP manager component for ESP-IDF with a generic client API and
helpers for common network-management MIBs.

## Features

- SNMP manager support for SNMPv2c and SNMPv3 authPriv
- BER/ASN.1 encode/decode for common SNMP data types
- Confirmed operations: GET, GETNEXT, GETBULK, SET, RESPONSE, and REPORT
- WALK helper with subtree stopping rules and `endOfMibView` handling
- Built-in helpers for the SNMPv2-MIB system group and common IF-MIB columns
- Practical SNMPv3 USM support for SHA-1 authentication and AES-128 privacy

## Usage

```cpp
#include "snmp.hpp"

espp::Snmp snmp({
    .endpoint = {.host = "192.168.1.1", .port = 161},
    .version = espp::Snmp::Version::V2C,
    .v2c = {.community = "public"},
    .timeout = std::chrono::milliseconds(1500),
    .retries = 2,
    .log_level = espp::Logger::Verbosity::INFO,
});

espp::Snmp::SystemGroup system;
std::error_code ec;
if (snmp.get_system_group(system, ec)) {
  printf("sysName: %s\n", system.sys_name.c_str());
}
```

## SNMPv3 support

The first release focuses on the practical SNMPv3 profile that is most commonly
enabled on embedded switches, routers, and access points:

- USM security model
- `authPriv` security level for normal manager traffic
- SHA-1 authentication
- AES-128 privacy

The component performs engine-ID discovery, caches authoritative engine
parameters, handles key localization, and retries when the remote agent reports
timeliness issues.

## Example

See [./example](./example) for a WiFi STA example that queries an SNMP agent
without requiring one at build time.
