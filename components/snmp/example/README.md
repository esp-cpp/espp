# SNMP Example

This example connects to WiFi in station mode and performs a few read-only SNMP
manager operations against a configurable target.

## Supported modes

- SNMPv2c using a community string
- SNMPv3 `authPriv` using SHA-1 authentication and AES-128 privacy

## How to use example

Set the WiFi and SNMP target configuration through menuconfig, then build and
flash:

```sh
idf.py flash monitor
```

The example reads the system group, prints a small interface summary, and exits.
It does not require an SNMP agent at build time.
