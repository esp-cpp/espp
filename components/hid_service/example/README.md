# BLE HID Service Example

This example shows how to use the `espp::HidService` class together with the
`espp::BleGattServer` class to create and manage a BLE GATT server that provides
an HID service. It uses the `hid-rp` component to emulate various game controllers
over BLE.

## Supported Controllers

The example can emulate the following game controllers (selectable via Kconfig):

- **Xbox One Controller** (default) - Full Xbox controller emulation with rumble support
- **PS4 DualShock 4** - Sony PlayStation 4 controller emulation over BLE
- **PS5 DualSense** - Sony PlayStation 5 controller emulation over BLE
- **Nintendo Switch Pro** - Nintendo Switch Pro controller emulation

Each controller configuration includes proper device identification (VID/PID), 
device name, manufacturer info, and controller-specific HID report descriptors.

https://github.com/esp-cpp/espp/assets/213467/36d3d04d-1d8e-4b1d-9661-4ce115c7e9cc

https://github.com/esp-cpp/espp/assets/213467/fd64e526-8c63-4456-8235-056edb418135


## How to use example

### Hardware Required

This example should run on any ESP32s3 development board as it requires no
peripheral connections.

### Configuration

Use `idf.py menuconfig` to select which controller to emulate:

1. Navigate to "HID Service Example Configuration"
2. Select "Example Gamepad Emulation"
3. Choose from: Xbox, PS4 DualShock 4, PS5 DualSense, or Switch Pro

Or set directly in sdkconfig:
- `CONFIG_EXAMPLE_AS_XBOX=y` (default)
- `CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSHOCK4_BLE=y`
- `CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE=y`
- `CONFIG_EXAMPLE_AS_SWITCH_PRO=y`

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2025-02-10 at 10 45 24](https://github.com/user-attachments/assets/036ebd74-2ef0-4cfa-9683-6fe4dc383fa7)
