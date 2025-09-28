# Ping Example

This example demonstrates using the `Ping` component to run `esp_ping` with a simple CLI menu.

Run the app, then use serial CLI:
- `sta ...` to bring up WiFi (see WiFi STA menu), e.g. `sta connect <ssid> <password>`
- `ping` to enter the `ping` submenu to start/stop pinging a target IP address
  - `ping google.com`, or `ping 127.0.0.1` for example

## How to use example

### Hardware Required

This example should run on any espressif development board as it requires no
peripheral connections.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

<img width="708" height="1603" alt="CleanShot 2025-09-26 at 15 18 36" src="https://github.com/user-attachments/assets/47f6701b-c231-40b2-ae64-33601a7eaa5c" />
<img width="594" height="872" alt="CleanShot 2025-09-26 at 15 23 24" src="https://github.com/user-attachments/assets/9c45e505-6b06-4f14-a64e-3c6f2c8fe5af" />
<img width="639" height="275" alt="CleanShot 2025-09-26 at 15 24 20" src="https://github.com/user-attachments/assets/ea30e6ab-dc68-4031-a282-d6d8fb601fb7" />
