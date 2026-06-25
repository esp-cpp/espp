# ESP32-P4 Function EV Board Example

This example demonstrates the `espp::Esp32P4FunctionEvBoard` BSP for the
Espressif ESP32-P4 Function EV Board + ESP32-P4-HMI-Subboard.

<img width="1524" height="2024" alt="image" src="https://github.com/user-attachments/assets/9b9987a2-4181-4462-af12-fdd7c7d8434a" />

It:

- initializes the MIPI-DSI display (EK79007 or ILI9881C, per Kconfig) and the
  GT911 touch controller,
- draws an LVGL UI that lets you **draw circles wherever you touch** and plays a
  **click sound** on each touch (press the BOOT button to clear the drawing),
- shows a live on-screen status read-out: display panel, touch coordinates, SD
  card size, Ethernet IP, **RTPS publisher** status, and **system** info (free
  internal/PSRAM heap and uptime),
- mounts the microSD card (if inserted),
- initializes the ES8311 audio codec (and loads an embedded `click.wav`),
- brings up Ethernet (IP101) with DHCP and, once it has an IP, starts an
  **RTPS participant** that publishes a counter on `espp/test/counter`, and
- wires the BOOT button.

> [!IMPORTANT]
> The LCD adapter board's `RST_LCD` and `PWM` signals must be jumpered to the
> ESP32-P4 main board (`RST_LCD` → J1 GPIO27, `PWM` → J1 GPIO26) or the screen
> stays black. See the component README.

## Configuration

Use `idf.py menuconfig` → *ESP32-P4 Function EV Board Configuration* to select
the display panel (EK79007 1024x600 by default, or ILI9881C 800x1280), adjust
task stack sizes, and enable Ethernet.

By default touch is **polled**. If you wire the GT911 touch INT pin (from the LCD
expansion header) to a free GPIO, enable **`Use interrupt-driven touch instead of
polling`** and set **`Touch interrupt GPIO`** to read touch from an interrupt
instead. The example needs no code change — `initialize_touch()` follows the
Kconfig setting.

## Build and Flash

```sh
idf.py set-target esp32p4
idf.py build flash monitor
```

## Notes

- PSRAM is required (enabled in `sdkconfig.defaults`) for the MIPI-DSI frame
  buffers.
- Plug an Ethernet cable to see the DHCP-assigned IP appear on screen and in the
  log.

## Example Output

  ```
I (1352) main_task: Calling app_main()
[ESP32-P4 Function EV Board Example/I][1.355]: Starting example!
[ESP32-P4 Function EV Board Example/I][1.363]: Display panel: EK79007
[ESP32-P4 Function EV Board Example/I][1.388]: Found 2 I2C device(s)
[Esp32P4FunctionEvBoard/I][1.388]: Initializing LCD (MIPI-DSI)
[Esp32P4FunctionEvBoard/I][1.390]: Performing LCD hardware reset on GPIO27
[Esp32P4FunctionEvBoard/I][1.529]: Creating MIPI DSI bus (2 lanes, 900 Mbps/lane)
[Esp32P4FunctionEvBoard/I][1.530]: Installing MIPI DSI DBI panel IO
[Esp32P4FunctionEvBoard/I][1.532]: Using display panel: EK79007 (1024x600)
[Esp32P4FunctionEvBoard/I][1.540]: Creating DPI panel (1024x600 @ 52 MHz)
[Esp32P4FunctionEvBoard/I][1.675]: LCD initialization completed (EK79007)
[Esp32P4FunctionEvBoard/I][1.675]: Initializing LVGL display with pixel buffer size: 51200 pixels
[Display/W][1.680]: No rotation callback provided, resolution changed event will not automatically update the display hardware rotation.
[Esp32P4FunctionEvBoard/I][1.695]: LVGL display initialized
[Esp32P4FunctionEvBoard/I][1.703]: Initializing GT911 multi-touch controller
[Esp32P4FunctionEvBoard/I][1.707]: Using GT911 at address 0x5D
[Esp32P4FunctionEvBoard/I][1.713]: Touch in interrupt mode (GT911 INT on GPIO33)
[Esp32P4FunctionEvBoard/I][1.721]: Touch controller initialized
[Esp32P4FunctionEvBoard/I][1.762]: Initializing SD card (4-bit SDMMC)
E (1790) sdmmc_common: sdmmc_init_ocr: send_op_cond (1) returned 0x107
HINT: Please reboot the board and then try again
E (1790) vfs_fat_sdmmc: sdmmc_card_init failed (0x107).
HINT: Please verify if there is an SD card inserted into the SD slot. Then, try rebooting the board.
E (1791) vfs_fat_sdmmc: esp_vfs_fat_sdmmc_sdcard_init failed (0x107).
[Esp32P4FunctionEvBoard/W][1.797]: Failed to initialize the card (ESP_ERR_TIMEOUT). Make sure an SD card is inserted.
[ESP32-P4 Function EV Board Example/W][1.808]: No SD card mounted
[Esp32P4FunctionEvBoard/I][1.819]: Initializing audio (ES8311) at 44100 Hz
W (1822) i2s_common: dma frame num is adjusted to 256 to align the dma buffer with 64, bufsize = 512
I (1833) DRV8311: ES8311 in Slave mode
[ESP32-P4 Function EV Board Example/I][1.843]: Loaded 35875 bytes of click audio @ 44100 Hz
[Esp32P4FunctionEvBoard/I][1.843]: Initializing Ethernet (EMAC + IP101 EMAC)
[Esp32P4FunctionEvBoard/I][1.850]: Creating ESP32 EMAC
[Esp32P4FunctionEvBoard/I][1.857]: Creating generic PHY (IP101)
[Esp32P4FunctionEvBoard/I][1.862]: Installing Ethernet driver
I (2410) esp_eth.netif.netif_glue: 60:55:f9:f9:49:a1
I (2410) esp_eth.netif.netif_glue: ethernet attached to netif
[Esp32P4FunctionEvBoard/I][4.610]: Ethernet started
[Esp32P4FunctionEvBoard/I][4.610]: Ethernet initialized; waiting for link/DHCP
[Esp32P4FunctionEvBoard/I][4.612]: Ethernet link up: 100 Mbps, full duplex
[Esp32P4FunctionEvBoard/E][4.619]: BOOT button shares GPIO35 with Ethernet RMII TXD1; refusing to initialize it while Ethernet is up (it would kill Ethernet TX).
[ESP32-P4 Function EV Board Example/I][4.637]: BOOT button not initialized (shared with Ethernet RMII TXD1 pin)
I (6758) esp_netif_handlers: eth ip: 192.168.1.31, mask: 255.255.255.0, gw: 192.168.1.1
[Esp32P4FunctionEvBoard/I][6.758]: Ethernet got IP: 192.168.1.31
[ESP32-P4 Function EV Board Example/I][6.761]: Ethernet IP: 192.168.1.31
[ESP32-P4 Function EV Board Example/I][6.770]: Got IP 192.168.1.31, starting RTPS participant
[ESP32-P4 Function EV Board Example/I][10.905]: === Connectivity self-test (ping) ===
[ESP32-P4 Function EV Board Example/I][10.905]: Pinging gateway (192.168.1.1)...
[ESP32-P4 Function EV Board Example/I][10.912]:   gateway: seq=1207959553 ttl=1341597504 time=0ms (64 bytes)
[ESP32-P4 Function EV Board Example/I][11.411]:   gateway: seq=1207959554 ttl=1341597504 time=0ms (64 bytes)
[ESP32-P4 Function EV Board Example/I][11.910]:   gateway: seq=1207959555 ttl=1341597504 time=0ms (64 bytes)
[ESP32-P4 Function EV Board Example/I][12.410]:   gateway: seq=1207959556 ttl=1341597504 time=0ms (64 bytes)
[ESP32-P4 Function EV Board Example/I][12.910]: Ping gateway (192.168.1.1): 4/4 received, 0% loss, avg 0 ms
[ESP32-P4 Function EV Board Example/W][12.910]: Ping self-test: no RTPS peer discovered yet to ping
[ESP32-P4 Function EV Board Example/I][12.918]: === Connectivity self-test done ===
  ```

<img width="1524" height="2024" alt="image" src="https://github.com/user-attachments/assets/9b9987a2-4181-4462-af12-fdd7c7d8434a" />


https://github.com/user-attachments/assets/b6fe4516-af14-455f-9c21-8f5d4323dd56


