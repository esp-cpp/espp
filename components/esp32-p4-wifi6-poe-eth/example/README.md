# ESP32-P4-WIFI6-POE-ETH Example

This example shows how to use the `espp::Esp32P4Wifi6PoeEth` BSP to bring up
the wired Ethernet interface (DHCP client or DHCP server, selectable via
`menuconfig`) and print the assigned IP address.

If the board is powered through Waveshare's plug-in PoE module, nothing changes
in software — PoE only supplies the 5 V rail.

## Hardware

[Waveshare ESP32-P4-WIFI6-POE-ETH](https://www.waveshare.com/wiki/ESP32-P4-WIFI6-POE-ETH)

## How to build and flash

```bash
idf.py set-target esp32p4
idf.py -p PORT flash monitor
```

Use `idf.py menuconfig` → `ESP32-P4-WIFI6-POE-ETH Example Configuration` to
switch between DHCP client (default) and DHCP server mode.

## Wi-Fi 6 via the on-board ESP32-C6 (ESP-Hosted)

The board's Wi-Fi 6 / Bluetooth LE radio is an ESP32-C6-MINI-1 connected to the
P4 over SDIO, running Espressif's ESP-Hosted slave firmware (pre-flashed by
Waveshare). This BSP does not wrap ESP-Hosted; to add Wi-Fi to your own
application, use Espressif's managed components directly.

Note: fetching managed components requires the IDF Component Manager, which
this example — like all espp examples — explicitly disables (the
`set(ENV{IDF_COMPONENT_MANAGER} "0")` line in its `CMakeLists.txt`, which
avoids manifest scanning of the whole espp tree). The steps below therefore
apply to your own project; if you instead adapt this example in place, first
remove that line from `CMakeLists.txt` to re-enable the Component Manager.

1. Add the dependencies to your project (this pulls in `esp_hosted` and
   `esp_wifi_remote`, which transparently forwards the standard `esp_wifi` API
   to the C6) by listing them in your `main` component's `idf_component.yml`:

   ```yaml
   dependencies:
     espressif/esp_wifi_remote: "*"
     espressif/esp_hosted: "*"
   ```

   or equivalently by running:

   ```bash
   idf.py add-dependency "espressif/esp_wifi_remote"
   idf.py add-dependency "espressif/esp_hosted"
   ```

2. In `menuconfig`, under `Component config → ESP-Hosted config`, select the
   SDIO transport (slot 1) and verify the host-side pins match this board's
   wiring (these are the ESP-Hosted slot-1 defaults for the ESP32-P4, since
   the board copies the ESP32-P4-Function-EV-Board wiring):

   ```
   CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_CLK_SLOT_1=18
   CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_CMD_SLOT_1=19
   CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D0_SLOT_1=14
   CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D1_4BIT_BUS_SLOT_1=15
   CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D2_4BIT_BUS_SLOT_1=16
   CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D3_4BIT_BUS_SLOT_1=17
   CONFIG_ESP_HOSTED_SDIO_GPIO_RESET_SLAVE=54
   ```

   (The `CONFIG_ESP_HOSTED_SDIO_PIN_*` symbols are derived, prompt-less
   Kconfig values; the `..._PRIV_..._SLOT_1` symbols above are the editable
   ones exposed in `menuconfig`.)

3. Use the normal `esp_wifi` APIs (`esp_wifi_init()`, `esp_wifi_start()`, ...)
   — `esp_wifi_remote` routes them to the C6.

If the C6's slave firmware ever needs re-flashing, its UART (U0TXD / U0RXD /
IO9) is exposed on the 4-pin H7 header; see the Waveshare wiki for the
procedure.

## Example output

```
[P4Wifi6PoeEth/I][0.058]: ESP32-P4-WIFI6-POE-ETH example starting
[Esp32P4Wifi6PoeEth/I][0.064]: Initializing Ethernet (EMAC + IP101GRI RMII, DHCP client)
[P4Wifi6PoeEth/I][0.955]: Waiting for Ethernet...
[P4Wifi6PoeEth/I][2.482]: Ethernet link up
[P4Wifi6PoeEth/I][3.960]: DHCP lease acquired: 192.168.1.114
[P4Wifi6PoeEth/I][3.961]: Connected. IP: 192.168.1.114
[P4Wifi6PoeEth/I][8.962]: Ethernet up, IP: 192.168.1.114
```
