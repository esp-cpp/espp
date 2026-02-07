# Provisioning Example

This example demonstrates the `espp::Provisioning` component, which provides a
web-based WiFi configuration interface for ESP32 devices.

__Scanning for network and connecting__:
<table>
  <tr>
    <td>scan <img alt="image" src="https://github.com/user-attachments/assets/ffc31171-8575-49a9-b5bf-0b934f030497"></td>
    <td>enter password <img alt="image" src="https://github.com/user-attachments/assets/e230ce28-c1a4-4919-bac0-91757ac3f71c"></td>
  </tr>
  <tr>
    <td>connecting <img alt="image" src="https://github.com/user-attachments/assets/b8a6d2de-927e-4b12-b749-9190b9cf5200"></td>
    <td>connected <img alt="image" src="https://github.com/user-attachments/assets/19e7d9e8-1c9d-4da6-adba-591ec0c886f9"></td>
  </tr>
  <tr>
    <td>provisioning completed <img alt="image" src="https://github.com/user-attachments/assets/adcbc5db-c4c3-49fb-9a81-e78ef70303c4"></td>
  </tr>
</table>

__Managing saved networks__:
<table>
  <tr>
    <td>view <img alt="image" src="https://github.com/user-attachments/assets/ce099bfc-2fe7-4dd6-b0f3-6bf0f7223a4c"></td>
    <td>connect <img alt="image" src="https://github.com/user-attachments/assets/a17a4999-ae34-4705-a568-c15c428fcff5"></td>
  </tr>
  <tr>
    <td>connected <img alt="image" src="https://github.com/user-attachments/assets/d8f42595-3102-439e-9e1b-3d5b5a438148"></td>
    <td>delete <img alt="image" src="https://github.com/user-attachments/assets/5d6370ef-0d1b-4a43-bb40-1017b7044f7a"></td>
  </tr>
</table>

## How to use example

### Hardware Required

This example can run on any ESP32 development board with WiFi capability.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Usage

### First Time Setup

1. Flash the example to your ESP32 device
2. The device will create a WiFi access point named `ESP-Prov-XXXX` (where XXXX is part of the MAC address)
3. Connect to this AP using your phone or computer (password: "configure")
4. Open a web browser and navigate to `http://192.168.4.1`
5. Use the web interface to:
   - Click "Scan Networks" to see available WiFi networks
   - Select a network from the list (or manually enter SSID)
   - Enter the network password
   - Click "Connect" to test the connection
   - If successful, click "Complete Provisioning" to save and exit

### Subsequent Boots

The device stores WiFi credentials in NVS for future use. The provisioning web
interface allows you to view saved networks and reconnect to them. Note: This
example does not automatically connect to saved networks - it only demonstrates
the provisioning UI. To implement auto-connect on boot, your application should
retrieve saved credentials from NVS and use the `WifiSta` component to connect.

## Example Output


Aut-opening provisioning portal on Android when connecting to the AP:

https://github.com/user-attachments/assets/bd43125d-1641-4586-a958-39720e82f960

And on iOS:
<img width="1206" height="2622" alt="image" src="https://github.com/user-attachments/assets/be61fcba-177f-4266-a5a7-c74d05c32097" />
<img width="1206" height="2622" alt="image" src="https://github.com/user-attachments/assets/129184eb-53d4-49ec-b391-9a8021bec6b2" />
<img width="1206" height="2622" alt="image" src="https://github.com/user-attachments/assets/3e0fbd40-5176-4853-a5c2-2606d001d05b" />

Scanning for network and connecting:
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/ffc31171-8575-49a9-b5bf-0b934f030497" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/e230ce28-c1a4-4919-bac0-91757ac3f71c" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/b8a6d2de-927e-4b12-b749-9190b9cf5200" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/19e7d9e8-1c9d-4da6-adba-591ec0c886f9" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/a261f075-d780-4be9-8a0d-46b8c3e469aa" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/adcbc5db-c4c3-49fb-9a81-e78ef70303c4" />

Managing saved networks:
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/ce099bfc-2fe7-4dd6-b0f3-6bf0f7223a4c" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/a17a4999-ae34-4705-a568-c15c428fcff5" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/d8f42595-3102-439e-9e1b-3d5b5a438148" />
<img width="852" height="1898" alt="image" src="https://github.com/user-attachments/assets/5d6370ef-0d1b-4a43-bb40-1017b7044f7a" />

```console
I (415) main_task: Calling app_main()
[Provisioning Example/I][0.415]: Starting WiFi Provisioning Example
[Provisioning Example/I][0.415]: Initializing NVS...
[Provisioning/I][0.465]: Initialized with AP SSID: ESP-Prov-8590
[Provisioning Example/I][0.465]: Existing provisioned network SSID: HouseOfBoo
[WifiAp/I][0.465]: Creating network interface and ensuring WiFi stack is initialized
I (475) pp: pp rom version: e7ae62f
I (475) net80211: net80211 rom version: e7ae62f
I (495) wifi:wifi driver task: 3fcedc7c, prio:23, stack:6656, core=0
I (505) wifi:wifi firmware version: ee91c8c
I (505) wifi:wifi certification version: v7.0
I (505) wifi:config NVS flash: enabled
I (505) wifi:config nano formatting: disabled
I (505) wifi:Init data frame dynamic rx buffer num: 32
I (515) wifi:Init static rx mgmt buffer num: 5
I (515) wifi:Init management short buffer num: 32
I (525) wifi:Init dynamic tx buffer num: 32
I (525) wifi:Init static tx FG buffer num: 2
I (535) wifi:Init static rx buffer size: 1600
I (535) wifi:Init static rx buffer num: 10
I (535) wifi:Init dynamic rx buffer num: 32
I (545) wifi_init: rx ba win: 6
I (545) wifi_init: accept mbox: 6
I (545) wifi_init: tcpip mbox: 32
I (555) wifi_init: udp mbox: 6
I (555) wifi_init: tcp mbox: 6
I (555) wifi_init: tcp tx win: 5760
I (565) wifi_init: tcp rx win: 5760
I (565) wifi_init: tcp mss: 1440
I (565) wifi_init: WiFi IRAM OP enabled
I (575) wifi_init: WiFi RX IRAM OP enabled
[WifiAp/I][0.575]: Reconfiguring WiFi AP
[WifiAp/I][0.575]: WiFi AP stopped
I (585) phy_init: phy_version 711,97bcf0a2,Aug 25 2025,19:04:10
I (625) wifi:mode : softAP (f4:12:fa:5a:85:91)
I (625) wifi:Total power save buffer number: 16
I (625) wifi:Init max length of beacon: 752/752
I (625) wifi:Init max length of beacon: 752/752
I (635) esp_netif_lwip: DHCP server started on interface WIFI_AP_DEF with IP: 192.168.4.1
[WifiAp/I][0.635]: WiFi AP started
[WifiAp/I][0.645]: WiFi AP started
[WifiAp/I][0.645]: WiFi AP started, SSID: 'ESP-Prov-8590'
[Provisioning/I][0.655]: Provisioning started at http://192.168.4.1
[Provisioning Example/I][0.655]: Provisioning started
[Provisioning Example/I][0.665]: Connect to WiFi network: ESP-Prov-8590
[Provisioning Example/I][0.675]: Open browser to: http://192.168.4.1
I (7265) wifi:new:<1,0>, old:<1,1>, ap:<1,1>, sta:<255,255>, prof:1, snd_ch_cfg:0x0
I (7265) wifi:station: a2:51:f2:06:a8:34 join, AID=1, bgn, 20
[WifiAp/I][7.265]: Station join, AID=1
I (7475) wifi:<ba-add>idx:2 (ifx:1, a2:51:f2:06:a8:34), tid:0, ssn:643, winSize:64
I (7695) esp_netif_lwip: DHCP server assigned IP to a client, IP is: 192.168.4.2
I (7935) wifi:<ba-add>idx:3 (ifx:1, a2:51:f2:06:a8:34), tid:6, ssn:2922, winSize:64
[WifiAp/I][15.025]: Station leave, AID=1
I (15035) wifi:<ba-del>idx:2, tid:0
I (15035) wifi:<ba-del>idx:3, tid:6
I (15035) wifi:new:<1,0>, old:<1,0>, ap:<1,1>, sta:<255,255>, prof:1, snd_ch_cfg:0x0
I (15035) wifi:station: a2:51:f2:06:a8:34 join, AID=1, bgn, 20
[WifiAp/I][15.045]: Station join, AID=1
I (15205) wifi:<ba-add>idx:2 (ifx:1, a2:51:f2:06:a8:34), tid:0, ssn:2714, winSize:64
I (15345) esp_netif_lwip: DHCP server assigned IP to a client, IP is: 192.168.4.2
I (15655) wifi:<ba-add>idx:3 (ifx:1, a2:51:f2:06:a8:34), tid:6, ssn:3177, winSize:64
[Provisioning/I][17.785]: Starting WiFi scan...
I (17795) wifi:mode : sta (f4:12:fa:5a:85:90) + softAP (f4:12:fa:5a:85:91)
I (17795) wifi:enable tsf
I (20705) wifi:mode : softAP (f4:12:fa:5a:85:91)
[Provisioning/I][20.715]: Found 3 networks
[Provisioning/I][34.945]: Testing connection to: HouseOfBoo
[Provisioning/I][34.955]: Creating test WiFi STA instance
[WifiSta/I][34.955]: Creating network interface and ensuring WiFi stack is initialized
[WifiSta/D][34.965]: Adding event handler for WIFI_EVENT(s)
[WifiSta/D][34.965]: Adding IP event handler for IP_EVENT_STA_GOT_IP
[WifiSta/I][34.975]: Reconfiguring WiFi STA
[WifiSta/D][34.975]: Setting WiFi SSID to 'HouseOfBoo'
[WifiSta/D][34.985]: AP mode already active, setting mode to APSTA
[WifiSta/D][34.995]: Setting WiFi mode to WIFI_MODE_APSTA
I (35005) wifi:mode : sta (f4:12:fa:5a:85:90) + softAP (f4:12:fa:5a:85:91)
I (35005) wifi:enable tsf
[WifiSta/D][35.025]: Setting WiFi config
W (35025) wifi:Password length matches WPA2 standards, authmode threshold changes from OPEN to WPA2
[WifiSta/D][35.025]: WIFI_EVENT_STA_START - initiating connection
[WifiSta/D][35.035]: Setting WiFi PHY rate to MCS0_LGI (6.5-13.5 Mbps)
I (35045) wifi:primary chan differ, old=1, new=9, start CSA timer
[WifiSta/I][35.045]: WiFi STA reconfigured successfully
[WifiSta/D][35.055]: Starting WiFi
[WifiSta/I][35.055]: WiFi started
[WifiSta/I][35.065]: WiFi STA started, SSID: 'HouseOfBoo'
[Provisioning/I][35.065]: Waiting for connection (max 15 seconds)...
I (35445) wifi:switch to channel 9
I (35445) wifi:ap channel adjust o:1,1 n:9,2
I (35445) wifi:new:<9,0>, old:<1,0>, ap:<9,2>, sta:<0,0>, prof:1, snd_ch_cfg:0x0
I (35455) wifi:new:<9,2>, old:<9,0>, ap:<9,2>, sta:<9,0>, prof:1, snd_ch_cfg:0x0
I (35455) wifi:state: init -> auth (0xb0)
I (35475) wifi:state: auth -> assoc (0x0)
I (35495) wifi:state: assoc -> run (0x10)
I (35525) wifi:connected with HouseOfBoo, aid = 7, channel 9, BW20, bssid = 08:02:8e:8a:54:73
I (35525) wifi:security: WPA2-PSK, phy: bgn, rssi: -55
I (35525) wifi:pm start, type: 1

I (35525) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (35535) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (35545) wifi:dp: 2, bi: 102400, li: 4, scale listen interval from 307200 us to 409600 us
I (35555) wifi:AP's beacon interval = 102400 us, DTIM period = 2
[WifiSta/D][35.565]: WIFI_EVENT_STA_CONNECTED - waiting for IP
I (36905) wifi:<ba-add>idx:0 (ifx:0, 08:02:8e:8a:54:73), tid:6, ssn:4, winSize:64
I (37905) esp_netif_handlers: sta ip: 192.168.1.25, mask: 255.255.255.0, gw: 192.168.1.1
[WifiSta/I][37.905]: got ip: 192.168.1.25
[Provisioning/I][37.905]: Connection callback - connected to AP
[Provisioning/I][37.905]: Got IP callback - 192.168.1.25
[Provisioning/I][38.015]: Connection test result: true (got_ip=true, failed=false)
[Provisioning/I][38.015]: Cleaning up test STA
[WifiSta/D][38.015]: Destroying WiFiSta
[WifiSta/D][38.015]: Unregistering event handlers
[WifiSta/D][38.025]: Unregistering any wifi event handler
[WifiSta/D][38.035]: Unregistering got ip event handler
[WifiSta/D][38.035]: In APSTA mode, disconnecting STA interface
I (38045) wifi:state: run -> init (0x0)
I (38065) wifi:pm stop, total sleep time: 1777943 us / 2533745 us

I (38065) wifi:<ba-del>idx:0, tid:6
I (38065) wifi:new:<9,0>, old:<9,2>, ap:<9,2>, sta:<9,0>, prof:1, snd_ch_cfg:0x0
I (38065) wifi:mode : softAP (f4:12:fa:5a:85:91)
[WifiSta/I][38.075]: WiFi STA disconnected, AP still running
[Provisioning/I][38.285]: Saving credentials for: HouseOfBoo
[Provisioning Example/I][40.965]: Provisioned successfully!
[Provisioning Example/I][40.965]:   SSID: HouseOfBoo
[Provisioning Example/I][40.965]:   Password: ********
[Provisioning Example/I][40.965]: Credentials saved to NVS
[Provisioning Example/I][41.085]: Provisioning completed by user, stopping service
I (41185) wifi:station: a2:51:f2:06:a8:34 leave, AID = 1, reason = 2, bss_flags is 33721443, bss:0x3fcb5a9c
I (41185) wifi:new:<9,0>, old:<9,0>, ap:<9,2>, sta:<255,255>, prof:1, snd_ch_cfg:0x0
I (41185) wifi:<ba-del>idx:2, tid:0
I (41195) wifi:<ba-del>idx:3, tid:6
I (41195) wifi:new:<9,2>, old:<9,0>, ap:<9,2>, sta:<255,255>, prof:1, snd_ch_cfg:0x0
I (41255) wifi:flush txq
I (41255) wifi:stop sw txq
I (41255) wifi:lmac stop hw txq
[WifiAp/I][41.255]: WiFi AP stopped
[Provisioning/I][41.255]: Provisioning stopped
[Provisioning Example/I][41.255]: Provisioning example finished
```

## Configuration

The example can be configured through `idf.py menuconfig` under "Provisioning Example Configuration":

- **AP SSID**: Name of the provisioning access point (default: "ESP-Prov")
- **AP Password**: Password for the provisioning AP (default: "configure")
- **Device Name**: Friendly name shown in web interface
- **Auto-generate unique SSID**: Append MAC address to SSID to avoid conflicts
- **Auto-shutdown AP**: Automatically stop AP after successful provisioning
- **Web Server Port**: HTTP server port (default: 80)

## Web Interface Features

The embedded web interface provides:

- **Network Scanner**: Shows available WiFi networks with signal strength indicators
- **Manual Entry**: Option to manually enter SSID for hidden networks
- **Connection Testing**: Validates credentials before saving
- **Credential Management**: View, delete, and reconnect to stored networks
- **Responsive Design**: Works on mobile phones, tablets, and desktop browsers
- **Real-time Feedback**: Shows connection status and error messages
