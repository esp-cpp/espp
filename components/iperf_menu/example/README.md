# Iperf Menu Example

This example shows the use of the `espp::IperfMenu` component to measure the
throughput of a network (WiFi / ethernet) connection between an ESP chip and
another device running iperf.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Iperf Menu Example](#iperf-menu-example)
  - [How to use example](#how-to-use-example)
    - [Build and Flash](#build-and-flash)
    - [Configuration](#configuration)
    - [Example Output](#example-output)

<!-- markdown-toc end -->

![CleanShot 2025-07-07 at 13 27 54](https://github.com/user-attachments/assets/3eedd8e3-b864-4385-9b7b-46d573480e90)

This supports:

- client operation
- server operation
- ipv4/ipv6

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Configuration

This example uses WiFi (specifically WiFi STA), so you should use the `wifi`
command line interface (CLI) menu that the example outputs over the serial
monitor to configure the WiFi network.

Once you're connected, you can use the `iperf` menu to select the operation mode
(client or server) and set the parameters for the test.

### Example Output

![CleanShot 2025-07-07 at 14 27 02@2x](https://github.com/user-attachments/assets/9053320f-40c6-45b6-9224-9f2c16ca7960)

![CleanShot 2025-07-07 at 13 27 54](https://github.com/user-attachments/assets/3eedd8e3-b864-4385-9b7b-46d573480e90)


```console
I (399) main_task: Calling app_main()
[iperf_example/I][0.015]: Starting example...
I (429) pp: pp rom version: e7ae62f
I (429) net80211: net80211 rom version: e7ae62f
I (439) wifi:wifi driver task: 3fcb05cc, prio:23, stack:6656, core=0
I (449) wifi:wifi firmware version: 79fa3f41ba
I (449) wifi:wifi certification version: v7.0
I (449) wifi:config NVS flash: enabled
I (449) wifi:config nano formatting: disabled
I (449) wifi:Init data frame dynamic rx buffer num: 32
I (449) wifi:Init static rx mgmt buffer num: 5
I (449) wifi:Init management short buffer num: 32
I (449) wifi:Init dynamic tx buffer num: 32
I (449) wifi:Init static tx FG buffer num: 2
I (449) wifi:Init static rx buffer size: 1600
I (449) wifi:Init static rx buffer num: 10
I (449) wifi:Init dynamic rx buffer num: 32
I (449) wifi_init: rx ba win: 6
I (449) wifi_init: accept mbox: 6
I (449) wifi_init: tcpip mbox: 32
I (449) wifi_init: udp mbox: 6
I (449) wifi_init: tcp mbox: 6
I (449) wifi_init: tcp tx win: 5760
I (449) wifi_init: tcp rx win: 5760
I (449) wifi_init: tcp mss: 1440
I (449) wifi_init: WiFi IRAM OP enabled
I (449) wifi_init: WiFi RX IRAM OP enabled
[WifiSta/D][0.069]: SSID is empty, trying to load saved WiFi configuration
[WifiSta/I][0.070]: Loaded saved WiFi configuration: SSID = 'HouseOfBoo'
I (459) phy_init: phy_version 700,8582a7fd,Feb 10 2025,20:13:11
I (489) wifi:mode : sta (f4:12:fa:5a:85:90)
I (489) wifi:enable tsf
I (499) wifi:new:<6,0>, old:<1,0>, ap:<255,255>, sta:<6,0>, prof:1, snd_ch_cfg:0x0
I (499) wifi:state: init -> auth (0xb0)
example> I (529) wifi:state: auth -> assoc (0x0)
I (539) wifi:state: assoc -> run (0x10)
I (659) wifi:connected with HouseOfBoo, aid = 1, channel 6, BW20, bssid = 3c:28:6d:bf:ed:ad
I (669) wifi:security: WPA2-PSK, phy: bgn, rssi: -63
I (669) wifi:pm start, type: 1

I (669) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (669) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (679) wifi:dp: 2, bi: 102400, li: 4, scale listen interval from 307200 us to 409600 us
I (679) wifi:AP's beacon interval = 102400 us, DTIM period = 2
I (689) wifi:<ba-add>idx:0 (ifx:0, 3c:28:6d:bf:ed:ad), tid:6, ssn:3, winSize:64
I (1399) wifi:<ba-add>idx:1 (ifx:0, 3c:28:6d:bf:ed:ad), tid:0, ssn:0, winSize:64
I (1889) esp_netif_handlers: sta ip: 192.168.86.32, mask: 255.255.255.0, gw: 192.168.86.1
[WifiSta/I][1.502]: got ip: 192.168.86.32
[iperf_example/I][1.503]: got IP: 192.168.86.32

example>
example> iperf
iperf> client_tcp 192.168.86.21 5001 0 10
Using iperf config: iperf_cfg_t client {  host: 358000832,  dest port: 5001,  source port: 5001,  protocol: TCP,  duration: 30,  bandwidth: -1,  buffer_size: 0 }
iperf> I (23009) iperf: Successfully connected

Interval       Bandwidth
 0.0-10.0 sec  6784.00 Kbits/sec
10.0-20.0 sec  5964.80 Kbits/sec
20.0-30.0 sec  4377.60 Kbits/sec
 0.0-30.0 sec  5708.80 Kbits/sec
I (53029) iperf: TCP Socket client is closed.
I (53029) iperf: iperf exit

iperf> client_udp 192.168.86.21 5001 0 1
Using iperf config: iperf_cfg_t client {  host: 358000832,  dest port: 5001,  source port: 5001,  protocol: UDP,  duration: 30,  bandwidth: -1,  buffer_size: 0 }
iperf> I (101369) iperf: Socket created, sending to 358000832:5001

Interval       Bandwidth
 0.0- 1.0 sec  8130.94 Kbits/sec
 1.0- 2.0 sec  7315.55 Kbits/sec
```
