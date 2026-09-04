# Switch 2 Pro Controller Example

Brings up the emulated Switch 2 Pro Controller and connects to a real Nintendo
Switch 2 console. On boot it runs the pairing-crypto self-test, stands up the
custom Nintendo GATT services, and advertises with Nintendo manufacturer data.
Once paired it streams input reports continuously (like a real controller); the
board's BOOT button doubles as the **A** button while connected, and — when
bonded but disconnected — a BOOT press broadcasts the wake advertisement to wake
the console.

> **Supported targets: ESP32-C6** (default) **and ESP32-S3**. Pairing, input,
> reconnect, and wake-from-sleep all work against a real console. RISC-V siblings
> (C61/C2/H2) use the same open NimBLE controller as the C6.
>
> **ESP32-S3** works with **ESP-IDF ≥ v6.1**, where the official
> `CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE` (default on) gives it the console's
> 5 ms reconnect/wake with no binary patch — verified on real hardware.

## Build, flash, monitor

Default target is **esp32c6**:

```bash
idf.py build flash monitor
```

Reconnect and wake-from-sleep require the console's sub-spec 5 ms connection
interval, which needs the opt-in controller patch (off by default because it
modifies your global `$IDF_PATH` install). Fresh pairing and input streaming
work **without** it. To enable it:

```bash
idf.py menuconfig   # Component config -> Switch 2 Pro -> enable the 5 ms NimBLE patch
# or: python ../tools/patch_nimble_5ms.py --target esp32c6   (undo with --restore)
python ../tools/smoke_test_5ms.py --target esp32c6   # verify (no hardware needed)
idf.py build flash monitor
```

The ESP32-S3 works out of the box on ESP-IDF ≥ v6.1 (`idf.py set-target esp32s3`);
no patch is needed there — the official `CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`
is default-on and covers the console's 5 ms reconnect/wake.

## Testing with a real Switch 2

1. Flash and open the monitor. You should see `pairing crypto self-test passed`,
   the GATT handle map, and `Switch2Pro advertising as 'Pro Controller'`.
2. On the Switch 2: **System Settings → Controllers → Pair New Controllers**.
3. Watch the log:
   - `connected: peer=… interval=…ms` — the negotiated connection interval.
   - `pairing: finalised — bonded` then `AUTH complete: encrypted=true` — the
     handshake and link encryption completed.
   - `input-report streaming ENABLED (0x000e)` — the console subscribed; input
     now streams.
4. Open **Test Input Devices** (Controllers → *your* controller) and press the
   board's **BOOT** button — **A** should register on screen.

### Reconnect and wake

After the first pairing the bond is saved to NVS. On the next boot the controller
advertises for reconnection. With the console asleep, press **BOOT** while the
log shows `connected=false` to broadcast the wake advertisement — the console
should power on and reconnect without re-pairing. (On the S3 this needs
ESP-IDF ≥ v6.1; on C6-family chips it needs the 5 ms controller patch.)

### Verified on ESP32-S3 (ESP-IDF v6.1)

A full session on a real Switch 2 — fresh pair, input testing, then powering the
console off and back on with reconnect + input still working — with the official
`CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE` (default on, **no binary patch**).
The key moments:

- `connected: … interval=15.00ms` — fresh pairing connects at the spec-legal
  15 ms, and the 0x15 handshake completes there.
- `CONN PARAMS UPDATE: itvl=5.00ms` — **~1.5 s after streaming starts the console
  drops the link to 5 ms** and streams there for the rest of the session. On the
  pre-v6.1 controller lib this is exactly where the old "~3 s tx-stall" hit; v6.1
  applies the update cleanly and input keeps flowing.
- After the console is powered off (`REMOTE_USER_TERMINATED`) and a `wake`
  broadcast, it reconnects **straight at `interval=5.00ms`** and input resumes.

<details>
<summary>Full monitor log (repetitive heartbeat lines elided)</summary>

```
[Switch2Pro/I][0.223]: pairing crypto self-test passed
I (224) BLE_INIT: BT controller compile version [51d9dfd]
I (226) BLE_INIT: Bluetooth MAC: f4:12:fa:5a:85:92
[Switch2Pro/I][0.289]: BLE address: using PUBLIC
[Switch2Pro/I][0.290]:   common_input  = 0x000a (0x000a)
[Switch2Pro/I][0.291]:   pro2_input    = 0x000e (0x000e)
[Switch2Pro/I][0.291]:   command       = 0x0014 (0x0014)
[Switch2Pro/I][0.291]:   vib_command   = 0x0016 (0x0016)
[Switch2Pro/I][0.292]:   resp1         = 0x001a (0x001a)
[Switch2Pro/I][0.292]:   resp2         = 0x001e (0x001e)
[Switch2Pro/I][0.296]: Switch2Pro advertising as 'Pro Controller'

# --- fresh pairing, connects at 15 ms ---
[Switch2Pro/I][5.331]: connected: peer=C8:48:05:64:A0:BB interval=15.00ms supervision=2000ms latency=0
[Switch2Pro/I][6.291]: SUBSCRIBE resp1(0x001a) value=0x0001 (on)
[Switch2Pro/I][6.321]: SUBSCRIBE resp2(0x001e) value=0x0001 (on)
[Switch2Pro/I][6.501]: pairing: stored console identity addr c8:48:05:64:a0:bb
[Switch2Pro/I][6.503]: pairing: exchange addresses -> replied with our address 92 85 5a fa 12 f4 (little-endian)
[Switch2Pro/I][6.532]: pairing: exchange keys -> LTK derived, replied B1
[Switch2Pro/I][6.578]: pairing: confirm -> replied B2
[Switch2Pro/I][6.607]: pairing: finalised — bonded
[Switch2Pro/I][6.611]: injected LTK into NimBLE store (rc=0) — ready for LL encryption
[Switch2Pro/I][6.613]: saved bond to NVS (console addr + LTK)
[Switch2Pro/I][6.710]: AUTH complete: encrypted=true bonded=true authenticated=true
[Switch2Pro/I][7.176]: SUBSCRIBE pro2_input(0x000e) value=0x0001 (on)
[Switch2Pro/I][7.176]: input-report streaming ENABLED (0x000e)

# --- console drops the FRESH session to 5 ms ~1.5 s after streaming begins ---
[Switch2Pro/I][7.699]: LINK CHANGE: itvl=15.00ms latency=0 timeout=2000ms tx_phy=2 rx_phy=2
[Switch2Pro/I][8.666]: CONN PARAMS UPDATE: itvl=5.00ms latency=0 timeout=2000ms
[Switch2Pro/I][8.717]: LINK CHANGE: itvl=5.00ms latency=0 timeout=2000ms tx_phy=2 rx_phy=2

# --- streams fine at 5 ms; BOOT press registers as A ---
[switch2_pro example/I][9.749]: connected=true streaming=true A(boot)=false ...
   ... (streaming continuously at 5 ms) ...
[switch2_pro example/I][32.944]: connected=true streaming=true A(boot)=true L+R(auto)=false
   ...

# --- console powered off ---
[Switch2Pro/I][40.616]: SUBSCRIBE pro2_input(0x000e) value=0x0000 (off)
[Switch2Pro/W][40.620]: disconnected: peer=C8:48:05:64:A0:BB reason=REMOTE_USER_TERMINATED (paired=true)
[Switch2Pro/I][40.623]: advertising (reconnect): flags+mfr(26 B) in adv, name in scan response
   ... (advertising for reconnect) ...

# --- BOOT press broadcasts wake; console powers back on and reconnects at 5 ms ---
[Switch2Pro/I][54.114]: wake: broadcasting wake advertisement (user-requested)
[switch2_pro example/I][54.118]: BOOT pressed while disconnected -> sent wake advertisement
[Switch2Pro/I][56.801]: connected: peer=C8:48:05:64:A0:BB interval=5.00ms supervision=2000ms latency=0
[Switch2Pro/I][57.225]: AUTH complete: encrypted=true bonded=true authenticated=true
[Switch2Pro/I][57.226]: SUBSCRIBE pro2_input(0x000e) value=0x0001 (on)
[Switch2Pro/I][57.227]: input-report streaming ENABLED (0x000e)
[Switch2Pro/I][57.731]: LINK CHANGE: itvl=5.00ms latency=0 timeout=2000ms tx_phy=2 rx_phy=2

# --- input works again after wake/reconnect; BOOT press registers as A ---
[switch2_pro example/I][63.454]: connected=true streaming=true A(boot)=true L+R(auto)=false
   ...
```

</details>

## Feeding real input

`set_input_report()` just stores the latest state; a driver-owned task paces the
BLE notifications. Replace the BOOT-button read in `main` with your real
button/stick source and call `set_input_report()` whenever state changes.
