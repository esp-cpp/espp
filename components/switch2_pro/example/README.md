# Switch 2 Pro Controller Example

Brings up the emulated Switch 2 Pro Controller and connects to a real Nintendo
Switch 2 console. On boot it runs the pairing-crypto self-test, stands up the
custom Nintendo GATT services, and advertises with Nintendo manufacturer data.
Once paired it streams input reports continuously (like a real controller); the
board's BOOT button doubles as the **A** button while connected, and — when
bonded but disconnected — a BOOT press broadcasts the wake advertisement to wake
the console.

> **Supported target: ESP32-C6** (default). Pairing, input, reconnect, and
> wake-from-sleep all work against a real console. RISC-V siblings (C61/C2/H2)
> use the same open NimBLE controller as the C6.
>
> **ESP32-S3** also builds and pairs. Its 5 ms reconnect/wake support is now
> official in ESP-IDF via `CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`
> (default on; needs a recent IDF — see the component README); full on-hardware
> S3 verification is still pending on our side.

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

The ESP32-S3 also builds (`idf.py set-target esp32s3`), but see the caveat above.

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

### Reconnect and wake (5 ms patch enabled)

After the first pairing the bond is saved to NVS. On the next boot the controller
advertises for reconnection. With the console asleep, press **BOOT** while the
log shows `connected=false` to broadcast the wake advertisement — the console
should power on and reconnect without re-pairing.

## Feeding real input

`set_input_report()` just stores the latest state; a driver-owned task paces the
BLE notifications. Replace the BOOT-button read in `main` with your real
button/stick source and call `set_input_report()` whenever state changes.
