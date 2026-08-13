# Switch 2 Pro Controller Example

Brings up the emulated Switch 2 Pro Controller (milestones 1–2: GATT + pairing).
On boot it runs the pairing-crypto self-test, stands up the custom Nintendo GATT
services, and advertises with Nintendo manufacturer data. DEBUG logging traces
every command write and response so you can watch the handshake with a real
console.

## Build, flash, monitor

Default target is **esp32s3** (what most people have on hand):

```bash
idf.py build flash monitor
```

For the full path on a RISC-V board (the console's 5 ms interval works there):

```bash
idf.py set-target esp32c6
idf.py menuconfig   # Switch 2 Pro Controller -> enable the 5 ms NimBLE patch
idf.py build flash monitor
```

## Testing pairing with a real Switch 2

1. Flash and open the monitor. You should see `pairing crypto self-test passed`
   and `advertising as 'Pro Controller'`.
2. On the Switch 2: **System Settings → Controllers → Pair New Controllers**
   (or the Change Grip/Order screen — but note that screen is known to be
   flaky; prefer the Controllers menu).
3. Watch the log:
   - `connected: peer=… interval=…ms supervision=…ms` — **the interval is the
     key number.** The Switch 2 drives **5 ms**. If you see ~5 ms here on S3,
     great; if the console forces 5 ms and S3 can't hold it, expect a
     `disconnected: … reason=…` shortly after (often a supervision timeout).
   - `cmd<-0x0016 […]: 15 91 01 04 …` / `rsp->0x001e […]: 15 01 01 04 …` — the
     0x15 pairing exchange (addresses → keys → confirm → finalise).
   - `pairing: finalised — bonded` — the handshake completed.

### What to expect on S3

S3 **cannot** accept the console's sub-spec 5 ms connection interval (the
NimBLE 5 ms patch is RISC-V-only; see the component `DESIGN.md`). Depending on
how the console negotiates the interval, S3 may connect and get partway through
pairing before the link drops, or drop soon after connecting. The **logs show
exactly how far it got** — which is the useful data. A clean pass-through to
`pairing: finalised` and a stable connection is expected only on C6/C61 with
the patch enabled.

If pairing completes but the controller isn't fully usable, that's milestone 3
(input report streaming), which also needs the 5 ms interval.
