# espp BLDC Haptics Console (WebUSB)

`haptics_console.html` is the single-file browser console for the
`bldc_haptics` USB example: live knob dial + telemetry, detent-preset
switching, control commands and firmware (OTA) updates over the example's USB
vendor / WebUSB interface.

It is a **symlink** to the authoritative copy that lives next to the example it
speaks to: [`../example/webapp/index.html`](../example/webapp/index.html) (edit
that file). It sits in this `web/` directory so the docs CI hosts it at
<https://esp-cpp.github.io/espp/apps/haptics_console.html> — the WebUSB
landing page the example firmware advertises.

The wire protocol is documented in
[`../example/PROTOCOL.md`](../example/PROTOCOL.md); usage instructions are in
the [example README](../example/README.md).

WebUSB requires a Chromium-based browser (Chrome / Edge / Opera) on a secure
origin — `https://`, `http://localhost`, or a `file://` URL.
