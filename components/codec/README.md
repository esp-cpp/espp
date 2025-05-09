# Codec (Audio) Component

[![Badge](https://components.espressif.com/components/espp/codec/badge.svg)](https://components.espressif.com/components/espp/codec)

The `codec` component contains a couple audio codec components copied and
modified from [esp-adf](https://github.com/espressif/esp-adf), specifically from
the
[esp_codec_dev](https://github.com/espressif/esp-adf/tree/master/components/esp_codec_dev)
component.

It supports the following components:
- es7210 microphone input
- es8311 audio output
- es8388 audio output

These components needed to be modified to better support defining custom `write`
and `read` functions, to better make use of `espp::I2c` and functional
interfaces.
