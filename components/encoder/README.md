# ABI Encoder Component

[![Badge](https://components.espressif.com/components/espp/encoder/badge.svg)](https://components.espressif.com/components/espp/encoder)

The `AbiEncoder` allow the user a configurable container wrapping the `pulse
count
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html>`_
ESP peripheral api as it would be configured for an incremental encoder with
quadrature output (see `Wikipedia
<https://en.wikipedia.org/wiki/Incremental_encoder>`_). The AbiEncoder can be
configured to be either `LINEAR` or `ROTATIONAL`, and provides access to the
current `count` of the encoder (including the overflow underflow conditions).

## Example

The [example](./example) shows the use of the `espp::AbiEncoder` from the
`encoder` component. It showcases both the `EncoderType::ROTATIONAL` and
`EncoderType::LINEAR` uses of the `AbiEncoder`.
