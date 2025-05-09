# ADC (Analog to Digital Converter) Component

[![Badge](https://components.espressif.com/components/espp/adc/badge.svg)](https://components.espressif.com/components/espp/adc)

This component provides some classes which encapsulate analog value sampling
using the ESP's built-in ADC hardware and peripheral drivers.

## Oneshot ADC

The `OneshotAdc` allows the user a simple, low-resource way to sporadically (and
with moderate frequency needs) measure an analog voltage for multiple channels
on a single ADC UNIT. It does not start or manage any tasks and does not perform
any filtering on the data. Each time the user calls `read_raw(adc_channel_t)` or
`read_mv(adc_channel_t)`, it block and trigger an analog read for the associated
channel (if it was configured to do so).

## Continuous ADC

The `ContinuousAdc` provides a mechanism for high-frequency, continuous,
deterministic sampling of analog voltages for multiple channels (potentially
across multiple ADC units, depending on the ESP32 chip used). It does this be
enabling the continuous ADC DMA mode and then running its own task which
retrieves the data and filters it. When the user calls `get_mv(adc_channel_t)`,
it simply returns the most recent filtered value for that channel, if it was
configured.

## Example

There is an [example](./example) which shows the use of the `espp::OneshotAdc`
and the `espp::ContinuousAdc` components.
