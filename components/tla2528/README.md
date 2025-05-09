# Tla2528 ADC Exapander Peripheral Component

https://components.espressif.com/components/espp/tla2528/badge.svg

The `Tla2528` class implements support for the Texas Instruments TLA2528 12-bit
8-channel ADC. The TLA2528 is a 12-bit, 8-channel, low-power, successive
approximation register (SAR) analog-to-digital converter (ADC) which can
configure any of its 8 channels as single-ended analog inputs, digital inputs,
or digital outputs. It has an operating mode that allows the user to configure
the device for a single conversion, or to automatically convert on a sequenced
basis.

## Example

The [example](./example) shows the use of the `Tla2528` component to communicate
with an TLA2528 I2C analog to digital converter using the I2C peripheral of the
ESP32.

