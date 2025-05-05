# Ads7138 I2C ADC Expander

The `Ads7138` class implements support for the Texas Instruments ADS7138 12-bit
8-channel ADC. The ADS7138 is a 12-bit, 8-channel, low-power, successive
approximation register (SAR) analog-to-digital converter (ADC) which can
configure any of its 8 channels as single-ended analog inputs, digital inputs,
or digital outputs. It has an operating mode that allows the user to configure
the device for a single conversion, or to automatically convert on a continuous
basis.

## Example

The [example](./example) shows the use of the `Ads7138` component to communicate
with an ADS7138 I2C analog to digital converter using the I2C peripheral of the
ESP32.

