# Ads1x15 I2C ADC Expander

The `ADS1x15` provides a class for communicating with the ADS1x15 (ADS1015 and
ADS1115) family of I2C ADC chips with configurable gain and sampling rate.

## Example

The [example](./example) shows the use of the `Ads1x15` component to communicate
with an ADS1015 I2C analog to digital converter using the I2C peripheral of the
ESP32.

It uses the `task` component to periodically read two channels of analog data
from the ADS1015 and print them in CSV format using the `format` component.
