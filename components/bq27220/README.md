# BQ27220 I2C Battery Fuel Gauge Component

[![Badge](https://components.espressif.com/components/espp/bq27220/badge.svg)](https://components.espressif.com/components/espp/bq27220)

The Texas Instruments BQ27220 is a single-cell Li-Ion battery fuel gauge (gas
gauge) that uses the compensated end-of-discharge voltage (CEDV) algorithm to
provide accurate state-of-charge and remaining-capacity information without
requiring the host to maintain a battery-learn cycle.

The BQ27220 reports battery voltage, instantaneous and average current, average
power, temperature, state of charge, state of health, remaining and full-charge
capacity, cycle count, and time-to-empty / time-to-full estimates over the I2C
interface. All data values are 16-bit little-endian.

## Example

The [example](./example) shows how to use the BQ27220 driver to talk to the
BQ27220 and retrieve the current battery:
* Voltage (mV)
* Current (mA)
* State of Charge (%)
* Temperature (°C)
