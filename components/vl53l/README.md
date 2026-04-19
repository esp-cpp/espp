# VL53LXX Time of Flight (Tof) Distance Sensor Peripheral Component

[![Badge](https://components.espressif.com/components/espp/vl53l/badge.svg)](https://components.espressif.com/components/espp/vl53l)

The
[VL53L4CD](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html)
(and [VL53L4CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cx.html))
is a Time-of-Flight (ToF) distance sensor. It is a sensor that can measure
distance from a target object without touching it. The sensor uses a laser to
send a pulse of light and measures the time it takes for the light to reflect
back to the sensor. The sensor can measure distance up to 1.3 meters (VL53L4CD)
or up to 6 meters (VL53L4CX) with a resolution of 1mm.

> **Note:** This component is designed for the **VL53L4CD** and **VL53L4CX**
> sensors, which use a 16-bit register addressing scheme. It is **not**
> compatible with the older VL53L0X or VL53L1X sensors, which use a different
> register map and communication protocol.

The sensor can struggle with outdoor use, as sunlight can interfere with the
sensor's ability to measure distance. The sensor is also not able to measure
distance through glass or other transparent materials.

## Example

The [example](./example) shows the use of the `Vl53l` component to communicate
with a VL53L4CD time-of-flight distance sensor.

