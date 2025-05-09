# VL53LXX Time of Flight (Tof) Distance Sensor Peripheral Component

[![Badge](https://components.espressif.com/components/espp/vl53l/badge.svg)](https://components.espressif.com/components/espp/vl53l)

The
[Vl53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) is
a Time-of-Flight (ToF) distance sensor. It is a sensor that can measure distance
from a target object without touching it. The sensor uses a laser to send a
pulse of light and measures the time it takes for the light to reflect back to
the sensor. The sensor can measure distance up to 2 meters with a resolution of
1mm. Because of the way the sensor works, it is able to measure multiple
distances at the same time.

The sensor can struggle with outdoor use, as sunlight can interfere with the
sensor's ability to measure distance. The sensor is also not able to measure
distance through glass or other transparent materials.

## Example

The [example](./example) shows the use of the `Vl53l` component to communicate
with a VL53L0X time-of-flight distance sensor.

