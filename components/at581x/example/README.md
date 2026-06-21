# AT581X Example

<img width="2470" height="1860" alt="image" src="https://github.com/user-attachments/assets/697434b1-7415-4ac7-b73a-6a8f4b92080f" />


This example demonstrates the use of the `espp::At581x` driver to configure the
AT581X 5.8 GHz radar presence/motion sensor (e.g. the MS58-3909S68U4 module on
the ESP32-S3-BOX-3 sensor dock) over I2C.

It:

- creates an I2C bus (default SDA=GPIO41, SCL=GPIO40 for the BOX-3 sensor dock),
- constructs the `At581x` driver, which writes the configuration to the chip,
- periodically changes the sensing distance (sensitivity) at runtime, and
- attaches an `espp::Interrupt` to the radar output GPIO
  (`CONFIG_EXAMPLE_RADAR_OUTPUT_GPIO`, default GPIO21 on the BOX-3 dock) and
  prints presence/motion transitions. Set it to -1 to disable.

When the **ESP32-S3-BOX-3** hardware profile is selected (the default on
esp32s3), the example also uses the `esp-box` BSP to show the live radar status
on the screen — a color-coded presence label plus the detection count and
configured sensing distance. The radar is placed on `I2C_NUM_1` in this mode so
it doesn't collide with the box's internal I2C bus (`I2C_NUM_0`, GPIO8/18).

On the ESP32-S3-BOX-3 with the sensor dock attached, the defaults (SDA=41,
SCL=40, radar output=21) should work as-is.

## Configuration

Use `idf.py menuconfig` → *Example Configuration* to set the I2C pins and the
radar output GPIO for your hardware.

## Build and Flash

```sh
idf.py set-target esp32s3
idf.py build flash monitor
```

## Output

The example logs the configuration it applies and, when the radar output GPIO is
wired up, `Radar presence DETECTED` / `Radar presence cleared` as the sensor
sees motion within its configured range.


<img width="2470" height="1860" alt="image" src="https://github.com/user-attachments/assets/a7b6f343-5e3f-49ad-b927-bb0eeef25655" />

<img width="2470" height="1860" alt="image" src="https://github.com/user-attachments/assets/697434b1-7415-4ac7-b73a-6a8f4b92080f" />


```console
I (188) main_task: Calling app_main()
[at581x example/I][0.188]: Starting AT581X radar example
[At581x/I][0.190]: Writing config: freq=5800MHz, sensing_distance=700, gain=3, power=70uA, trigger_base=500ms, trigger_keep=1000ms, protect=1000ms, selfcheck=2000ms
[At581x/I][0.193]: Resetting RF frontend
[at581x example/I][0.196]: Watching radar output on GPIO 21
[at581x example/I][0.196]: Radar presence DETECTED
[Interrupt/W][0.197]: ISR service already installed, not installing again
W (199) ledc: the binded timer can't keep alive in sleep
[at581x example/I][2.322]: Radar presence cleared
[at581x example/I][5.057]: Radar presence DETECTED
[at581x example/I][7.838]: Radar presence cleared
[at581x example/I][14.645]: Radar presence DETECTED
[at581x example/I][16.186]: Radar presence cleared
```
