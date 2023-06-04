# PID Example

This example shows how to use the `espp::Pid` class to perform closed-loop
feedback control using proportional-integral-derivative (PID) control.

The example does not operate on real hardware, but simply runs the PID algorithm
on fake data.

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![output](https://user-images.githubusercontent.com/213467/228945440-0f98cffe-33fa-47b0-94a7-eaafb3062ed3.png)
