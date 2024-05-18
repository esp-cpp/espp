# Serialization Example

This example shows the use of the `NVS` component to save a variable to the NVS and load 
it after reset. 

## How to use example

The example outputs a log that counts up the number of times the system has been reset 
since the NVS was last cleared. 

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

