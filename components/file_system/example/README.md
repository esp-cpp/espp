# File System Example

This example shows how the `file_system` component can be used to manage files
on a [littlefs](https://github.com/littlefs-project/littlefs) file system
(wrapping the associated [esp_littlefs
component](https://github.com/joltwallet/esp_littlefs)).

The example further shows how POSIX / newlib APIs can be used alongside (most
of) the `std::filesystem` APIs.

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

![CleanShot 2024-06-28 at 09 08 19](https://github.com/esp-cpp/espp/assets/213467/ae2791b0-e240-4a55-808d-fed00335b4e4)

