# Binary Log Example

This example shows how to use the `binary-log` component to format and store
logs in a binary packed form, either to disk or into a ringbuffer in memory.

This component simply exposes the
[p-ranav/binary_log](https://github.com/p-ranav/binary_log) binary logger, with
some modifications in a fork
[finger563/binary_log](https://github.com/finger563/binary_log) to allow packer
customization as is required on embedded systems such as with ESP-IDF.

The `binary-log` does not support all datatypes, but only some base datatypes
such as integer types, float types, and string types.

The logger produces the following data:
1. Log data - these are indexes and the arguments which point to either the
   index data or the runfile data.
2. Index data - these are static data (format strings, constant arguments, etc.)
3. Runfile data - these are indices of the log file which run multiple times in
   a row.
   
For more information about the logging, its format, and the types supported,
please see [p-ranav/binary_log](https://github.com/p-ranav/binary_log).

## How to use example

This example is designed to be run on an ESP dev board which has embedded
flash - the partitions and build are configured to match the QtPy ESP32-S3,
though if you modify the `sdkconfig.defaults` and the `partitions.csv` you could
run it on other dev boards easily.

The example does the following:
1. Use standard `binary_log::packer` to log in a loop for three seconds to files
   on the filesystem.
2. Print the contents of the log files.
3. Erase the file system.
4. Use the `binary_log::ringbuffer_packer` to log in a loop for three seconds to
   into an in-memory ringbuffer.
5. Print the contents of the log buffers.

### Unpacking the logs

#### Getting the log data

You can copy the text from the example output into
[write_files.py](./write_files.py), then run that script, and it will produce
the relevant `log.out`, `log.out.index`, and `log.out.runfile` on your computer.

#### Unpacking the log data

Once you've done that, you can unpack the log data using the `unpacker` that is
automatically built when you build the app and is located within your `build`
folder as `build/unpacker`:

``` console
./build/unpacker log.out > log.inflated
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2025-01-31 at 14 31 23](https://github.com/user-attachments/assets/72e3c118-d5c0-4e48-b5ab-732e6b32e433)
