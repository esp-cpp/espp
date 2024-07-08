# NVS Example

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

## NVS Example Output

![CleanShot 2024-05-18 at 12 54 46](https://github.com/esp-cpp/espp/assets/213467/60b2db2f-8796-4ae3-9a8c-51f69fa21911)
![CleanShot 2024-05-18 at 12 54 54](https://github.com/esp-cpp/espp/assets/213467/ddceddbf-0690-4590-93b6-66cf91ad1898)
![CleanShot 2024-05-18 at 12 55 02](https://github.com/esp-cpp/espp/assets/213467/1181fc79-f7bd-4b1d-b351-e5ca24ee7c55)

## NVSHandle Example Output

First run:
```
Opening Non-Volatile Storage (NVS) handle... Done
Reading restart counter from NVS ... [NVSHandle/E][0.061]: The value is not initialized in NVS, key = 'restart_counter'
The value is not initialized yet!
Updating restart counter in NVS ... Done
Committing updates in NVS ... Done

Restarting in 10 seconds...
Restarting in 9 seconds...
Restarting in 8 seconds...
Restarting in 7 seconds...
Restarting in 6 seconds...
Restarting in 5 seconds...
Restarting in 4 seconds...
Restarting in 3 seconds...
Restarting in 2 seconds...
Restarting in 1 seconds...
Restarting in 0 seconds...
Restarting now.
```

Subsequent runs:

```
Opening Non-Volatile Storage (NVS) handle... Done
Reading restart counter from NVS ... Done
Restart counter = 1
Updating restart counter in NVS ... Done
Committing updates in NVS ... Done

Restarting in 10 seconds...
Restarting in 9 seconds...
Restarting in 8 seconds...
Restarting in 7 seconds...
Restarting in 6 seconds...
Restarting in 5 seconds...
Restarting in 4 seconds...
Restarting in 3 seconds...
Restarting in 2 seconds...
Restarting in 1 seconds...
Restarting in 0 seconds...
Restarting now.
```

Restart counter will increment on each run.

To reset the counter, erase the contents of flash memory using `idf.py erase-flash`, then upload the program again as described above.
