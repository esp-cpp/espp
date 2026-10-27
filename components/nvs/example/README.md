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

## NVS Validation Output

The last section of the example exercises the NVS operations (integer, bool,
float, string and binary-blob round trips through `get_or_set_var` / `set_var` / `get_var`,
key and namespace erase) with explicit pass / fail checks in a `validate`
namespace that is erased before and after, so the same sequence runs on every
boot. Each check prints its own line and a summary closes the section:

```
Validating NVS operations...
  [PASS] uint8 get_or_set_var on a missing key returns the default
  [PASS] uint8 default was stored
  [PASS] int32 set_var / get_var round trip
  [PASS] bool set_var / get_var round trip
  [PASS] float set_var / get_var round trip
  [PASS] string get_or_set_var on a missing key returns the default
  [PASS] string get_or_set_var on an existing key returns the stored value
  [PASS] string set_var / get_var round trip
  [PASS] shorter string overwrite reads back exactly
  [PASS] empty string set_var / get_var round trip
  [PASS] 64 character string round trip
  [PASS] string c_str() has no embedded terminator
  [PASS] blob set_var / get_var round trip (embedded zeros kept)
  [PASS] shorter blob overwrite reads back exactly
  [PASS] reading a missing blob key fails and leaves the value alone
  [PASS] 1 KB blob round trip
  [PASS] blob get_or_set_var on a missing key returns the default
  [PASS] blob default was stored
  [PASS] erase blob key succeeds
  [PASS] erased blob key is not found
  [PASS] erase key succeeds
  [PASS] erased key is not found
  [PASS] other keys survive erasing one key
  [PASS] reading a missing string key fails and leaves the value alone
  [PASS] erase namespace succeeds
  [PASS] keys are not found after erasing the namespace
NVS validation: 26/26 checks passed - PASS
```

Any failing check is listed again after the summary line. The string checks
assert that `size()` is the stored string's length (no zero terminator in the
value, see [#854](https://github.com/esp-cpp/espp/issues/854)).
