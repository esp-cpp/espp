# Persistent Data Example

This example shows how the `persistent_data` component stores a settings
structure in a file on a [littlefs](https://github.com/littlefs-project/littlefs)
partition, how the structure evolves over firmware versions (a version-1 file
written by old firmware is migrated v1 -> v2 -> v3 and rewritten), and how
corrupt, truncated and newer-firmware files are reported instead of guessed at.

The example ends with a validation section that exercises every operation
(create, auto-load, `update()`, migration, newer-version refusal, corruption
detection, `encode()` / `decode()`) and prints a pass / fail summary.

## How to use example

### Hardware Required

Any ESP32 with a `littlefs` partition (see `partitions.csv`); the partition
label is configured through `menuconfig` (`CONFIG_ESPP_FILE_SYSTEM_PARTITION_LABEL`).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Expected Output

```
I (…) persistent data example: Starting persistent data example!
I (…) persistent data example: Day-to-day use: load or create, then update
I (…) PersistentData:settings.bin: No file at '/littlefs/settings.bin'
I (…) PersistentData:settings.bin: No file at '/littlefs/settings.bin', creating it with defaults
I (…) persistent data example: Loaded (file version 0): Settings{name: 'espp', brightness: 0.50, color: 0x00ff00, favorites: [], boot_count: 0}
I (…) persistent data example: After update: Settings{name: 'espp', brightness: 1.00, color: 0x00ff00, favorites: ["red", "green"], boot_count: 1}
I (…) persistent data example: Validating persistent data operations...
  [PASS] erase succeeds
  [PASS] file does not exist after erase
  [PASS] load of a missing file reports file_not_found
  [PASS] is_loaded() is false before any load or save
  [PASS] load_or_create creates the file
  [PASS] file exists after create
  [PASS] is_loaded() is true after create
  [PASS] auto_load loads the created file
  [PASS] loaded data matches
  [PASS] loaded_version() is the current version
  [PASS] was_migrated() is false for a current-version file
  [PASS] update() saves
  [PASS] update() result persisted
  [PASS] update() returning false does not save
  [PASS] conversion operator returns the loaded data
  [PASS] assignment operator sets the data in memory
  [PASS] assignment does not write the file
  [PASS] save() after assignment persists it
  [PASS] v1 file written
  [PASS] v1 file loads
  [PASS] v1 file was migrated
  [PASS] migration kept the name
  [PASS] migration converted brightness 80 -> 0.80
  [PASS] migration filled v2 defaults
  [PASS] migration filled v3 defaults
  [PASS] file was rewritten as the current version
  [PASS] newer file reports unknown_version
  [PASS] memory untouched after a failed load
  [PASS] load_or_create does not replace a newer file
  [PASS] read the raw file
  [PASS] write the corrupted file
  [PASS] corrupt file reports bad_checksum
  [PASS] truncated file reports bad_header
  [PASS] encode
  [PASS] decode
  [PASS] decode round trip
  [PASS] final save
Persistent data validation: 37/37 checks passed - PASS
I (…) persistent data example: Persistent data example complete!
```

On the next boot the file exists, so the first section loads it (boot count 1)
and the update bumps it to 2.
