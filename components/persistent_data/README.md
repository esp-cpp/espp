# Persistent Data Component

[![Badge](https://components.espressif.com/components/espp/persistent_data/badge.svg)](https://components.espressif.com/components/espp/persistent_data)

The `persistent_data` component stores one data structure (a settings struct,
calibration data, a paired-device list) in a file on any filesystem reachable
through the VFS (LittleFS, SPIFFS, SD card), tracks the schema version of what
is on disk, and migrates older files forward when the structure changes.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Persistent Data Component](#persistent-data-component)
  - [Overview](#overview)
  - [Defining versions](#defining-versions)
  - [Using it](#using-it)
  - [File format](#file-format)
  - [Errors](#errors)
  - [Other backends](#other-backends)
  - [Thread safety](#thread-safety)
  - [Host test](#host-test)
  - [Example](#example)

<!-- markdown-toc end -->

## Overview

`espp::PersistentData<Versions...>` takes the schema versions of a structure as
a type list, oldest first; the last type is the current one. Every file carries
a 16-byte header (magic, schema version, payload length, CRC32) in front of the
payload, which is serialized with the
[serialization](../serialization) component (alpaca). A load therefore
distinguishes, without guessing:

| Situation                             | Result                                     |
|---------------------------------------|--------------------------------------------|
| no file yet                           | `file_not_found`                           |
| file of the current version           | loaded                                     |
| file of an older listed version       | loaded, migrated, rewritten (configurable) |
| file of a version not in the list     | `unknown_version` (newer firmware wrote it, or support was dropped) |
| truncated file, wrong magic, trailing bytes | `bad_header`                         |
| bit error                             | `bad_checksum`                             |
| same version number, different layout | `deserialize_failed` (alpaca's type hash)  |

Saves are atomic: the data is written to `<file>.tmp`, flushed and synced,
then renamed over the file, so a reset in the middle of a save keeps the
previous contents.

## Defining versions

Each version is an aggregate (no user-declared constructors; that is what alpaca
serializes) with a `static constexpr uint32_t VERSION`, strictly increasing
along the list. Every version after the first provides `static T migrate(const
Previous &)`; an older file is walked up the chain one step at a time, so each
version only knows how to come from the one before it.

```cpp
struct SettingsV1 {
  static constexpr uint32_t VERSION = 1;
  uint8_t brightness{50};
  std::string device_name{"espp"};
};

struct SettingsV2 {
  static constexpr uint32_t VERSION = 2;
  uint8_t brightness{50};
  std::string device_name{"espp"};
  uint32_t color_rgb{0x00FF00};

  static SettingsV2 migrate(const SettingsV1 &v1) {
    return {.brightness = v1.brightness, .device_name = v1.device_name};
  }
};

struct Settings { // current
  static constexpr uint32_t VERSION = 3;
  std::string device_name{"espp"};
  float brightness{0.5f};
  uint32_t color_rgb{0x00FF00};
  uint32_t boot_count{0};

  static Settings migrate(const SettingsV2 &v2) {
    return {.device_name = v2.device_name,
            .brightness = v2.brightness / 100.0f,
            .color_rgb = v2.color_rgb};
  }
};

using PersistentSettings = espp::PersistentData<SettingsV1, SettingsV2, Settings>;
```

Dropping support for a very old version is removing it from the list; such
files then load with `unknown_version` and the application decides what to do.
A missing `VERSION`, a non-increasing version, or a missing `migrate` is a
compile error with a message saying which requirement failed.

## Using it

```cpp
std::error_code ec;
PersistentSettings settings({.file_path = fs.get_root_path() / "settings.bin",
                             .auto_load = false});

// load, or create the file from defaults if there is none; a corrupt file or
// one written by newer firmware is reported, not silently replaced
if (!settings.load_or_create(Settings{}, ec)) {
  logger.error("settings: {}", ec.message());
}

// read-modify-write under one lock, saved atomically
settings.update([](Settings &s) { s.boot_count++; }, ec);

// or explicitly
Settings s = settings.get(); // a copy
s.brightness = 1.0f;
settings.save(s, ec);
```

With `auto_load` (the default) the constructor loads the file and logs any
problem; `is_loaded()`, `loaded_version()` and `was_migrated()` say what
happened. `erase()` removes the file (succeeding when there is none).

## File format

All fields little-endian:

| offset | size | field                                                      |
|--------|------|------------------------------------------------------------|
| 0      | 4    | magic `"ESPD"`                                             |
| 4      | 4    | schema version (`VERSION` of the type that wrote it)       |
| 8      | 4    | payload length                                             |
| 12     | 4    | CRC32 over the three fields above and the payload          |
| 16     | n    | payload: alpaca, `fixed_length_encoding \| with_version \| strict` |

`with_version` embeds a hash of the payload type's field types, so a payload
never decodes as a type with a different layout even if a version number was
reused; `strict` rejects payloads shorter than the type.

## Errors

Every operation takes a `std::error_code &` and returns `bool`. The codes are
`espp::PersistentDataErrc` (`file_not_found`, `io_error`, `file_too_large`,
`bad_header`, `bad_checksum`, `unknown_version`, `deserialize_failed`,
`serialize_failed`) and compare directly: `ec == espp::PersistentDataErrc::file_not_found`.

## Other backends

`PersistentData::encode()` and `decode()` produce and consume the same bytes
as the file (header included, migrations applied on decode) without touching
the filesystem, for storing the structure as an NVS blob or sending it over a
transport.

## Thread safety

All methods lock an internal mutex; `get()` returns a copy, and `update()`
runs the caller's function and the save under the same lock. A save blocks
other callers for the duration of the flash write.

## Host test

The file format, version list, migration chain and file helpers live in
`include/detail/persistent_data_codec.hpp` (standard library + alpaca only) and
are covered by `test/persistent_data_host_test.cpp`, which builds and runs on
the host; the build line is in the file's header comment.

## Example

The [example](./example) stores a settings struct on a LittleFS partition,
migrates a version-1 file written by "old firmware" and validates every
operation with a pass / fail summary.
