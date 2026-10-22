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
a type list, newest first; the first type is the current one. Every file carries
a 20-byte header (magic, schema version, layout fingerprint, payload length,
CRC32) in front of the payload, which is serialized with the
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
| same version number, different layout | `layout_mismatch` (header fingerprint)     |

Saves are atomic: the data is written to `<file>.tmp`, flushed and synced,
then renamed over the file, so a reset in the middle of a save keeps the
previous contents.

## Defining versions

Each version is an aggregate (no user-declared constructors; that is what alpaca
serializes) with a `static constexpr uint32_t VERSION`, strictly decreasing
along the list (newest first). Every version except the last provides `static T
migrate(const Older &)` from the version after it in the list; an older file is
walked toward the front of the list one step at a time, so each version only
knows how to come from the one before it. The structs themselves are declared
oldest first (C++ needs the older type complete before `migrate` can mention
it); only the type list is newest first.

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

using PersistentSettings = espp::PersistentData<Settings, SettingsV2, SettingsV1>;
```

A new version is prepended to the list. Dropping support for the oldest
version is removing the last entry; such files then load with
`unknown_version` and the application decides what to do.
A missing `VERSION`, a non-decreasing version, or a missing `migrate` is a
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

// get() / set() are also available as conversion / assignment operators;
// saving stays explicit because it can fail
Settings t = settings; // == settings.get()
t.boot_count = 0;
settings = t;          // == settings.set(t)
settings.save(ec);
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
| 8      | 4    | layout fingerprint of the type that wrote it (below)       |
| 12     | 4    | payload length                                             |
| 16     | 4    | CRC32 over the four fields above and the payload           |
| 20     | n    | payload: alpaca, `fixed_length_encoding \| strict`         |

The format is the same on every target and on the host: alpaca encodes
container lengths as 32-bit values, and nothing in the header depends on the
platform, so a PC tool can read and write a device's file.

### Layout fingerprint

The fingerprint is a CRC32 over a byte sequence describing the type exactly as
alpaca serializes it, built from alpaca's `field_type` codes
(`components/serialization/detail/alpaca/include/alpaca/detail/field_type.h`),
recursively, one entry per type; multi-byte counts are little-endian:

| type                        | bytes                                   |
|-----------------------------|-----------------------------------------|
| scalar (`bool` ... `double`) | code                                   |
| enum                        | `enum_class`, underlying integer        |
| `std::string`               | `string`                                |
| `std::vector<T>`            | `vector`, T                             |
| `std::array<T, N>`          | `array`, u32 N, T                       |
| `std::list` / `deque<T>`    | `list` / `deque`, T                     |
| `std::map<K, V>`            | `map`, K, V (`unordered_map` likewise)  |
| `std::set<T>`               | `set`, T (`unordered_set` likewise)     |
| `std::optional<T>`          | `optional`, T                           |
| `std::pair<A, B>`           | `pair`, A, B                            |
| `std::tuple<Ts...>`         | `tuple`, u16 count, Ts...               |
| `std::variant<Ts...>`       | `variant`, u16 count, Ts...             |
| aggregate                   | `struct_`, u16 field count, fields...   |

A version number reused for a different layout therefore loads with
`layout_mismatch` instead of producing garbage, and a reader without the C++
type (a Python tool) can compute the same fingerprint from a field spec.

alpaca's own `with_version` option is deliberately not used: its hash folds
`sizeof(struct)` in, which differs between a 32-bit ESP32 and a 64-bit host
(a struct of a string, a vector and an integer is 40 bytes on the ESP32, 56 on
macOS and 64 on Linux), so a file written on one would not load on the other.

## Errors

Every operation takes a `std::error_code &` and returns `bool`. The codes are
`espp::PersistentDataErrc` (`file_not_found`, `io_error`, `file_too_large`,
`bad_header`, `bad_checksum`, `unknown_version`, `layout_mismatch`,
`deserialize_failed`, `serialize_failed`) and compare directly: `ec == espp::PersistentDataErrc::file_not_found`.

## Other backends

`PersistentData::encode()` and `decode()` produce and consume the same bytes
as the file (header included, migrations applied on decode) without touching
the filesystem, for storing the structure as an NVS blob or sending it over a
transport.

## Thread safety

All methods lock an internal mutex; `get()` returns a copy, and `update()`
runs the caller's function and the save under the same lock. A save blocks
other callers for the duration of the flash write.

## Host (cross-platform library)

The component is part of the cross-platform C++ library in `lib/` (included
by `espp.hpp`), so the same class and file format work on a PC against the
installed `espp::espp` target, for desktop tools that read or write a device's
settings file, or for host-side tests of migrations. It is not in the Python
bindings: `PersistentData` is a class template over the application's own
types, which the generated bindings cannot instantiate generically.

## Host test

The file format, version list, migration chain and file helpers live in
`include/detail/persistent_data_codec.hpp` (standard library + alpaca only) and
are covered by `test/persistent_data_host_test.cpp`, which builds and runs on
the host; the build line is in the file's header comment.

## Example

The [example](./example) stores a settings struct on a LittleFS partition,
migrates a version-1 file written by "old firmware" and validates every
operation with a pass / fail summary.
