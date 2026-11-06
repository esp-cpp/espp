# ESP-BOX Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/esp-box/badge.svg)](https://components.espressif.com/components/espp/esp-box)

The ESP32-S3-BOX and ESP32-S3-BOX-3 are development boards for the ESP32-S3
module. They feature a nice touchscreen display, an IMU, a speaker, microphones,
and expansion headers.

The `espp::EspBox` component provides a singleton hardware abstraction for
initializing the touch, display, and audio subsystems, as well as automatically
determining which version of the Box it's running on. The audio subsystem is
built on the [`espp::Audio`](../audio) component and runs full duplex on one
I2S bus: `initialize_audio()` sets up playback, recording or both (an
`AudioConfig`, i.e. `espp::Audio::AppConfig`, started from
`EspBox::default_audio_config()`), `play_audio()` streams 16-bit stereo out through the
ES8311 codec and speaker, and `start_audio_recording()` delivers 16-bit stereo
recordings from the dual microphone array (through the ES7210 ADC, with
adjustable gain via `microphone_volume()`) at the speaker's sample rate.
`audio()` returns the underlying `espp::Audio` for anything else. The older
`initialize_sound()`, `enable_sound()` and `initialize_microphone()` are kept
as deprecated forwarders.

## Example

The [example](./example) shows how to use the `espp::EspBox` hardware
abstraction component to automatically detect and initialize components on both
the ESP32-S3-BOX and the ESP32-S3-BOX-3.
