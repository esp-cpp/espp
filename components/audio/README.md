# Audio Component

[![Badge](https://components.espressif.com/components/espp/audio/badge.svg)](https://components.espressif.com/components/espp/audio)

The `audio` component provides `espp::Audio`, I2S audio playback and recording
for one I2S port in standard (Philips) mode with 16-bit PCM: a TX channel for
playback, an RX channel for recording, or both, full duplex on the same clocks.

It owns the machinery the espp BSPs used to copy into each board:

- the I2S channels and their DMA rings, sized so the TX ring outlasts the
  playback task's bounded wait (see `codec/include/i2s_tx_feed.hpp`),
- the playback stream buffer and task, which always writes whole blocks so the
  TX ring never runs dry (an idle ring pops at the start of the next sound),
- the recording task, which drains the RX ring continuously and hands the
  samples to a callback while recording,
- volume, mute and microphone gain: in the codec, or volume and mute in
  software when there is no codec (e.g. a MAX98357A amplifier),
- sample rate changes (the I2S clocks and the codecs), and
- a complete unwind when `initialize()` fails, `deinitialize()` and the
  destructor.

## Codecs

Chip-specific setup goes through the small `espp::AudioCodec` interface
(`start()` / `stop()` / `set_sample_rate()` / `set_volume()` / `set_mute()` /
`set_gain()`). `espp::Audio` calls it only from initialization, teardown and
its setters, never per sample. This component provides adapters on the
drivers in the `codec` component:

- `espp::Es8388Codec`: the ES8388 DAC, for playback (M5Stack Tab5).
- `espp::Es7210Codec`: the ES7210 microphone ADC, for recording (M5Stack Tab5).

Each adapter takes the register read / write functions, e.g.
`espp::make_i2c_addressed_write(device)` for an `espp::I2c` device. A chip that
handles both directions is passed as both `Config::output_codec` and
`Config::input_codec`.

## Board-specific parts

Amplifier enable pins, IO expanders and buttons stay in the BSP.
`Config::on_playback_block` is called once per playback block with whether it
carried audio, so a BSP can follow playback activity (the M5Stack Tab5 pauses
its battery charger with it). A board whose microphones are on a separate I2S
port uses two instances, one for playback and one for recording.

## BSP audio API

The BSPs that use this component expose the same audio methods, which forward
to it:

| Method | |
|---|---|
| `initialize_audio()` / `initialize_audio(const AudioConfig &)` | set up playback and / or recording |
| `deinitialize_audio()`, `is_audio_initialized()` | tear down / query |
| `audio()` | the `espp::Audio` instance |
| `enable_audio(bool)` | the speaker amplifier |
| `play_audio(std::span<const uint8_t>)` / `play_audio(const uint8_t *, size_t)` | queue audio, returns the bytes queued |
| `clear_audio()` | drop the queued audio |
| `volume()`, `mute()` / `is_muted()` | output level |
| `audio_sample_rate()`, `audio_buffer_size()` | rate (get / set), block size |
| `start_audio_recording(callback)` / `stop_audio_recording()` | recording |
| `microphone_volume()` | microphone gain |

Older names are kept as `[[deprecated]]` methods that forward to these.

## Example

The [example](./example) plays a tone through an I2S amplifier with no codec
(software volume and mute).
