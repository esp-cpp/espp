Audio
*****

The :cpp:class:`espp::Audio` component drives one I2S port with 16-bit PCM: a
TX channel for playback, an RX channel for recording, or both, full duplex, in
standard (Philips) mode. An instance that only records can instead receive TDM
(e.g. an ES7210 with four microphones) or a PDM microphone, and
``HardwareConfig::recording_slots`` picks the slots its callback receives. It owns the
machinery the espp BSPs used to copy into each board:

- the I2S channels and their DMA rings, sized so the TX ring outlasts the
  playback task's bounded wait,
- the playback stream buffer and task, which always writes whole blocks so the
  TX ring never runs dry (no pops between sounds),
- the recording task, which drains the RX ring continuously and hands the
  samples to a callback while recording,
- volume, mute and microphone gain (in the codec, or volume and mute in
  software when there is no codec),
- sample rate changes, and
- a complete unwind when initialization fails, ``deinitialize()`` and the
  destructor.

Chip-specific setup goes through the :cpp:class:`espp::AudioCodec` interface.
This component provides adapters for the ES8311 (:cpp:class:`espp::Es8311Codec`,
playback and recording), the ES8388 (:cpp:class:`espp::Es8388Codec`, playback)
and the ES7210 (:cpp:class:`espp::Es7210Codec`, recording), on the drivers in
the ``codec`` component. A chip that handles both directions, like the ES8311,
is passed as both the output and the input codec.

:cpp:struct:`espp::Audio::Config` has two parts: an
:cpp:struct:`espp::Audio::AppConfig` (what an application chooses: the sample
rate, which directions run, the playback buffer and the tasks) and an
:cpp:struct:`espp::Audio::HardwareConfig` (how the board is wired: the I2S port
and pins, channels, the recording format and slots, the MCLK multiple and the
codecs). The BSPs take only the ``AppConfig``, as their ``AudioConfig``, and
fill in the ``HardwareConfig`` for their board.

Board-specific parts - amplifier enable pins, IO expanders, buttons - stay in
the BSP. ``Config::on_playback_block`` lets a BSP follow playback activity (the
M5Stack Tab5 pauses its battery charger with it, since the charger pops the
speaker). A board whose microphones are on a separate I2S port uses two
instances: one for playback and one for recording, which such a BSP's
``microphone_audio()`` returns.

BSP audio API
-------------

The BSPs that use this component expose the same audio methods, which forward
to it: ``initialize_audio()`` / ``initialize_audio(const AudioConfig &)``
(``AudioConfig`` is ``espp::Audio::AppConfig``; ``default_audio_config()``
returns the board's defaults to start one from),
``deinitialize_audio()``, ``is_audio_initialized()``, ``audio()``,
``enable_audio(bool)`` (the amplifier), ``play_audio(std::span<const uint8_t>)``
/ ``play_audio(const uint8_t *, size_t)``, ``clear_audio()``, ``volume()``,
``mute()`` / ``is_muted()``, ``audio_sample_rate()``, ``audio_buffer_size()``,
``start_audio_recording(callback)`` / ``stop_audio_recording()`` and
``microphone_volume()``. Older names are kept as ``[[deprecated]]`` methods
that forward to these.

.. ------------------------------- Example -------------------------------------

.. toctree::

   audio_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/audio.inc
.. include-build-file:: inc/audio_codec.inc
.. include-build-file:: inc/es8311_codec.inc
.. include-build-file:: inc/es8388_codec.inc
.. include-build-file:: inc/es7210_codec.inc
