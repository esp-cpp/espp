#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

/// Shared policy for feeding an I2S TX channel from a FreeRTOS stream buffer.
///
/// Every audio BSP has the same shape: producers push PCM into a stream
/// buffer with play_audio(), and a playback task moves it, one block at a
/// time, into the I2S channel with i2s_channel_write(). The rules below are
/// what keep that from popping; they are the same on every board, so they
/// live here once and each BSP keeps only what is specific to it (write
/// timeouts, a reconfig mutex, software volume scaling, a stop-request check).
///
/// Why the DMA ring must stay full
/// -------------------------------
/// The I2S driver fills DMA descriptors progressively and the DMA transmits
/// them in ring order, so the only safe steady state is a full ring with the
/// write position a whole ring ahead of the DMA. The playback task therefore
/// writes a whole block EVERY iteration - audio when there is some, silence
/// when idle - and the write, blocking on the ring, paces the loop. Letting
/// the task sit on the stream instead (waiting for data, writing whatever
/// arrived) drains the ring while idle; the next sound then starts in
/// descriptors the DMA is about to send, half stale and half new - a pop at
/// the start of every sound, and more whenever a partial write is left behind.
/// Deepening the ring alone does not help: an idle ring is empty at any depth.
///
/// Why there is still a bounded wait
/// ---------------------------------
/// Producers push audio in bursts (a video frame at a time, with jitter), so a
/// stream that is flowing can momentarily hold less than a block. Padding the
/// gap with silence is audible as stutter, so the task gives the producer a
/// bounded chance to complete the block first. The wait is event-driven
/// (play_audio() notifies the task when the stream reaches a block) and
/// shorter than the ring minus one block, so the ring never drains while
/// waiting; a stream that was already idle gets silence at once.
///
/// Why the ring is 12 x 240 frames
/// -------------------------------
/// A descriptor the task has only partly filled is safe only while the DMA is
/// a full ring behind it, so the ring has to outlast the longest pause between
/// writes (WHOLE_BLOCK_WAIT_MS) plus the block written after it: 12 x 240
/// frames is 60 ms at 48 kHz / 65 ms at 44.1 kHz, against a 24 ms wait and a
/// 17 ms block; the driver's default of 6 (about 30 ms) leaves no margin.
/// ring_outlasts_wait() states the relation for a static_assert in the BSP.
namespace espp::i2s_tx_feed {

/// How long the playback task gives a producer that is mid-stream to complete
/// a block before the block is written padded with silence. Longer than one
/// block at every rate the boards support (a block is 1/60 s), so a producer
/// whose next frame simply has not arrived yet is not padded; shorter than the
/// DMA ring minus one block, so the ring never drains while waiting.
constexpr uint32_t WHOLE_BLOCK_WAIT_MS = 24;

/// Frames per DMA descriptor (the I2S driver's default).
constexpr uint32_t DMA_FRAME_NUM = 240;

/// Descriptors in the TX DMA ring (see "Why the ring is 12 x 240 frames").
constexpr uint32_t DMA_DESC_NUM = 12;

/// Whether a DMA ring of `desc_num` descriptors of `frame_num` frames outlasts
/// the whole-block wait plus one block (sample_rate_hz / update_frequency_hz
/// frames), at `sample_rate_hz`. Check it at the highest rate the board
/// supports, where the ring is shortest in time.
constexpr bool ring_outlasts_wait(uint32_t desc_num, uint32_t frame_num, uint32_t sample_rate_hz,
                                  uint32_t update_frequency_hz,
                                  uint32_t wait_ms = WHOLE_BLOCK_WAIT_MS) {
  return desc_num * frame_num * 1000 / sample_rate_hz > wait_ms + 1000 / update_frequency_hz;
}

/// Consumer-side state for one TX stream: one per BSP instance, as a member.
/// The BSP owns the stream buffer and the I2S channel; this only decides what
/// goes into each block.
struct Feeder {
  /// The stream producers push into (set by the BSP once it is created).
  StreamBufferHandle_t stream{nullptr};
  /// Bytes per PCM frame (channels x bytes per channel); producers queue whole
  /// frames only.
  size_t frame_bytes{4};
  /// The playback task's handle, so a producer can wake its bounded wait.
  std::atomic<TaskHandle_t> task{nullptr};
  /// Whether the previous block carried audio (only the playback task touches
  /// this): a stream that momentarily holds less than a block while flowing
  /// is given time to complete it; an idle one gets silence at once.
  bool was_flowing{false};
  /// Diagnostic: how many bytes the stream held the last time fill_block()
  /// found it misaligned (before the reset), so the BSP's warning can say what
  /// the producer left behind (`misaligned_bytes % frame_bytes` is the partial
  /// frame).
  size_t misaligned_bytes{0};

  /// Called by the playback task (its first iteration suffices): records the
  /// calling task as the one to notify.
  void attach_current_task() {
    if (task.load(std::memory_order_relaxed) == nullptr) {
      task.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    }
  }

  /// Called before the playback task is stopped or destroyed, so a later
  /// play_audio() cannot notify a task that no longer exists.
  void detach_task() { task.store(nullptr, std::memory_order_release); }

  /// Called by a producer after a successful send (send -> notify -> return):
  /// wakes the playback task if the stream now holds a whole block. A
  /// notification nobody waits for is harmless (the task re-reads the stream
  /// after every wake).
  void notify_if_block_ready(size_t block) const {
    if (stream != nullptr && xStreamBufferBytesAvailable(stream) >= block) {
      if (TaskHandle_t handle = task.load(std::memory_order_acquire)) {
        xTaskNotifyGive(handle);
      }
    }
  }

  /// Fill `buf` with the next block for the I2S channel: a bounded,
  /// notification-driven wait while the stream is flowing and holds less than
  /// `block` bytes, then up to `block` bytes of audio from the stream and
  /// silence for the rest (only the tail is zeroed). Returns the number of
  /// audio bytes in `buf`. Sets `misaligned` (and resets the stream, since a
  /// stuck partial frame would misalign every frame after it for good) when
  /// the stream held a partial frame - producers never queue one, so the
  /// caller should warn (misaligned_bytes says what it held).
  size_t fill_block(uint8_t *buf, size_t block, bool &misaligned) {
    misaligned = false;
    size_t available = stream != nullptr ? xStreamBufferBytesAvailable(stream) : 0;
    if (available % frame_bytes != 0) {
      misaligned = true;
      misaligned_bytes = available;
      xStreamBufferReset(stream);
      available = 0;
    }
    if (available < block && (available > 0 || was_flowing)) {
      TimeOut_t timeout;
      vTaskSetTimeOutState(&timeout);
      TickType_t remaining = pdMS_TO_TICKS(WHOLE_BLOCK_WAIT_MS);
      // xTaskCheckForTimeOut() keeps `remaining` current across wakes, so a
      // notification left over from an earlier send (which wakes this at
      // once) just re-reads the stream and resumes the same bounded wait
      while (available < block && xTaskCheckForTimeOut(&timeout, &remaining) == pdFALSE) {
        ulTaskNotifyTake(pdTRUE, remaining);
        available = xStreamBufferBytesAvailable(stream);
      }
    }
    available = std::min(available, block);
    if (available > 0) {
      // what was actually received: a producer-side reset (clear_audio(), a
      // rate change) between the measurement and here leaves less, and the
      // rest of buf still holds the previous block. (The stream holds whole
      // frames only, and this asks for whole frames, so the count is whole
      // frames too.)
      available = xStreamBufferReceive(stream, buf, available, 0);
    }
    if (available < block) {
      std::memset(buf + available, 0, block - available);
    }
    was_flowing = available > 0;
    return available;
  }
};

} // namespace espp::i2s_tx_feed
