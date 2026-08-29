#include <algorithm>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <driver/gpio.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "linux/videodev2.h"

#include "esp_video_device.h"
#include "esp_video_init.h"

#include "esp32-p4-module-dev-kit.hpp"

using namespace espp;

////////////////////////
//  Camera Functions  //
////////////////////////

bool Esp32P4ModuleDevKit::initialize_camera(const camera_frame_callback_t &callback,
                                            const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing camera (MIPI-CSI, OV5647)");
  if (camera_initialized_) {
    // Idempotent, matching the other initialize_* methods: keep the running
    // pipeline (and its existing callback) and report success. Call
    // stop_camera() first to re-initialize with a different callback/config.
    logger_.warn("Camera already initialized, not initializing again!");
    return true;
  }
  if (!callback) {
    logger_.error("A callback is required to receive camera frames");
    return false;
  }
  // Reap a task left over from an in-callback stop_camera() (deferred stop:
  // the task tore the pipeline down and exited on its own, but could not join
  // itself). stop() on the exited task just joins it.
  if (camera_task_) {
    camera_task_->stop();
    camera_task_.reset();
  }
  camera_stop_requested_ = false;
  camera_callback_ = callback;

  // Bring up the CSI receiver + ISP + sensor. The OV5647's SCCB shares the
  // internal I2C bus (SDA=7/SCL=8), so hand esp_video that existing bus handle
  // (via internal_i2c_.native_bus_handle()) rather than letting it create a
  // second master on the same pins.
  //
  // reset/pwdn are not routed on this board (RPi-style CSI connector); the
  // sensor free-runs (esp_video handles CSI/ISP/LDO) - hardware-verify. Unlike
  // the M5Stack Tab5 there is no IO expander here to pulse the camera reset, so
  // no expander-reset step is performed; the sensor comes up on power.
  esp_video_init_csi_config_t csi_config = {};
  csi_config.sccb_config.init_sccb = false;
  csi_config.sccb_config.i2c_handle = internal_i2c_.native_bus_handle();
  // Run the sensor SCCB at 100 kHz, matching Waveshare's own camera demos for
  // these boards (their Kconfig floor is 100 kHz). Some OV sensors are
  // unreliable at higher SCCB rates during probe, and esp_video applies this
  // freq to the SCCB device even when reusing an external I2C bus handle.
  csi_config.sccb_config.freq = 100000;
  csi_config.reset_pin = GPIO_NUM_NC;
  csi_config.pwdn_pin = GPIO_NUM_NC;
  csi_config.dont_init_ldo = false;

  esp_video_init_config_t video_config = {};
  video_config.csi = &csi_config;

  esp_err_t err = esp_video_init(&video_config);
  if (err != ESP_OK) {
    logger_.error("esp_video_init failed: {}", esp_err_to_name(err));
    camera_callback_ = nullptr;
    return false;
  }
  camera_video_inited_ = true;

  // Open the MIPI-CSI capture device. Non-blocking so the capture task can
  // observe a stop request even when no frame is ready. From here on, failures
  // go through stop_camera() so the esp_video pipeline is torn down (otherwise a
  // later initialize_camera() retry would fail).
  camera_fd_ = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR | O_NONBLOCK);
  if (camera_fd_ < 0) {
    logger_.error("Could not open camera device {} (errno {}: {})", ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
                  errno, strerror(errno));
    stop_camera();
    return false;
  }

  struct v4l2_capability capability = {};
  if (ioctl(camera_fd_, VIDIOC_QUERYCAP, &capability) != 0) {
    logger_.error("VIDIOC_QUERYCAP failed (errno {}: {})", errno, strerror(errno));
    stop_camera();
    return false;
  }

  // Choose a capture resolution: enumerate the RGB565 discrete frame sizes and
  // pick the largest that fits <= 1280 wide, falling back to 1280x720. S_FMT
  // below adjusts to the sensor's actual size and we read the real values back,
  // so this is only a hint.
  uint32_t want_w = 1280, want_h = 720;
  {
    struct v4l2_frmsizeenum frmsize = {};
    frmsize.pixel_format = V4L2_PIX_FMT_RGB565;
    uint32_t best_w = 0, best_h = 0;
    for (frmsize.index = 0; ioctl(camera_fd_, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0;
         ++frmsize.index) {
      if (frmsize.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
        break;
      }
      uint32_t w = frmsize.discrete.width;
      uint32_t h = frmsize.discrete.height;
      if (w <= 1280 && (w > best_w || (w == best_w && h > best_h))) {
        best_w = w;
        best_h = h;
      }
    }
    if (best_w > 0) {
      want_w = best_w;
      want_h = best_h;
    }
  }

  struct v4l2_format format = {};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = want_w;
  format.fmt.pix.height = want_h;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  if (ioctl(camera_fd_, VIDIOC_S_FMT, &format) != 0) {
    logger_.error("VIDIOC_S_FMT (RGB565 {}x{}) failed (errno {}: {})", want_w, want_h, errno,
                  strerror(errno));
    stop_camera();
    return false;
  }
  camera_width_ = static_cast<uint16_t>(format.fmt.pix.width);
  camera_height_ = static_cast<uint16_t>(format.fmt.pix.height);
  logger_.info("Camera format: {}x{} RGB565", camera_width_.load(), camera_height_.load());

  // Request and memory-map the capture buffers. REQBUFS may hand back fewer
  // buffers than requested; use the count it actually allocated (and require at
  // least one, capped at our array size).
  struct v4l2_requestbuffers req = {};
  req.count = CAMERA_BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(camera_fd_, VIDIOC_REQBUFS, &req) != 0) {
    logger_.error("VIDIOC_REQBUFS failed (errno {}: {})", errno, strerror(errno));
    stop_camera();
    return false;
  }
  if (req.count < 1) {
    logger_.error("VIDIOC_REQBUFS allocated 0 buffers");
    stop_camera();
    return false;
  }
  camera_buffer_count_ = std::min<uint32_t>(req.count, CAMERA_BUFFER_COUNT);
  if (req.count < CAMERA_BUFFER_COUNT) {
    logger_.warn("VIDIOC_REQBUFS allocated {} of {} requested buffers", req.count,
                 CAMERA_BUFFER_COUNT);
  }
  for (int i = 0; i < camera_buffer_count_; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(camera_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      logger_.error("VIDIOC_QUERYBUF {} failed (errno {}: {})", i, errno, strerror(errno));
      stop_camera();
      return false;
    }
    camera_buffer_sizes_[i] = buf.length;
    camera_buffers_[i] =
        mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, camera_fd_, buf.m.offset);
    if (camera_buffers_[i] == MAP_FAILED) {
      camera_buffers_[i] = nullptr;
      logger_.error("mmap of camera buffer {} failed (errno {}: {})", i, errno, strerror(errno));
      stop_camera();
      return false;
    }
    if (ioctl(camera_fd_, VIDIOC_QBUF, &buf) != 0) {
      logger_.error("VIDIOC_QBUF {} failed (errno {}: {})", i, errno, strerror(errno));
      stop_camera();
      return false;
    }
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(camera_fd_, VIDIOC_STREAMON, &type) != 0) {
    logger_.error("VIDIOC_STREAMON failed (errno {}: {})", errno, strerror(errno));
    stop_camera();
    return false;
  }

  using namespace std::placeholders;
  camera_task_ = espp::Task::make_unique({
      .callback = std::bind(&Esp32P4ModuleDevKit::camera_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });
  // Publish the initialized state BEFORE starting the task: Task::start()
  // launches the worker immediately, and an immediate fatal capture error in
  // camera_task_callback() runs teardown_camera_pipeline() (which clears this
  // flag) — an assignment after start() could overwrite that clear and report
  // a released pipeline as initialized.
  camera_initialized_ = true;
  if (!camera_task_->start()) {
    logger_.error("Could not start the camera task");
    stop_camera(); // also clears camera_initialized_
    return false;
  }
  return true;
}

bool Esp32P4ModuleDevKit::camera_task_callback(std::mutex &m, std::condition_variable &cv,
                                               bool &task_notified) {
  (void)cv; // unused: this task polls the non-blocking fd instead of waiting
  // Dequeue a filled frame, hand it to the callback (valid only for the call),
  // then requeue the buffer for reuse. The fd is non-blocking, so if no frame
  // is ready yet the DQBUF fails and we wait briefly - this keeps the task
  // returning regularly so a stop request is observed promptly.
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(camera_fd_, VIDIOC_DQBUF, &buf) == 0) {
    if (camera_callback_ && buf.index < CAMERA_BUFFER_COUNT && camera_buffers_[buf.index]) {
      // Prefer the driver-reported payload size; fall back to the computed
      // RGB565 size only if the driver does not report bytesused.
      const uint16_t width = camera_width_;
      const uint16_t height = camera_height_;
      const size_t len = buf.bytesused ? static_cast<size_t>(buf.bytesused)
                                       : static_cast<size_t>(width) * height * 2;
      camera_callback_(static_cast<const uint8_t *>(camera_buffers_[buf.index]), width, height,
                       len);
    }
    // Honor a stop_camera() issued from within the frame callback: it could
    // not join this task from its own context, so it deferred by setting this
    // flag. Tear the pipeline down and exit the task (skipping the requeue -
    // the pipeline is going away). The Task object itself is joined/destroyed
    // by the next stop_camera()/initialize_camera() from another context.
    if (camera_stop_requested_) {
      camera_stop_requested_ = false;
      teardown_camera_pipeline();
      return true; // stop the task
    }
    // Requeue the buffer. If this fails the capture queue drains and the stream
    // stalls, so treat it as fatal to the task rather than spinning silently.
    if (ioctl(camera_fd_, VIDIOC_QBUF, &buf) != 0) {
      logger_.error("VIDIOC_QBUF failed (errno {}); stopping the camera", errno);
      // Tear down the pipeline here (from the camera task itself) so the driver
      // is not left wedged (STREAMON + mmaps + esp_video init active) until
      // someone calls stop_camera(); this also lets initialize_camera() be
      // called again to recover. Only the pipeline is torn down - calling
      // stop_camera()/Task::stop() here would self-join the task, so the task
      // exits via `return true` instead. A later stop_camera() is still safe:
      // the teardown is idempotent and stop() on an exited task just joins it.
      teardown_camera_pipeline();
      return true; // stop the task; the owner can re-init via initialize_camera()
    }
  } else if (errno == EAGAIN) {
    // No frame ready yet on the non-blocking fd; wait briefly and retry.
    vTaskDelay(pdMS_TO_TICKS(5));
  } else {
    // A real capture error (device/stream/driver): surface it and stop the task
    // instead of spinning forever with no diagnostics, tearing down the
    // pipeline (see above) so the device is released for a later re-init.
    logger_.error("VIDIOC_DQBUF failed (errno {}); stopping the camera", errno);
    teardown_camera_pipeline();
    return true;
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
}

void Esp32P4ModuleDevKit::stop_camera() {
  // A stop from within the camera task itself (the frame callback runs in that
  // context) must not stop()/reset() the task here: Task::stop() deliberately
  // skips joining the current thread, so camera_task_.reset() would destroy a
  // still-joinable std::thread and terminate the program. Defer instead:
  // camera_task_callback() observes the flag right after the frame callback
  // returns, tears the pipeline down and exits the task. The exited (but not
  // yet joined) Task object is reaped by the next stop_camera() or
  // initialize_camera() call from another context.
  if (camera_task_ && espp::Task::get_current_id() == camera_task_->get_id()) {
    camera_stop_requested_ = true;
    return;
  }
  // Stop the task first so nothing is using the fd/buffers during teardown. If
  // the task already exited on its own (fatal capture error or a deferred
  // in-callback stop, both of which tear the pipeline down themselves), stop()
  // just joins the exited task and the teardown below is a no-op.
  if (camera_task_) {
    camera_task_->stop();
    camera_task_.reset();
  }
  teardown_camera_pipeline();
}

void Esp32P4ModuleDevKit::teardown_camera_pipeline() {
  if (camera_fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(camera_fd_, VIDIOC_STREAMOFF, &type);
  }
  for (int i = 0; i < CAMERA_BUFFER_COUNT; ++i) {
    if (camera_buffers_[i]) {
      munmap(camera_buffers_[i], camera_buffer_sizes_[i]);
      camera_buffers_[i] = nullptr;
      camera_buffer_sizes_[i] = 0;
    }
  }
  if (camera_fd_ >= 0) {
    close(camera_fd_);
    camera_fd_ = -1;
  }
  if (camera_video_inited_) {
    esp_video_deinit();
    camera_video_inited_ = false;
  }
  // Reset the reported dimensions so camera_width()/height() honor their
  // documented "0 when not initialized" contract after a stop or a failure.
  camera_buffer_count_ = 0;
  camera_width_ = 0;
  camera_height_ = 0;
  camera_initialized_ = false;
  camera_callback_ = nullptr;
}

uint16_t Esp32P4ModuleDevKit::camera_width() const { return camera_width_; }

uint16_t Esp32P4ModuleDevKit::camera_height() const { return camera_height_; }
