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

#include "esp32-p4-nano.hpp"

using namespace espp;

////////////////////////
//  Camera Functions  //
////////////////////////

bool Esp32P4Nano::initialize_camera(const camera_frame_callback_t &callback,
                                    const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing camera (MIPI-CSI, OV5647)");
  if (camera_initialized_) {
    logger_.warn("Camera already initialized");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive camera frames");
    return false;
  }
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
  // Run the sensor SCCB at 400 kHz rather than 100 kHz. The SCCB shares the
  // internal I2C bus with the touch controller and audio codec, and the ISP's
  // auto-exposure writes the sensor over SCCB every frame; at 100 kHz each of
  // those writes holds the shared bus ~4x longer than needed. 400 kHz is a safe
  // SCCB speed for this sensor and matches the bus's configured clock.
  csi_config.sccb_config.freq = 400000;
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
  logger_.info("Camera format: {}x{} RGB565", camera_width_, camera_height_);

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
      .callback = std::bind(&Esp32P4Nano::camera_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });
  if (!camera_task_->start()) {
    logger_.error("Could not start the camera task");
    stop_camera();
    return false;
  }
  camera_initialized_ = true;
  return true;
}

bool Esp32P4Nano::camera_task_callback(std::mutex &m, std::condition_variable &cv,
                                       bool &task_notified) {
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
      const size_t len = buf.bytesused ? static_cast<size_t>(buf.bytesused)
                                       : static_cast<size_t>(camera_width_) * camera_height_ * 2;
      camera_callback_(static_cast<const uint8_t *>(camera_buffers_[buf.index]), camera_width_,
                       camera_height_, len);
    }
    // Requeue the buffer. If this fails the capture queue drains and the stream
    // stalls, so treat it as fatal to the task rather than spinning silently.
    if (ioctl(camera_fd_, VIDIOC_QBUF, &buf) != 0) {
      logger_.error("VIDIOC_QBUF failed (errno {}); stopping the camera task", errno);
      return true; // stop the task; the owner can stop_camera() / re-init
    }
  } else if (errno == EAGAIN) {
    // No frame ready yet on the non-blocking fd; wait briefly and retry.
    vTaskDelay(pdMS_TO_TICKS(5));
  } else {
    // A real capture error (device/stream/driver): surface it and stop the task
    // instead of spinning forever with no diagnostics.
    logger_.error("VIDIOC_DQBUF failed (errno {}); stopping the camera task", errno);
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

void Esp32P4Nano::stop_camera() {
  if (camera_task_) {
    camera_task_->stop();
    camera_task_.reset();
  }
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

uint16_t Esp32P4Nano::camera_width() const { return camera_width_; }

uint16_t Esp32P4Nano::camera_height() const { return camera_height_; }
