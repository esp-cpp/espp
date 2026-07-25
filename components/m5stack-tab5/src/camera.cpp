#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <driver/ppa.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#include "linux/videodev2.h"

#include "esp_video_device.h"
#include "esp_video_init.h"

#include "m5stack-tab5.hpp"

using namespace espp;

////////////////////////
//  Camera Functions  //
////////////////////////

bool M5StackTab5::camera_reset(bool assert_reset) {
  // CAM_RST is on IO expander 0x43 pin P6 and is active-low: drive the output
  // LOW to hold the sensor in reset, HIGH to release it. set_io_expander_output
  // takes the desired output level, so invert the "assert" request.
  return set_io_expander_output(0x43, IO43_BIT_CAM_RST, !assert_reset);
}

bool M5StackTab5::initialize_camera(const camera_frame_callback_t &callback,
                                    const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing camera (MIPI-CSI, SC202CS)");
  if (camera_initialized_) {
    logger_.warn("Camera already initialized");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive camera frames");
    return false;
  }
  camera_callback_ = callback;

  // Pulse the (active-low) camera reset for a clean sensor power-up, then
  // release it and give the sensor a moment before esp_video probes it over
  // SCCB. The IO expander defaults this pin HIGH, so the camera is normally
  // out of reset already; the pulse just guarantees a known start state.
  camera_reset(true);
  vTaskDelay(pdMS_TO_TICKS(10));
  camera_reset(false);
  vTaskDelay(pdMS_TO_TICKS(20));

  // Bring up the CSI receiver + ISP + sensor. The SC202CS's SCCB shares the
  // internal I2C bus (same GPIO32/31), so hand esp_video that existing bus
  // handle rather than letting it create a second master on the same pins.
  // Reset / power-down are NC here: the sensor reset is on the IO expander and
  // was handled above.
  esp_video_init_csi_config_t csi_config = {};
  csi_config.sccb_config.init_sccb = false;
  csi_config.sccb_config.i2c_handle = internal_i2c_.native_bus_handle();
  // Run the sensor SCCB at 400 kHz rather than 100 kHz. The SCCB shares the
  // internal I2C bus with the touch controller (and IMU / RTC / power monitor),
  // and the ISP's auto-exposure writes the sensor over SCCB every frame; at
  // 100 kHz each of those writes holds the shared bus ~4x longer than needed,
  // starving the (deliberately short, fail-fast) touch reads and making touch
  // feel laggy. 400 kHz is a safe SCCB speed for this sensor and cuts that
  // bus-hold time. (The bus itself runs the other devices at 1 MHz; the I2C
  // master reprograms the clock per transaction.)
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
    logger_.error("Could not open camera device {}", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
    stop_camera();
    return false;
  }

  struct v4l2_capability capability = {};
  if (ioctl(camera_fd_, VIDIOC_QUERYCAP, &capability) != 0) {
    logger_.error("VIDIOC_QUERYCAP failed");
    stop_camera();
    return false;
  }

  // Choose a capture resolution: enumerate the RGB565 discrete frame sizes and
  // pick the largest that fits the panel (<= 1280 wide, the landscape width),
  // falling back to 1280x720 (the SC202CS's HD mode). VIDIOC_S_FMT below
  // adjusts to the sensor's actual size and we read the real values back, so
  // this is only a hint.
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
    logger_.error("VIDIOC_S_FMT (RGB565 {}x{}) failed", want_w, want_h);
    stop_camera();
    return false;
  }
  camera_width_ = static_cast<uint16_t>(format.fmt.pix.width);
  camera_height_ = static_cast<uint16_t>(format.fmt.pix.height);
  logger_.info("Camera format: {}x{} RGB565", camera_width_, camera_height_);

  // Request and memory-map the capture buffers.
  struct v4l2_requestbuffers req = {};
  req.count = CAMERA_BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(camera_fd_, VIDIOC_REQBUFS, &req) != 0) {
    logger_.error("VIDIOC_REQBUFS failed");
    stop_camera();
    return false;
  }
  for (int i = 0; i < CAMERA_BUFFER_COUNT; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(camera_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      logger_.error("VIDIOC_QUERYBUF {} failed", i);
      stop_camera();
      return false;
    }
    camera_buffer_sizes_[i] = buf.length;
    camera_buffers_[i] =
        mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, camera_fd_, buf.m.offset);
    if (camera_buffers_[i] == MAP_FAILED) {
      camera_buffers_[i] = nullptr;
      logger_.error("mmap of camera buffer {} failed", i);
      stop_camera();
      return false;
    }
    if (ioctl(camera_fd_, VIDIOC_QBUF, &buf) != 0) {
      logger_.error("VIDIOC_QBUF {} failed", i);
      stop_camera();
      return false;
    }
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(camera_fd_, VIDIOC_STREAMON, &type) != 0) {
    logger_.error("VIDIOC_STREAMON failed");
    stop_camera();
    return false;
  }

  // Silence the ISP CCM-rejection spam. The stock SC202CS IPA config's auto
  // color-correction (esp_ipa ACC) periodically computes a CCM coefficient
  // beyond the ESP32-P4 ISP's +/-4.0 hardware limit under some illuminants. The
  // ISP driver safely rejects it and keeps the previous CCM - the image is
  // unaffected - but esp_video's ISP pipeline logs the rejection several times
  // per frame (ISP_CCM / ISP / isp_video / esp_video, all at ERROR level). That
  // per-frame *blocking* UART logging both floods the console and starves the
  // capture pipeline enough to drop frames. The out-of-range CCM is produced
  // inside the precompiled esp_ipa and cannot be fully bounded from the JSON
  // config, so silence just these tags (done after esp_video_init so its
  // bring-up diagnostics are still printed). A genuine CSI/ISP failure still
  // surfaces as a missing feed.
  esp_log_level_set("ISP_CCM", ESP_LOG_NONE);
  esp_log_level_set("ISP", ESP_LOG_NONE);
  esp_log_level_set("isp_video", ESP_LOG_NONE);
  esp_log_level_set("esp_video", ESP_LOG_NONE);

  // Register a PPA client and allocate a downscaled preview buffer. Each frame
  // is run through the PPA to (1) halve its size - a full-resolution frame is
  // expensive to re-render and rotate every frame, which is what made the feed
  // slow in the rotated display orientations - and (2) swap the RGB565 byte
  // order the ISP emits so it matches the LVGL canvas. Both happen in one
  // hardware pass. The callback receives this preview buffer.
  ppa_client_config_t ppa_cfg = {};
  ppa_cfg.oper_type = PPA_OPERATION_SRM;
  if (ppa_register_client(&ppa_cfg, &camera_ppa_client_) != ESP_OK) {
    logger_.error("Could not register the camera PPA client");
    stop_camera();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(camera_controls_mutex_);
    camera_active_scale_ = camera_controls_.scale;
  }
  if (!allocate_camera_preview_buffer(camera_active_scale_)) {
    stop_camera();
    return false;
  }
  logger_.info("Camera preview: {}x{} RGB565 (PPA downscaled)", camera_preview_width_,
               camera_preview_height_);

  // Apply whatever controls were requested before streaming started.
  {
    std::lock_guard<std::mutex> lock(camera_controls_mutex_);
    camera_controls_dirty_ = true;
  }
  apply_camera_controls();

  using namespace std::placeholders;
  camera_task_ = espp::Task::make_unique({
      .callback = std::bind(&M5StackTab5::camera_task_callback, this, _1, _2, _3),
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

bool M5StackTab5::allocate_camera_preview_buffer(CameraScale scale) {
  const int factor = (scale == CameraScale::FULL) ? 1 : (scale == CameraScale::QUARTER) ? 4 : 2;
  const uint16_t w = static_cast<uint16_t>((camera_width_ / factor) & ~1u);
  const uint16_t h = static_cast<uint16_t>((camera_height_ / factor) & ~1u);
  static constexpr size_t kCacheAlign = 128; // PPA output buffer alignment (L2 line)
  const size_t data_bytes = static_cast<size_t>(w) * h * 2;
  const size_t needed = (data_bytes + kCacheAlign - 1) / kCacheAlign * kCacheAlign;
  // Reuse the buffer if the size is unchanged; otherwise allocate a new one
  // first so we don't lose the current preview on OOM.
  if (camera_preview_buffer_ == nullptr || camera_preview_bytes_ != needed) {
    auto *new_buf = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(kCacheAlign, needed, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_buf == nullptr) {
      logger_.error("Could not allocate the camera preview buffer ({} bytes)", needed);
      return false;
    }
    if (camera_preview_buffer_ != nullptr) {
      heap_caps_free(camera_preview_buffer_);
    }
    camera_preview_buffer_ = new_buf;
    camera_preview_bytes_ = needed;
  }
  camera_preview_width_ = w;
  camera_preview_height_ = h;
  camera_active_scale_ = scale;
  return true;
}

void M5StackTab5::apply_camera_controls() {
  CameraControls c;
  {
    std::lock_guard<std::mutex> lock(camera_controls_mutex_);
    if (!camera_controls_dirty_) {
      return;
    }
    camera_controls_dirty_ = false;
    c = camera_controls_;
  }

  // Scale: reallocate the preview buffer if the requested scale changed. On
  // failure (OOM) the current buffer is kept and the scale is not marked
  // applied; log it rather than silently dropping the request.
  if (c.scale != camera_active_scale_) {
    if (!allocate_camera_preview_buffer(c.scale)) {
      logger_.warn("Could not apply camera scale change (out of memory); keeping the current size");
    }
  }
  // Mirror / flip are applied by the PPA pass (the sensor rejects V4L2_CID_HFLIP
  // / VFLIP on the capture device); just record the requested state here.
  camera_active_hmirror_ = c.hmirror;
  camera_active_vflip_ = c.vflip;
}

void M5StackTab5::set_camera_controls(const CameraControls &controls) {
  std::lock_guard<std::mutex> lock(camera_controls_mutex_);
  camera_controls_ = controls;
  camera_controls_dirty_ = true;
}

M5StackTab5::CameraControls M5StackTab5::camera_controls() const {
  std::lock_guard<std::mutex> lock(camera_controls_mutex_);
  return camera_controls_;
}

bool M5StackTab5::camera_task_callback(std::mutex &m, std::condition_variable &cv,
                                       bool &task_notified) {
  // Apply any pending control changes (scale / mirror / flip) here so the PPA
  // scale and the preview-buffer realloc happen in this one thread.
  apply_camera_controls();
  // Dequeue a filled frame, hand it to the callback (valid only for the call),
  // then requeue the buffer for reuse. The fd is non-blocking, so if no frame
  // is ready yet the DQBUF fails and we wait briefly - this keeps the task
  // returning regularly so a stop request is observed promptly.
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(camera_fd_, VIDIOC_DQBUF, &buf) == 0) {
    if (camera_callback_ && buf.index < CAMERA_BUFFER_COUNT && camera_buffers_[buf.index] &&
        camera_ppa_client_ && camera_preview_buffer_) {
      // Rotate the frame to match the current display orientation. The camera is
      // fixed to the tablet, so as the display is rotated the captured scene
      // tilts; rotating here keeps the scene upright in the (rotated) UI. Two
      // parts: (1) follow the display rotation using the same LVGL->step mapping
      // as the display flush (LVGL 90/180/270 -> 1/2/3 quarter-turns), and (2)
      // add a fixed mount offset, because the sensor is mounted turned 90 deg
      // clockwise relative to the panel. If the feed is still turned, adjust
      // kCameraMountSteps (0..3, each step = 90 deg CCW). For a net 90/270 turn
      // the output width/height are swapped.
      static constexpr int kCameraMountSteps = 3; // +270 CCW == 90 deg clockwise
      int disp_steps = 0;
      auto rotation = lv_display_get_rotation(lv_display_get_default());
      if (rotation == LV_DISPLAY_ROTATION_90) {
        disp_steps = 1;
      } else if (rotation == LV_DISPLAY_ROTATION_180) {
        disp_steps = 2;
      } else if (rotation == LV_DISPLAY_ROTATION_270) {
        disp_steps = 3;
      }
      int steps = (disp_steps + kCameraMountSteps) & 3;
      ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
      uint16_t out_w = camera_preview_width_;
      uint16_t out_h = camera_preview_height_;
      if (steps == 1) {
        angle = PPA_SRM_ROTATION_ANGLE_90;
        out_w = camera_preview_height_;
        out_h = camera_preview_width_;
      } else if (steps == 2) {
        angle = PPA_SRM_ROTATION_ANGLE_180;
      } else if (steps == 3) {
        angle = PPA_SRM_ROTATION_ANGLE_270;
        out_w = camera_preview_height_;
        out_h = camera_preview_width_;
      }
      // Downscale + rotate the frame into the preview buffer in one PPA pass.
      ppa_srm_oper_config_t srm = {};
      srm.in.buffer = camera_buffers_[buf.index];
      srm.in.pic_w = camera_width_;
      srm.in.pic_h = camera_height_;
      srm.in.block_w = camera_width_;
      srm.in.block_h = camera_height_;
      srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.out.buffer = camera_preview_buffer_;
      srm.out.buffer_size = camera_preview_bytes_;
      srm.out.pic_w = out_w;
      srm.out.pic_h = out_h;
      srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.rotation_angle = angle;
      // scale is applied in the input axes (before rotation), so it is the same
      // regardless of the rotation angle.
      srm.scale_x = static_cast<float>(camera_preview_width_) / static_cast<float>(camera_width_);
      srm.scale_y = static_cast<float>(camera_preview_height_) / static_cast<float>(camera_height_);
      // Mirror / flip in the same hardware pass (the sensor rejects flip on the
      // capture device). If a flip comes out on the wrong axis in a rotated
      // orientation, swap these two.
      srm.mirror_x = camera_active_hmirror_;
      srm.mirror_y = camera_active_vflip_;
      srm.mode = PPA_TRANS_MODE_BLOCKING;
      if (ppa_do_scale_rotate_mirror(camera_ppa_client_, &srm) == ESP_OK) {
        const size_t preview_len = static_cast<size_t>(out_w) * out_h * 2;
        camera_callback_(camera_preview_buffer_, out_w, out_h, preview_len);
      }
    }
    ioctl(camera_fd_, VIDIOC_QBUF, &buf);
  } else {
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
}

void M5StackTab5::stop_camera() {
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
  if (camera_ppa_client_) {
    ppa_unregister_client(camera_ppa_client_);
    camera_ppa_client_ = nullptr;
  }
  if (camera_preview_buffer_) {
    heap_caps_free(camera_preview_buffer_);
    camera_preview_buffer_ = nullptr;
    camera_preview_bytes_ = 0;
  }
  if (camera_video_inited_) {
    esp_video_deinit();
    camera_video_inited_ = false;
  }
  camera_initialized_ = false;
  camera_callback_ = nullptr;
}

uint16_t M5StackTab5::camera_width() const { return camera_width_; }

uint16_t M5StackTab5::camera_height() const { return camera_height_; }
