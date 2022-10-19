#pragma once

#include <functional>

#include "sdkconfig.h"
#include "lvgl.h"

#include "task.hpp"

namespace espp {
  /**
   * @brief Wrapper class around LVGL display buffer and display driver.
   *
   *  Allocates and owns the memory associated with the pixel display buffers.
   *  Starts and maintains a task which runs the high priority lv_tick_inc()
   *  function every 10 ms.
   *
   *  For more information, see
   *  https://docs.lvgl.io/8.3/porting/display.html#display-interface
   */
  class Display {
  public:
    /**
      * @brief Callback for lvgl to flush segments of pixel data from the pixel
      *        buffers to the display.
      * @param lv_disp_drv_t* Pointer to display driver.
      * @param const lv_area_t* Pointer to structure describing the area of
      *        pixels to flush.
      * @param lv_color_t* Pointer to pixel buffer containing color data.
      */
    typedef void (*flush_fn)(lv_disp_drv_t*, const lv_area_t*, lv_color_t*);

    /**
      * @brief Low-level callback to write bytes to the display controller.
      * @param uint8_t* Pointer to array of bytes to write.
      * @param size_t Number of bytes to write.
      * @param uint16_t user data associated with this transfer, used for flags.
      */
    typedef std::function<void(uint8_t*, size_t, uint16_t)> write_fn;

    /**
     *  @brief Signals used by LVGL to let the post_transfer_callback know
     *         whether or not to call lv_disp_flush_ready.
     */
    enum class Signal : uint32_t {
      NONE,
      FLUSH
    };

    /**
     *  @brief Possible orientations of the display.
     */
    enum class Rotation : uint8_t {
      LANDSCAPE,
      PORTRAIT,
      LANDSCAPE_INVERTED,
      PORTRAIT_INVERTED
    };

    struct Config {
      size_t width; /**< Width of th display, in pixels. */
      size_t height; /**< Height of the display, in pixels. */
      size_t pixel_buffer_size; /**< Size of the display buffer in pixels. */
      flush_fn flush_callback; /**< Function provided to LVGL for it to flush data to the display. */
      Rotation rotation{Rotation::LANDSCAPE}; /**< Default / Initial rotation of the display. */
      bool software_rotation_enabled{true}; /**< Enable LVGL software display rotation, incurs additional overhead. */
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity for the Display logger_. */
    };

    /**
     * @brief Setup the dsiplay buffers and low-level LVGL, then start the
     *        update task.
     * @param config Display configuration including buffer size and flush
     *        callback.
     */
    Display(const Config& config)
      : width_(config.width), height_(config.height),
        display_buffer_px_size_(config.pixel_buffer_size),
        logger_({.tag="Display", .level=config.log_level}) {
      lv_init();

      // create the display buffers
      // NOTE: could also use LV_BUF_ALLOC_INTERNAL
      uint32_t malloc_caps = MALLOC_CAP_DMA;
      vram_0_ = (lv_color_t*)heap_caps_malloc(vram_size_bytes(), malloc_caps);
      assert(vram_0_ != NULL);

      // Use double buffered when not working with monochrome displays
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
      vram_1_ = (lv_color_t*)heap_caps_malloc(vram_size_bytes(), malloc_caps);
      assert(vram_1_ != NULL);
#endif

      // Configure the LVGL display buffer with our pixel buffers
      lv_disp_draw_buf_init(&disp_buffer_, vram_0_, vram_1_, config.pixel_buffer_size);

      lv_disp_drv_init(&disp_driver_);
      disp_driver_.draw_buf = &disp_buffer_;
      disp_driver_.flush_cb = config.flush_callback;
      disp_driver_.sw_rotate = (uint8_t)config.software_rotation_enabled;
      disp_driver_.ver_res = config.height;
      disp_driver_.hor_res = config.width;
      disp_driver_.rotated = (uint8_t)config.rotation;

      // Register the display driver with lvgl
      lv_disp_drv_register(&disp_driver_);

      // Now start the task for the ui management
      using namespace std::placeholders;
      task_ = Task::make_unique({
          .name = "Display",
          .callback = std::bind(&Display::update, this, _1, _2),
          .stack_size_bytes = 4096 * 2,
          .priority = 99,
          .core_id = 0, // pin it to a core for maximum speed
        });
      task_->start();
    }

    /**
     * @brief Stops the upate task and frees the display buffer memory.
     */
    ~Display() {
      task_->stop();
      free(vram_0_);
      free(vram_1_);
    }

    /**
     * @brief Return the configured width of the display in pixels.
     * @return size_t width of the display.
     */
    size_t width() const { return width_; }

    /**
     * @brief Return the configured height of the display in pixels.
     * @return size_t height of the display.
     */
    size_t height() const { return height_; }

    /**
     * @brief Pause the display update task, to prevent LVGL from writing to the
     *        display.
     */
    void pause() { paused_ = true; }

    /**
     * @brief Resume the display update task, to allow LVGL to write to the
     *        display.
     */
    void resume() { paused_ = false; }

    /**
     * @brief Get pointer to main display buffer for custom writing.
     * @return uint16_t* Pointer to the main display buffer.
     */
    uint16_t* vram() { return (uint16_t*) vram_0_; }

    /**
     * @brief Return the number of pixels that vram() can hold.
     * @return size_t Number of pixels that fit in the display buffer.
     */
    size_t vram_size_px() { return display_buffer_px_size_; }

    /**
     * @brief Return the number of bytes that vram() can hold.
     * @return size_t Number of bytes that fit in the display buffer.
     */
    size_t vram_size_bytes() { return display_buffer_px_size_ * sizeof(lv_color_t); }

  protected:
    /**
     * @brief Flush the data to the display, called within the task_.
     *
     *   This task should always be high priority, so that it is higher than
     *   than the task running lv_task_handler(). For more info, see
     *   https://docs.lvgl.io/latest/en/html/porting/tick.html
     */
    void update(std::mutex& m, std::condition_variable& cv) {
      if (!paused_){
        // we shouldn't stop, update the display
        lv_tick_inc(10);
      }
      // delay
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
    }

    std::atomic<bool> paused_{false};
    std::unique_ptr<Task> task_;
    size_t width_;
    size_t height_;
    size_t display_buffer_px_size_;
    lv_color_t *vram_0_{nullptr};
    lv_color_t *vram_1_{nullptr};
    lv_disp_draw_buf_t disp_buffer_;
    lv_disp_drv_t disp_driver_;
    Logger logger_;
  };
}
