#pragma once

#include <functional>

#include "sdkconfig.h"
#include "lvgl.h"

#include "task.hpp"

namespace espp {
  /**
   * @brief Wrapper class around LVGL display buffer and display driver.
   *
   *  Optionally allocates and owns the memory associated with the pixel display
   *  buffers. Initializes the LVGL subsystem then starts and maintains a task
   *  which runs the high priority lv_tick_inc() function every update period
   *  (default = 10 ms).
   *
   *  For more information, see
   *  https://docs.lvgl.io/8.3/porting/display.html#display-interface
   */
  class Display {
  public:
    /**
     * @brief Callback for lvgl to flush segments of pixel data from the pixel
     *        buffers to the display.
     * @param driver Pointer to display driver.
     * @param area Pointer to structure describing the area of pixels to flush.
     * @param color_data Pointer to pixel buffer containing color data.
     */
    typedef void (*flush_fn)(lv_disp_drv_t* driver, const lv_area_t* area, lv_color_t* color_data);

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

    /**
      * @brief Used if you want the Display to manage the allocation / lifecycle
      * of the display buffer memory itself.
      */
    struct AllocatingConfig {
      size_t width; /**< Width of th display, in pixels. */
      size_t height; /**< Height of the display, in pixels. */
      size_t pixel_buffer_size; /**< Size of the display buffer in pixels. */
      flush_fn flush_callback; /**< Function provided to LVGL for it to flush data to the display. */
      std::chrono::duration<float> update_period{0.01}; /**< How frequently to run the update function. */
      bool double_buffered{true}; /**< Whether to use double buffered rendering (two display buffers) or not. */
      uint32_t allocation_flags{MALLOC_CAP_8BIT | MALLOC_CAP_DMA}; /**< For configuring how the display buffer is allocated*/
      Rotation rotation{Rotation::LANDSCAPE}; /**< Default / Initial rotation of the display. */
      bool software_rotation_enabled{true}; /**< Enable LVGL software display rotation, incurs additional overhead. */
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity for the Display logger_. */
    };

    /**
      * @brief Used if you want to manage allocation / lifecycle of the display
      * buffer memory separately from this class. This structure allows you to
      * configure the Display with up to two display buffers.
      */
    struct NonAllocatingConfig {
      lv_color_t *vram0; /**< Pointer to display buffer 1, that lvgl will use. */
      lv_color_t *vram1; /**< Pointer to display buffer 2 (if double buffered), that lvgl will use. */
      size_t width; /**< Width of th display, in pixels. */
      size_t height; /**< Height of the display, in pixels. */
      size_t pixel_buffer_size; /**< Size of the display buffer in pixels. */
      flush_fn flush_callback; /**< Function provided to LVGL for it to flush data to the display. */
      std::chrono::duration<float> update_period{0.01}; /**< How frequently to run the update function. */
      Rotation rotation{Rotation::LANDSCAPE}; /**< Default / Initial rotation of the display. */
      bool software_rotation_enabled{true}; /**< Enable LVGL software display rotation, incurs additional overhead. */
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity for the Display logger_. */
    };

    /**
     * @brief Allocate the dsiplay buffers, initialize LVGL, then start the
     *        update task.
     * @param config Display configuration including buffer size and flush
     *        callback.
     */
    Display(const AllocatingConfig& config)
      : width_(config.width), height_(config.height),
        display_buffer_px_size_(config.pixel_buffer_size),
        update_period_(config.update_period),
        logger_({.tag="Display", .level=config.log_level}) {
      logger_.debug("Initializing with allocating config!");
      // create the display buffers
      vram_0_ = (lv_color_t*)heap_caps_malloc(vram_size_bytes(), config.allocation_flags);
      assert(vram_0_ != NULL);
      if (config.double_buffered) {
        vram_1_ = (lv_color_t*)heap_caps_malloc(vram_size_bytes(), config.allocation_flags);
        assert(vram_1_ != NULL);
      }
      created_vram_ = true;
      init(config.flush_callback, config.software_rotation_enabled, config.rotation);
    }

    /**
     * @brief Initialize LVGL then start the update task.
     * @param config Display configuration including pointers to display buffer
     *        memory, the pixel buffer size and flush callback.
     */
    Display(const NonAllocatingConfig& config)
      : width_(config.width), height_(config.height),
        display_buffer_px_size_(config.pixel_buffer_size),
        vram_0_(config.vram0), vram_1_(config.vram1),
        update_period_(config.update_period),
        logger_({.tag="Display", .level=config.log_level}) {
      logger_.debug("Initializing with non-allocating config!");
      init(config.flush_callback, config.software_rotation_enabled, config.rotation);
    }

    /**
     * @brief Stops the upate task and frees the display buffer memory.
     */
    ~Display() {
      task_->stop();
      if (created_vram_) {
        free(vram_0_);
        free(vram_1_);
      }
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
     * @brief Force a redraw / refresh of the display.
     *
     * @note This is mainly useful after you have called pause() on the display
     *       (to draw to it with something other than LVGL) and want to switch
     *       back to the LVGL gui. Normally you should not call this function.
     */
    void force_refresh() {
      auto disp = lv_disp_get_default();
      // lv_refr_now(disp);
      lv_area_t area = {
        .x1 = 0,
        .y1 = 0,
        .x2 = (int16_t)width_,
        .y2 = (int16_t)height_
      };
      _lv_inv_area(disp, &area);
    }

    /**
     * @brief Get pointer to main display buffer for custom writing.
     * @return uint16_t* Pointer to the main display buffer.
     */
    uint16_t* vram0() { return (uint16_t*) vram_0_; }

    /**
     * @brief Get pointer to secondary display buffer for custom writing.
     * @return uint16_t* Pointer to the secondary display buffer.
     */
    uint16_t* vram1() { return (uint16_t*) vram_1_; }

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
      * @brief Initialize the lvgl subsystem, display buffer configuration, and
      *        display driver. Start the task to run the high-priority lvgl
      *        task.
      * @param flush_callback Callback used to flush color data to the display.
      * @param sw_rotation_enabled Whether to use software roation (slower) or
      *        not.
      * @param rotation Default / initial rotation of the display.
      */
    void init(flush_fn flush_callback, bool sw_rotation_enabled, Rotation rotation) {
      lv_init();

      // Configure the LVGL display buffer with our pixel buffers
      lv_disp_draw_buf_init(&disp_buffer_, vram_0_, vram_1_, display_buffer_px_size_);

      lv_disp_drv_init(&disp_driver_);
      disp_driver_.draw_buf = &disp_buffer_;
      disp_driver_.flush_cb = flush_callback;
      disp_driver_.sw_rotate = (uint8_t)sw_rotation_enabled;
      disp_driver_.ver_res = height_;
      disp_driver_.hor_res = width_;
      disp_driver_.rotated = (uint8_t)rotation;

      // Register the display driver with lvgl
      lv_disp_drv_register(&disp_driver_);

      // Now start the task for the ui management
      using namespace std::placeholders;
      task_ = Task::make_unique({
          .name = "Display",
          .callback = std::bind(&Display::update, this, _1, _2),
          .stack_size_bytes = 4096 * 2,
          .priority = 20,
          .core_id = 0, // pin it to a core for maximum speed
        });
      task_->start();

    }

    /**
     * @brief Flush the data to the display, called within the task_.
     *
     *   This task should always be high priority, so that it is higher than
     *   than the task running lv_task_handler(). For more info, see
     *   https://docs.lvgl.io/latest/en/html/porting/tick.html
     */
    void update(std::mutex& m, std::condition_variable& cv) {
      static auto prev = std::chrono::high_resolution_clock::now();
      if (!paused_){
        auto now = std::chrono::high_resolution_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now-prev).count();
        // we shouldn't stop, update the display
        lv_tick_inc(elapsed_ms);
        // update previous timestamp
        prev = now;
      }
      // delay
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, update_period_);
      }
      // don't want to stop the task
      return false;
    }

    std::atomic<bool> paused_{false};
    std::unique_ptr<Task> task_;
    size_t width_;
    size_t height_;
    size_t display_buffer_px_size_;
    lv_color_t *vram_0_{nullptr};
    lv_color_t *vram_1_{nullptr};
    bool created_vram_{false};
    std::chrono::duration<float> update_period_;
    lv_disp_draw_buf_t disp_buffer_;
    lv_disp_drv_t disp_driver_;
    Logger logger_;
  };
}
