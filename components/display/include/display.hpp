#pragma once

#include <functional>
#include <vector>

#include <lvgl.h>
#include <sdkconfig.h>

#include <driver/gpio.h>

#include "base_component.hpp"
#include "led.hpp"
#include "task.hpp"

namespace espp {

/**
 *  @brief Possible orientations of the display.
 */
enum class DisplayRotation : uint8_t { LANDSCAPE, PORTRAIT, LANDSCAPE_INVERTED, PORTRAIT_INVERTED };

/**
 *  @brief Signals used by LVGL to let the post_transfer_callback know
 *         whether or not to call lv_disp_flush_ready.
 */
enum class DisplaySignal : uint32_t { NONE, FLUSH };

/**
 * @brief Wrapper class around LVGL display buffer and display driver.
 *
 *  Optionally allocates and owns the memory associated with the pixel display
 *  buffers. Initializes the LVGL subsystem then starts and maintains a task
 *  which runs the high priority lv_tick_inc() function every update period
 *  (default = 10 ms).
 *
 *  For more information, see
 *  https://docs.lvgl.io/9.1/porting/display.html#display-interface
 *
 * @tparam Pixel The pixel format to be used for the display. This allows the class to be used with
 * displays of different color depths and formats.
 */
template <typename Pixel> class Display : public BaseComponent {
public:
  /**
   * @brief Callback for lvgl to flush segments of pixel data from the pixel
   *        buffers to the display.
   */
  using flush_fn = lv_display_flush_cb_t;

  /**
   * @brief Callback for lvgl event handler to reconfigure the display hardware.
   */
  typedef void (*rotation_fn)(const DisplayRotation &rotation);

  /**
   * @brief Callback for setting the display brightness.
   * @param brightness Brightness value between 0.0 and 1.0.
   */
  typedef std::function<void(float brightness)> set_brightness_fn;

  /**
   * @brief Callback for getting the display brightness.
   * @return float Brightness value between 0.0 and 1.0.
   */
  typedef std::function<float()> get_brightness_fn;

  /**
   * @brief Base component configuration for LVGL.
   */
  struct LvglConfig {
    size_t width;            /**< Width of th display, in pixels. */
    size_t height;           /**< Height of the display, in pixels. */
    flush_fn flush_callback; /**< Function provided to LVGL for it to flush data to the display. */
    rotation_fn rotation_callback{
        nullptr}; /**< Optional function used to configure display with new rotation setting. */
    DisplayRotation rotation{
        DisplayRotation::LANDSCAPE}; /**< Default / Initial rotation of the display. */
    Task::BaseConfig task_config{.name = "Display",
                                 .stack_size_bytes = 4096,
                                 .priority = 20,
                                 .core_id = 0}; /**< Task configuration. */
    std::chrono::duration<float> update_period{
        0.01}; /**< How frequently to run the update function. */
  };

  /**
   * @brief OLED specific configuration.
   */
  struct OledConfig {
    set_brightness_fn set_brightness_callback{
        nullptr}; /**< Callback for setting the display brightness */
    get_brightness_fn get_brightness_callback{nullptr}; /**< Callback for getting the display
                                            brightness. */
  };

  /**
   * @brief LCD specific configuration.
   */
  struct LcdConfig {
    gpio_num_t backlight_pin{GPIO_NUM_NC}; /**< GPIO pin for the backlight. */
    bool backlight_on_value{
        true}; /**< Value to write to the backlight pin to turn the backlight on. */
  };

  /**
   * @brief Used if you want the Display to manage the allocation / lifecycle
   * of the display buffer memory itself.
   */
  struct DynamicMemoryConfig {
    size_t pixel_buffer_size; /**< Size of the display buffer in pixels. */
    bool double_buffered{
        true}; /**< Whether to use double buffered rendering (two display buffers) or not. */
    uint32_t allocation_flags{
        MALLOC_CAP_8BIT |
        MALLOC_CAP_DMA}; /**< For configuring how the display buffer is allocated*/
  };

  /**
   * @brief Used if you want to manage allocation / lifecycle of the display
   * buffer memory separately from this class. This structure allows you to
   * configure the Display with up to two display buffers.
   */
  struct StaticMemoryConfig {
    size_t pixel_buffer_size; /**< Size of the display buffer in pixels. */
    Pixel *vram0{nullptr};    /**< Pointer to display buffer 1, that lvgl will use. */
    Pixel *vram1{nullptr}; /**< Optional pointer to display buffer 2 (if double buffered), that lvgl
                           will use. */
  };

  /**
   * @brief Initialize LVGL then start the update task.
   * @param lvgl_conf LVGL Configuration, including display size, flush callback and optional
   * rotation callback.
   * @param lcd_config LCD specific configuration.
   * @param mem_conf Static memory configuration.
   * @param log_level The verbosity level for the Display logger_.
   */
  explicit Display(const LvglConfig &lvgl_conf, const LcdConfig &lcd_config,
                   const StaticMemoryConfig &mem_conf,
                   const Logger::Verbosity log_level = Logger::Verbosity::WARN)
      : BaseComponent("Display", log_level)
      , width_(lvgl_conf.width)
      , height_(lvgl_conf.height)
      , display_buffer_px_size_(mem_conf.pixel_buffer_size)
      , vram_0_(mem_conf.vram0)
      , vram_1_(mem_conf.vram1)
      , update_period_(lvgl_conf.update_period) {
    init_backlight(lcd_config.backlight_pin, lcd_config.backlight_on_value);
    init_gfx(lvgl_conf.flush_callback, lvgl_conf.rotation_callback, lvgl_conf.rotation,
             lvgl_conf.task_config);
    set_brightness(1.0);
  }

  /**
   * @brief Allocate the display buffers, initialize LVGL then start the update task.
   * @param lvgl_conf LVGL Configuration, including display size, flush callback and optional
   * rotation callback.
   * @param lcd_config LCD specific configuration.
   * @param mem_conf Dynamic memory configuration.
   * @param log_level The verbosity level for the Display logger_.
   */
  explicit Display(const LvglConfig &lvgl_conf, const LcdConfig &lcd_config,
                   const DynamicMemoryConfig &mem_conf,
                   const Logger::Verbosity log_level = Logger::Verbosity::WARN)
      : BaseComponent("Display", log_level)
      , width_(lvgl_conf.width)
      , height_(lvgl_conf.height)
      , display_buffer_px_size_(mem_conf.pixel_buffer_size)
      , update_period_(lvgl_conf.update_period) {
    init_memory(mem_conf.double_buffered, mem_conf.allocation_flags);
    init_backlight(lcd_config.backlight_pin, lcd_config.backlight_on_value);
    init_gfx(lvgl_conf.flush_callback, lvgl_conf.rotation_callback, lvgl_conf.rotation,
             lvgl_conf.task_config);
    set_brightness(1.0);
  }

  /**
   * @brief Initialize LVGL then start the update task.
   * @param lvgl_conf LVGL Configuration, including display size, flush callback and optional
   * rotation callback.
   * @param oled_config OLED specific configuration.
   * @param mem_conf Static memory configuration.
   * @param log_level The verbosity level for the Display logger_.
   */
  explicit Display(const LvglConfig &lvgl_conf, const OledConfig &oled_config,
                   const StaticMemoryConfig &mem_conf,
                   const Logger::Verbosity log_level = Logger::Verbosity::WARN)
      : BaseComponent("Display", log_level)
      , width_(lvgl_conf.width)
      , height_(lvgl_conf.height)
      , display_buffer_px_size_(mem_conf.pixel_buffer_size)
      , vram_0_(mem_conf.vram0)
      , vram_1_(mem_conf.vram1)
      , update_period_(lvgl_conf.update_period)
      , set_brightness_(oled_config.set_brightness_callback)
      , get_brightness_(oled_config.get_brightness_callback) {
    init_gfx(lvgl_conf.flush_callback, lvgl_conf.rotation_callback, lvgl_conf.rotation,
             lvgl_conf.task_config);
    set_brightness(1.0);
  }

  /**
   * @brief Allocate the display buffers, initialize LVGL then start the update task.
   * @param lvgl_conf LVGL Configuration, including display size, flush callback and optional
   * rotation callback.
   * @param oled_config OLED specific configuration.
   * @param mem_conf Dynamic memory configuration.
   * @param log_level The verbosity level for the Display logger_.
   */
  explicit Display(const LvglConfig &lvgl_conf, const OledConfig &oled_config,
                   const DynamicMemoryConfig &mem_conf,
                   const Logger::Verbosity log_level = Logger::Verbosity::WARN)
      : BaseComponent("Display", log_level)
      , width_(lvgl_conf.width)
      , height_(lvgl_conf.height)
      , display_buffer_px_size_(mem_conf.pixel_buffer_size)
      , update_period_(lvgl_conf.update_period)
      , set_brightness_(oled_config.set_brightness_callback)
      , get_brightness_(oled_config.get_brightness_callback) {
    init_memory(mem_conf.double_buffered, mem_conf.allocation_flags);
    init_gfx(lvgl_conf.flush_callback, lvgl_conf.rotation_callback, lvgl_conf.rotation,
             lvgl_conf.task_config);
    set_brightness(1.0);
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
   * @brief Set the brightness of the display.
   * @param brightness Brightness value between 0.0 and 1.0.
   */
  void set_brightness(float brightness) {
    if (backlight_) {
      brightness = std::clamp(brightness, 0.0f, 1.0f);
      backlight_->set_duty(led_channel_configs_[0].channel, brightness * 100.0f);
    }
    if (set_brightness_ != nullptr) {
      set_brightness_(brightness);
    }
  }

  /**
   * @brief Get the brightness of the display.
   * @return float Brightness value between 0.0 and 1.0.
   */
  float get_brightness() const {
    if (backlight_) {
      auto maybe_duty = backlight_->get_duty(led_channel_configs_[0].channel);
      if (maybe_duty.has_value()) {
        return maybe_duty.value() / 100.0f;
      }
    }
    if (get_brightness_ != nullptr) {
      return get_brightness_();
    }
    return 0.0f;
  }

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
  void force_refresh() const {
    auto disp = lv_display_get_default();
    lv_obj_invalidate(lv_scr_act());
  }

  /**
   * @brief Get pointer to main display buffer for custom writing.
   * @return uint16_t* Pointer to the main display buffer.
   */
  Pixel *vram0() { return vram_0_; }

  /**
   * @brief Get pointer to secondary display buffer for custom writing.
   * @return uint16_t* Pointer to the secondary display buffer.
   */
  Pixel *vram1() { return vram_1_; }

  /**
   * @brief Return the number of pixels that vram() can hold.
   * @return size_t Number of pixels that fit in the display buffer.
   */
  size_t vram_size_px() const { return display_buffer_px_size_; }

  /**
   * @brief Return the number of bytes that vram() can hold.
   * @return size_t Number of bytes that fit in the display buffer.
   */
  size_t vram_size_bytes() const { return display_buffer_px_size_ * sizeof(Pixel); }

protected:
  /**
   * @brief LVGL event handler.
   */
  static void event_cb(lv_event_t *event) {
    if (lv_event_get_code(event) == LV_EVENT_RESOLUTION_CHANGED) {
      auto rotation = lv_display_get_rotation(lv_display_get_default());
      auto rotation_callback = reinterpret_cast<rotation_fn>(lv_event_get_user_data(event));
      if (rotation_callback != nullptr) {
        rotation_callback(static_cast<DisplayRotation>(rotation));
      }
    }
  };

  /**
   * @brief Initialize the lvgl subsystem, display buffer configuration, and
   *        display driver. Start the task to run the high-priority lvgl
   *        task.
   * @param flush_callback Callback used to flush color data to the display.
   * @param rotation_callback function to call in the event handler on rotation change.
   * @param rotation Default / initial rotation of the display.
   * @param task_config Configuration for the task that runs the lvgl tick
   */
  void init_gfx(const flush_fn flush_callback, const rotation_fn rotation_callback,
                DisplayRotation rotation, const Task::BaseConfig &task_config) {
    lv_init();

    display_ = lv_display_create(width_, height_);

    // Configure the lvgl display buffer with our pixel buffers
    lv_display_set_buffers(display_, vram_0_, vram_1_, vram_size_bytes(),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display_, flush_callback);

    if (rotation_callback != nullptr) {
      lv_display_add_event_cb(display_, event_cb, LV_EVENT_RESOLUTION_CHANGED,
                              reinterpret_cast<void *>(rotation_callback));
    }

    lv_display_set_rotation(display_, static_cast<lv_display_rotation_t>(rotation));

    // Setting as default display, allows the use of lv_disp_get_default()
    lv_display_set_default(display_);

    // Now start the task for the ui management
    using namespace std::placeholders;
    task_ = Task::make_unique({
        .callback = std::bind(&Display::update, this, _1, _2, _3),
        .task_config = task_config,
    });
    task_->start();
  }

  /**
   * @brief Initialize the PWM controlled backlight.
   * @param backlight_pin GPIO pin for the backlight. Does check for GPIO_NUM_NC.
   * @param backlight_on_value Value to write to the backlight pin to turn the backlight on.
   */
  void init_backlight(const gpio_num_t backlight_pin, const bool backlight_on_value) {
    if (backlight_pin != GPIO_NUM_NC) {
      led_channel_configs_.push_back({.gpio = static_cast<size_t>(backlight_pin),
                                      .channel = LEDC_CHANNEL_0,
                                      .timer = LEDC_TIMER_0,
                                      .output_invert = !backlight_on_value});

      backlight_ = std::make_unique<Led>((Led::Config{.timer = LEDC_TIMER_0,
                                                      .frequency_hz = 5000,
                                                      .channels = led_channel_configs_,
                                                      .duty_resolution = LEDC_TIMER_10_BIT}));
    } else {
      if (set_brightness_ == nullptr || get_brightness_ == nullptr) {
        logger_.warn("No backlight pin provided and no brightness control callbacks provided!");
      }
    }
  }

  /**
   * @brief Allocate the display buffer(s).
   * @param double_buffered Whether to use double buffered rendering or not, will allocate a second
   * buffer if true.
   * @param allocation_flags Flags passed to heap_caps_malloc for memory allocation.
   */
  void init_memory(const bool double_buffered, const uint32_t allocation_flags) {
    vram_0_ = static_cast<Pixel *>(heap_caps_malloc(vram_size_bytes(), allocation_flags));
    assert(vram_0_ != NULL && "Failed to allocate display buffer vram_0");
    if (double_buffered) {
      vram_1_ = static_cast<Pixel *>(heap_caps_malloc(vram_size_bytes(), allocation_flags));
      assert(vram_1_ != NULL && "Failed to allocate display buffer vram_1");
    }
    created_vram_ = true;
  }

  /**
   * @brief Flush the data to the display, called within the task_.
   *
   *   This task should always be high priority, so that it is higher than
   *   than the task running lv_task_handler(). For more info, see
   *   https://docs.lvgl.io/latest/en/html/porting/tick.html
   */
  bool update(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    auto now = std::chrono::high_resolution_clock::now();
    static auto prev = now;
    if (!paused_) {
      int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count();
      // we shouldn't stop, update the display
      lv_tick_inc(elapsed_ms);
    }
    // update previous timestamp
    prev = now;
    // delay
    {
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, update_period_, [&task_notified] { return task_notified; });
      task_notified = false;
    }
    // don't want to stop the task
    return false;
  }

  std::atomic<bool> paused_{false};
  std::unique_ptr<Task> task_;
  size_t width_;
  size_t height_;
  size_t display_buffer_px_size_;
  Pixel *vram_0_{nullptr};
  Pixel *vram_1_{nullptr};
  bool created_vram_{false};
  std::vector<Led::ChannelConfig> led_channel_configs_;
  std::unique_ptr<Led> backlight_{nullptr};
  std::chrono::duration<float> update_period_;
  lv_display_t *display_;
  set_brightness_fn set_brightness_{nullptr};
  get_brightness_fn get_brightness_{nullptr};
};
} // namespace espp
