#include <chrono>
#include <deque>
#include <stdlib.h>
#include <vector>

#include "esp-box.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
static std::deque<lv_obj_t *> circles;
static std::vector<uint8_t> audio_bytes;

static std::recursive_mutex lvgl_mutex;
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

static size_t load_audio();
static void play_click(espp::EspBox &box);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ESP BOX Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp box example]
  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on {}", box.box_type());

  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = box.touchpad_convert(touch);
    auto touchpad_data = box.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
      }
      // if there is a touch point, draw a circle and play a click sound
      if (touchpad_data.num_touch_points > 0) {
        play_click(box);
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };

  // initialize the sound
  if (!box.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  // initialize the LCD
  if (!box.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = box.lcd_width() * 50;
  espp::Task::BaseConfig display_task_config = {
      .name = "Display",
      .stack_size_bytes = 6 * 1024,
      .priority = 10,
      .core_id = 0,
  };
  // initialize the LVGL display for the esp-box
  if (!box.initialize_display(pixel_buffer_size, display_task_config)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // initialize the touchpad
  if (!box.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, box.lcd_width(), box.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Touch the screen!\nPress the home button to clear circles.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

  // add a button in the top left which (when pressed) will rotate the display
  // through 0, 90, 180, 270 degrees
  lv_obj_t *btn = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn, 50, 50);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_t *label_btn = lv_label_create(btn);
  lv_label_set_text(label_btn, LV_SYMBOL_REFRESH);
  // center the text in the button
  lv_obj_align(label_btn, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(
      btn,
      [](auto event) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
        static auto rotation = LV_DISPLAY_ROTATION_0;
        rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
        lv_display_t *disp = _lv_refr_get_disp_refreshing();
        lv_disp_set_rotation(disp, rotation);
      },
      LV_EVENT_PRESSED, nullptr);

  // disable scrolling on the screen (so that it doesn't behave weirdly when
  // rotated and drawing with your finger)
  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

  // start a simple thread to do the lv_task_handler every 16ms
  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        {
                          std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
                          lv_task_handler();
                        }
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {
                          .name = "lv_task",
                      }});
  lv_task.start();

  // load the audio file (wav file bundled in memory)
  size_t wav_size = load_audio();
  logger.info("Loaded {} bytes of audio", wav_size);

  // unmute the audio and set the volume to 60%
  box.mute(false);
  box.volume(60.0f);

  // set the display brightness to be 75%
  box.brightness(75.0f);

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [esp box example]
}

static void draw_circle(int x0, int y0, int radius) {
  // if the number of circles is greater than the max, remove the oldest circle
  if (circles.size() > MAX_CIRCLES) {
    lv_obj_delete(circles.front());
    circles.pop_front();
  }
  lv_obj_t *my_Cir = lv_obj_create(lv_screen_active());
  lv_obj_set_scrollbar_mode(my_Cir, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(my_Cir, radius * 2, radius * 2);
  lv_obj_set_pos(my_Cir, x0 - radius, y0 - radius);
  lv_obj_set_style_radius(my_Cir, LV_RADIUS_CIRCLE, 0);
  // ensure the circle ignores touch events (so things behind it can still be
  // interacted with)
  lv_obj_clear_flag(my_Cir, LV_OBJ_FLAG_CLICKABLE);
  circles.push_back(my_Cir);
}

static void clear_circles() {
  // remove the circles from lvgl
  for (auto circle : circles) {
    lv_obj_delete(circle);
  }
  // clear the vector
  circles.clear();
}

static size_t load_audio() {
  // if the audio_bytes vector is already populated, return the size
  if (audio_bytes.size() > 0) {
    return audio_bytes.size();
  }

  // these are configured in the CMakeLists.txt file
  extern const char wav_start[] asm("_binary_click_wav_start");
  extern const char wav_end[] asm("_binary_click_wav_end");

  // -1 due to the size being 1 byte too large, I think because end is the byte
  // immediately after the last byte in the memory but I'm not sure - cmm 2022-08-20
  //
  // Suppression as these are linker symbols and cppcheck doesn't know how to ensure
  // they are the same object
  // cppcheck-suppress comparePointers
  size_t wav_size = (wav_end - wav_start) - 1;
  FILE *fp = fmemopen((void *)wav_start, wav_size, "rb");
  // read the file into the audio_bytes vector
  audio_bytes.resize(wav_size);
  fread(audio_bytes.data(), 1, wav_size, fp);
  fclose(fp);
  return wav_size;
}

static void play_click(espp::EspBox &box) {
  // use the box.play_audio() function to play a sound, breaking it into
  // audio_buffer_size chunks
  auto audio_buffer_size = box.audio_buffer_size();
  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    size_t bytes_to_play = std::min(audio_buffer_size, audio_bytes.size() - offset);
    box.play_audio(audio_bytes.data() + offset, bytes_to_play);
    offset += bytes_to_play;
  }
}
