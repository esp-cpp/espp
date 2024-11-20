#pragma once

#include <memory>
#include <mutex>

#include "display.hpp"
#include "logger.hpp"
#include "task.hpp"

class Gui {
public:
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit Gui(const Config &config)
      : logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    // now start the gui updater task
    using namespace std::placeholders;
    task_ = espp::Task::make_unique({.name = "Gui Task",
                                     .callback = std::bind(&Gui::update, this, _1, _2, _3),
                                     .stack_size_bytes = 6 * 1024});
    task_->start();
  }

  void set_label(const std::string_view &label) {
    std::scoped_lock<std::recursive_mutex> lk(mutex_);
    lv_label_set_text(label_, label.data());
  }

  void set_meter(size_t value, bool animate = true) {
    std::scoped_lock<std::recursive_mutex> lk(mutex_);
    if (animate) {
      lv_bar_set_value(meter_, value, LV_ANIM_ON);
    } else {
      lv_bar_set_value(meter_, value, LV_ANIM_OFF);
    }
  }

protected:
  void init_ui() {
    auto display = lv_display_get_default();
    auto hor_res = lv_display_get_horizontal_resolution(display);
    auto ver_res = lv_display_get_vertical_resolution(display);

    // Create a container with COLUMN flex direction
    column_ = lv_obj_create(lv_screen_active());
    lv_obj_set_size(column_, hor_res, ver_res);
    lv_obj_set_flex_flow(column_, LV_FLEX_FLOW_COLUMN);

    label_ = lv_label_create(column_);
    lv_label_set_text(label_, "Hello world");

    meter_ = lv_bar_create(lv_screen_active());
    lv_obj_set_size(meter_, hor_res * 0.8f, 20);
    lv_obj_center(meter_);

    static lv_style_t style_indic;
    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_HOR);

    lv_obj_add_style(meter_, &style_indic, LV_PART_INDICATOR);
  }

  bool update(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    {
      std::scoped_lock<std::recursive_mutex> lk(mutex_);
      lv_task_handler();
    }
    {
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 16ms, [&task_notified] { return task_notified; });
      task_notified = false;
    }
    // don't want to stop the task
    return false;
  }

  // LVLG gui objects
  lv_obj_t *column_;
  lv_obj_t *label_;
  lv_obj_t *meter_;

  std::unique_ptr<espp::Task> task_;
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
