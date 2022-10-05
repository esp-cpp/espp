#pragma once

#include <string>
#include <string_view>

#include "format.hpp"

namespace espp {
  class Logger {
  public:

    enum class Level {
      DEBUG, INFO, WARN, ERROR,
    };

    struct Config {
      std::string_view tag;
      Level level = Level::WARN;
    };
    Logger(const Config& config) : tag_(config.tag), level_(config.level) {}

    void set_log_level(const Level level) { level_ = level; }

    // from https://en.cppreference.com/w/cpp/utility/format/format
    template <typename... Args>
    std::string format(std::string_view rt_fmt_str, Args&&... args) {
      return fmt::vformat(rt_fmt_str, fmt::make_format_args(args...));
    }

    // print log in GRAY if level is DEBUG or greater
    template <typename... Args>
    void debug(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::DEBUG) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::gray), "[{}/D]:{}\n", tag_, msg);
    }

    // print log in GREEN if level is INFO or greater
    template <typename... Args>
    void info(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::INFO) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::green), "[{}/I]:{}\n", tag_, msg);
    }

    // print log in YELLOW if level is WARN or greater
    template <typename... Args>
    void warn(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::WARN) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::yellow), "[{}/W]:{}\n", tag_, msg);
    }

    // print log in RED if level is ERROR or greater
    template <typename... Args>
    void error(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::ERROR) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::red), "[{}/E]:{}\n", tag_, msg);
    }

  protected:
    std::string tag_;
    std::atomic<Level> level_;
  };
}
