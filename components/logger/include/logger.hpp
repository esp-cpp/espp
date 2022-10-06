#pragma once

#include <string>
#include <string_view>

#include "format.hpp"

namespace espp {

  /**
   * @brief Logger provides a wrapper around nicer / more robust formatting than
   * standard ESP_LOG* macros with the ability to change the log level at
   * run-time. Logger currently is a light wrapper around libfmt (future
   * std::format).
   */
  class Logger {
  public:

    /**
     *   Verbosity levels for the logger, in order of increasing priority.
     */
    enum class Level {
      DEBUG, INFO, WARN, ERROR,
    };

    struct Config {
      std::string_view tag;
      Level level = Level::WARN;
    };
    Logger(const Config& config) : tag_(config.tag), level_(config.level) {}

    void set_log_level(const Level level) { level_ = level; }

    /**
     * @brief Format args into string according to format string. From:
     * https://en.cppreference.com/w/cpp/utility/format/format
     *
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     * @return formatted std::string
     */
    template <typename... Args>
    std::string format(std::string_view rt_fmt_str, Args&&... args) {
      return fmt::vformat(rt_fmt_str, fmt::make_format_args(args...));
    }

    /**
     * @brief Print log in GRAY if level is Level::DEBUG or greater.
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void debug(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::DEBUG) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::gray), "[{}/D]:{}\n", tag_, msg);
    }

    /**
     * @brief Print log in GREEN if level is Level::INFO or greater
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void info(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::INFO) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::green), "[{}/I]:{}\n", tag_, msg);
    }

    /**
     * @brief Print log in YELLOW if level is Level::WARN or greater
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void warn(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::WARN) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::yellow), "[{}/W]:{}\n", tag_, msg);
    }

    /**
     * @brief Print log in RED if level is Level::ERROR or greater
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void error(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Level::ERROR) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::red), "[{}/E]:{}\n", tag_, msg);
    }

  protected:
    /**
     *   Name given to the logger to be prepended to all logs.
     */
    std::string tag_;

    /**
     *   Current level of the logger. Determines what will be printed to
     *   console.
     */
    std::atomic<Level> level_;
  };
}
