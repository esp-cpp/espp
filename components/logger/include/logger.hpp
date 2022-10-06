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
   *
   * \section logger_ex1 Example
   * \snippet logger_example.cpp Logger example
   */
  class Logger {
  public:

    /**
     *   Verbosity levels for the logger, in order of increasing priority.
     */
    enum class Verbosity {
      DEBUG, INFO, WARN, ERROR,
    };

    struct Config {
      std::string_view tag; /**< The TAG that will be prepended to all logs. */
      Verbosity level = Verbosity::WARN; /**< The verbosity level for the logger. */
    };
    Logger(const Config& config) : tag_(config.tag), level_(config.level) {}

    /**
     * @brief Change the verbosity for the logger. \sa Logger::Verbosity
     * @param level new verbosity level
     */
    void set_log_level(const Verbosity level) { level_ = level; }

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
     * @brief Print log in GRAY if level is Verbosity::DEBUG or greater.
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void debug(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Verbosity::DEBUG) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::gray), "[{}/D]:{}\n", tag_, msg);
    }

    /**
     * @brief Print log in GREEN if level is Verbosity::INFO or greater
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void info(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Verbosity::INFO) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::green), "[{}/I]:{}\n", tag_, msg);
    }

    /**
     * @brief Print log in YELLOW if level is Verbosity::WARN or greater
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void warn(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Verbosity::WARN) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::yellow), "[{}/W]:{}\n", tag_, msg);
    }

    /**
     * @brief Print log in RED if level is Verbosity::ERROR or greater
     * @param rt_fmt_str format string
     * @param args optional arguments passed to be formatted.
     */
    template <typename... Args>
    void error(std::string_view rt_fmt_str, Args&&... args) {
      if (level_ > Verbosity::ERROR) return;
      auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
      fmt::print(fg(fmt::color::red), "[{}/E]:{}\n", tag_, msg);
    }

  protected:
    /**
     *   Name given to the logger to be prepended to all logs.
     */
    std::string tag_;

    /**
     *   Current verbosity of the logger. Determines what will be printed to
     *   console.
     */
    std::atomic<Verbosity> level_;
  };
}
