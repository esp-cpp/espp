#pragma once

#include <atomic>
#include <chrono>
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
 * \section logger_ex1 Basic Example
 * \snippet logger_example.cpp Logger example
 * \section logger_ex2 Threaded Logging and Verbosity Example
 * \snippet logger_example.cpp MultiLogger example
 */
class Logger {
public:
  /**
   *   Verbosity levels for the logger, in order of increasing priority.
   */
  enum class Verbosity {
    DEBUG, /**< Debug level verbosity. */
    INFO,  /**< Info level verbosity. */
    WARN,  /**< Warn level verbosity. */
    ERROR, /**< Error level verbosity. */
    NONE,  /**< No verbosity - logger will not print anything. */
  };

  /**
   * @brief Configuration struct for the logger.
   */
  struct Config {
    std::string_view tag; /**< The TAG that will be prepended to all logs. */
    std::chrono::duration<float> rate_limit{
        0}; /**< The rate limit for the logger. Optional, if <= 0 no rate limit. @note Only calls
               that have _rate_limited suffixed will be rate limited. */
    Verbosity level = Verbosity::WARN; /**< The verbosity level for the logger. */
  };

  /**
   * @brief Construct a new Logger object
   *
   * @param config configuration for the logger.
   */
  Logger(const Config &config)
      : tag_(config.tag), rate_limit_(config.rate_limit), level_(config.level) {}

  /**
   * @brief Change the verbosity for the logger. \sa Logger::Verbosity
   * @param level new verbosity level
   */
  void set_verbosity(const Verbosity level) { level_ = level; }

  /**
   * @brief Change the tag for the logger.
   * @param tag The new tag.
   */
  void set_tag(const std::string_view tag) {
    std::lock_guard<std::mutex> lock(tag_mutex_);
    tag_ = tag;
  }

  /**
   * @brief Format args into string according to format string. From:
   * https://en.cppreference.com/w/cpp/utility/format/format
   *
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   * @return formatted std::string
   */
  template <typename... Args> std::string format(std::string_view rt_fmt_str, Args &&...args) {
    return fmt::vformat(rt_fmt_str, fmt::make_format_args(args...));
  }

  /**
   * @brief Print log in GRAY if level is Verbosity::DEBUG or greater.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void debug(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::DEBUG)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::color::gray), "[{}/D]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in GREEN if level is Verbosity::INFO or greater
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void info(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::INFO)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::terminal_color::green), "[{}/I]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in YELLOW if level is Verbosity::WARN or greater
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void warn(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::WARN)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::terminal_color::yellow), "[{}/W]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in RED if level is Verbosity::ERROR or greater
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void error(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::ERROR)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::terminal_color::red), "[{}/E]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in GRAY if level is Verbosity::DEBUG or greater.
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void debug_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::DEBUG)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::color::gray), "[{}/D]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in GREEN if level is Verbosity::INFO or greater
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void info_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::INFO)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::terminal_color::green), "[{}/I]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in YELLOW if level is Verbosity::WARN or greater
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void warn_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::WARN)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::terminal_color::yellow), "[{}/W]:{}\n", tag_, msg);
  }

  /**
   * @brief Print log in RED if level is Verbosity::ERROR or greater
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void error_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
    if (level_ > Verbosity::ERROR)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(tag_mutex_);
    fmt::print(fg(fmt::terminal_color::red), "[{}/E]:{}\n", tag_, msg);
  }

protected:
  std::mutex tag_mutex_;

  /**
   *   Name given to the logger to be prepended to all logs.
   */
  std::string tag_;

  /**
   *   Rate limit for the logger. If set to 0, no rate limiting will be
   *   performed.
   */
  std::chrono::duration<float> rate_limit_{0.0f};

  /**
   *   Last time a log was printed. Used for rate limiting.
   */
  std::chrono::high_resolution_clock::time_point last_print_{};

  /**
   *   Current verbosity of the logger. Determines what will be printed to
   *   console.
   */
  std::atomic<Verbosity> level_;
};
} // namespace espp
