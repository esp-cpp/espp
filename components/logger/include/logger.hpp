#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <string_view>

#if defined(ESP_PLATFORM)
#include <esp_timer.h>
#include <sdkconfig.h>
#endif

#include "format.hpp"

// Undefine the logger verbosity levels to avoid conflicts with windows / msvc
#ifdef _MSC_VER
#undef ERROR
#undef WARN
#undef INFO
#undef DEBUG
#endif

namespace espp {

/**
 * @brief Logger provides a wrapper around nicer / more robust formatting than
 * standard ESP_LOG* macros with the ability to change the log level at
 * run-time. Logger currently is a light wrapper around libfmt (future
 * std::format).
 *
 * To save on code size, the logger has the ability to be compiled out based on
 * the log level set in the sdkconfig. This means that if the log level is set to
 * ERROR, all debug, info, and warn logs will be compiled out. This is done by
 * checking the log level at compile time and only compiling in the functions
 * that are needed.
 *
 * The logger can also be compiled with support for cursor commands. This allows
 * the logger to move the cursor up, down, clear the line, clear the screen, and
 * move the cursor to a specific position. This can be useful for creating
 * various types of interactive output or to maintian context with long-running
 * logs.
 *
 * \section logger_ex1 Basic Example
 * \snippet logger_example.cpp Logger example
 * \section logger_ex2 Threaded Logging and Verbosity Example
 * \snippet logger_example.cpp MultiLogger example
 * \section logger_ex3 Cursor Commands Example
 * \snippet logger_example.cpp Cursor Commands example
 */
class Logger {

#define ESPP_LOGGER_LOG_LEVEL_NONE 0
#define ESPP_LOGGER_LOG_LEVEL_ERROR 1
#define ESPP_LOGGER_LOG_LEVEL_WARN 2
#define ESPP_LOGGER_LOG_LEVEL_INFO 3
#define ESPP_LOGGER_LOG_LEVEL_DEBUG 4

#ifndef CONFIG_ESPP_LOGGER_LOG_LEVEL
#define CONFIG_ESPP_LOGGER_LOG_LEVEL ESPP_LOGGER_LOG_LEVEL_DEBUG
#endif

#define ESPP_LOGGER_DEBUG_ENABLED (CONFIG_ESPP_LOGGER_LOG_LEVEL >= ESPP_LOGGER_LOG_LEVEL_DEBUG)
#define ESPP_LOGGER_INFO_ENABLED (CONFIG_ESPP_LOGGER_LOG_LEVEL >= ESPP_LOGGER_LOG_LEVEL_INFO)
#define ESPP_LOGGER_WARN_ENABLED (CONFIG_ESPP_LOGGER_LOG_LEVEL >= ESPP_LOGGER_LOG_LEVEL_WARN)
#define ESPP_LOGGER_ERROR_ENABLED (CONFIG_ESPP_LOGGER_LOG_LEVEL >= ESPP_LOGGER_LOG_LEVEL_ERROR)

// If the cursor commands haven't been configured as a define, check the config
#if !defined(ESPP_LOGGER_CURSOR_COMMANDS_ENABLED)
#if !defined(CONFIG_ESPP_LOGGER_ENABLE_CURSOR_COMMANDS)
#define ESPP_LOGGER_CURSOR_COMMANDS_ENABLED 0
#else // CONFIG_ESPP_LOGGER_ENABLE_CURSOR_COMMANDS
#define ESPP_LOGGER_CURSOR_COMMANDS_ENABLED 1
#endif // CONFIG_ESPP_LOGGER_ENABLE_CURSOR_COMMANDS
#endif // ESPP_LOGGER_CURSOR_COMMANDS_ENABLED

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
    std::string_view tag;    /**< The TAG that will be prepended to all logs. */
    bool include_time{true}; /**< Include the time in the log. */
    std::chrono::duration<float> rate_limit =
        std::chrono::duration<float>(0); /**< The rate limit for the logger. Optional, if <= 0 no
rate limit. @note Only calls that have _rate_limited suffixed will be rate limited. */
    espp::Logger::Verbosity level =
        espp::Logger::Verbosity::WARN; /**< The verbosity level for the logger. */
  };

  /**
   * @brief Construct a new Logger object
   *
   * @param config configuration for the logger.
   */
  explicit Logger(const Config &config)
      : tag_(config.tag)
      , rate_limit_(config.rate_limit)
      , include_time_(config.include_time)
      , level_(config.level) {}

  /**
   * @brief Copy constructor
   * @param other The other logger to copy from.
   * @note This will NOT copy the last_print_ time, as this is not meaningful
   *       for the new logger.
   */
  Logger(const Logger &other) // cppcheck-suppress missingMemberCopy
      : tag_(other.get_tag())
      , rate_limit_(other.rate_limit_)
      , include_time_(other.include_time_.load())
      , level_(other.level_.load()) {}

  /**
   * @brief Move constructor
   * @param other The other logger to move from.
   */
  Logger(Logger &&other) noexcept // cppcheck-suppress missingMemberCopy
      : tag_([&other] {
        std::lock_guard<std::mutex> lock(other.tag_mutex_);
        return std::move(other.tag_);
      }())
      , rate_limit_(std::move(other.rate_limit_))
      , last_print_(std::move(other.last_print_))
      , include_time_(other.include_time_.load())
      , level_(other.level_.load()) {}

  /**
   * @brief Get the current verbosity for the logger.
   * @return The current verbosity level.
   * \sa Logger::Verbosity
   *
   */
  espp::Logger::Verbosity get_verbosity() const { return level_; }

  /**
   * @brief Change the verbosity for the logger. \sa Logger::Verbosity
   * @param level new verbosity level
   */
  void set_verbosity(const espp::Logger::Verbosity level) { level_ = level; }

  /**
   * @brief Change the tag for the logger.
   * @param tag The new tag.
   */
  void set_tag(const std::string_view tag) {
    std::lock_guard<std::mutex> lock(tag_mutex_);
    tag_ = tag;
  }

  /**
   * @brief Get the current tag for the logger.
   * @return A const reference to the current tag.
   */
  const std::string &get_tag() const { return tag_; }

  /**
   * @brief Whether to include the time in the log.
   * @param include_time Whether to include the time in the log.
   * @note The time is in seconds since boot and is represented as a floating
   *       point number with precision to the millisecond.
   */
  void set_include_time(bool include_time) { include_time_ = include_time; }

  /**
   * @brief Change the rate limit for the logger.
   * @param rate_limit The new rate limit.
   * @note Only calls that have _rate_limited suffixed will be rate limited.
   */
  void set_rate_limit(const std::chrono::duration<float> rate_limit) { rate_limit_ = rate_limit; }

  /**
   * @brief Get the current rate limit for the logger.
   * @return The current rate limit.
   */
  std::chrono::duration<float> get_rate_limit() const { return rate_limit_; }

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
#if ESPP_LOGGER_DEBUG_ENABLED
    if (level_ > espp::Logger::Verbosity::DEBUG)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    if (include_time_) {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::color::gray), "[{}/D][{}]: {}\n", tag_, get_time(), msg);
    } else {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::color::gray), "[{}/D]:{}\n", tag_, msg);
    }
#endif
  }

  /**
   * @brief Print log in GREEN if level is Verbosity::INFO or greater
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void info(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_INFO_ENABLED
    if (level_ > espp::Logger::Verbosity::INFO)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    if (include_time_) {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::terminal_color::green), "[{}/I][{}]: {}\n", tag_, get_time(), msg);
    } else {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::terminal_color::green), "[{}/I]:{}\n", tag_, msg);
    }
#endif
  }

  /**
   * @brief Print log in YELLOW if level is Verbosity::WARN or greater
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void warn(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_WARN_ENABLED
    if (level_ > espp::Logger::Verbosity::WARN)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    if (include_time_) {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::terminal_color::yellow), "[{}/W][{}]: {}\n", tag_, get_time(), msg);
    } else {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::terminal_color::yellow), "[{}/W]:{}\n", tag_, msg);
    }
#endif
  }

  /**
   * @brief Print log in RED if level is Verbosity::ERROR or greater
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void error(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_ERROR_ENABLED
    if (level_ > espp::Logger::Verbosity::ERROR)
      return;
    auto msg = format(rt_fmt_str, std::forward<Args>(args)...);
    if (include_time_) {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::terminal_color::red), "[{}/E][{}]: {}\n", tag_, get_time(), msg);
    } else {
      std::lock_guard<std::mutex> lock(tag_mutex_);
      fmt::print(fg(fmt::terminal_color::red), "[{}/E]:{}\n", tag_, msg);
    }
#endif
  }

  /**
   * @brief Print log in GRAY if level is Verbosity::DEBUG or greater.
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void debug_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_DEBUG_ENABLED
    if (level_ > espp::Logger::Verbosity::DEBUG)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    // forward the arguments to the debug function
    debug(rt_fmt_str, std::forward<Args>(args)...);
#endif
  }

  /**
   * @brief Print log in GREEN if level is Verbosity::INFO or greater
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void info_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_INFO_ENABLED
    if (level_ > espp::Logger::Verbosity::INFO)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    // forward the arguments to the info function
    info(rt_fmt_str, std::forward<Args>(args)...);
#endif
  }

  /**
   * @brief Print log in YELLOW if level is Verbosity::WARN or greater
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void warn_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_WARN_ENABLED
    if (level_ > espp::Logger::Verbosity::WARN)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    // forward the arguments to the warn function
    warn(rt_fmt_str, std::forward<Args>(args)...);
#endif
  }

  /**
   * @brief Print log in RED if level is Verbosity::ERROR or greater
   *        This function is rate limited by the rate specified in the
   *        constructor.
   * @param rt_fmt_str format string
   * @param args optional arguments passed to be formatted.
   */
  template <typename... Args> void error_rate_limited(std::string_view rt_fmt_str, Args &&...args) {
#if ESPP_LOGGER_ERROR_ENABLED
    if (level_ > espp::Logger::Verbosity::ERROR)
      return;
    if (rate_limit_ > std::chrono::duration<float>::zero()) {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_print_ < rate_limit_)
        return;
      last_print_ = now;
    }
    // forward the arguments to the error function
    error(rt_fmt_str, std::forward<Args>(args)...);
#endif
  }

#if ESPP_LOGGER_CURSOR_COMMANDS_ENABLED || defined(_DOXYGEN_)
  /**
   * @brief Move the cursor up one line.
   */
  static void move_up() { fmt::print("\033[A"); }

  /**
   * @brief Move the cursor up a number of lines.
   * @param lines The number of lines to move up.
   */
  static void move_up(int lines) { fmt::print("\033[{}A", lines); }

  /**
   * @brief Move the cursor down one line.
   */
  static void move_down() { fmt::print("\033[B"); }

  /**
   * @brief Move the cursor down a number of lines.
   * @param lines The number of lines to move down.
   */
  static void move_down(int lines) { fmt::print("\033[{}B", lines); }

  /**
   * @brief Move the cursor to the beginning of the line.
   */
  static void move_to_start() { fmt::print("\r"); }

  /**
   * @brief Clear the line.
   */
  static void clear_line() { fmt::print("\033[K"); }

  /**
   * @brief Clear the screen and move the cursor to the top left.
   */
  static void clear_screen() { fmt::print("\033[2J\033[H"); }

  /**
   * @brief Move the cursor to a specific position.
   * @param x The x position to move to.
   * @param y The y position to move to.
   */
  static void move_to(int x, int y) { fmt::print("\033[{};{}H", y, x); }
#endif

  /**
   *   Get the current time in seconds since the start of the logging system.
   *   @return time in seconds since the start of the logging system.
   */
  static std::string get_time() {
#if defined(ESP_PLATFORM)
    // use esp_timer_get_time to get the time in microseconds
    uint64_t time = esp_timer_get_time();
    uint64_t seconds = time / 1e6f;
    uint64_t milliseconds = (time % 1000000) / 1e3f;
    return fmt::format("{}.{:03}", seconds, milliseconds);
#else
    // get the elapsed time since the start of the logging system as floating
    // point seconds
    auto now = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration<float>(now - start_time_).count();
    return fmt::format("{:.3f}", seconds);
#endif
  }

protected:
  /**
   *   Start time for the logging system.
   */
  static std::chrono::steady_clock::time_point start_time_;

  std::mutex tag_mutex_; ///< Mutex for the tag.
  std::string tag_;      ///< Name of the logger to be prepended to all logs.
  std::chrono::duration<float> rate_limit_{
      0.0f}; ///< Rate limit for the logger. If set to 0, no rate limiting will be performed.
  std::chrono::high_resolution_clock::time_point
      last_print_{};                     ///< Last time a log was printed. Used for rate limiting.
  std::atomic<bool> include_time_{true}; ///< Whether to include the time in the log.
  std::atomic<espp::Logger::Verbosity> level_ =
      espp::Logger::Verbosity::WARN; ///< Current verbosity level of the logger.
};
} // namespace espp

#include "logger_formatters.hpp"
