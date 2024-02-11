#pragma once

#include <chrono>
#include <string>

#include "logger.hpp"

namespace espp {
/// Base class for all components
/// Provides a logger and some basic logging configuration
class BaseComponent {
public:
  /// Set the tag for the logger
  /// \param tag The tag to use for the logger
  void set_log_tag(const std::string_view &tag) { logger_.set_tag(tag); }

  /// Set the log level for the logger
  /// \param level The verbosity level to use for the logger
  /// \sa Logger::Verbosity
  /// \sa Logger::set_verbosity
  void set_log_level(Logger::Verbosity level) { logger_.set_verbosity(level); }

  /// Set the log verbosity for the logger
  /// \param level The verbosity level to use for the logger
  /// \note This is a convenience method that calls set_log_level
  /// \sa set_log_level
  /// \sa Logger::Verbosity
  /// \sa Logger::set_verbosity
  void set_log_verbosity(Logger::Verbosity level) { set_log_level(level); }

  /// Set the rate limit for the logger
  /// \param rate_limit The rate limit to use for the logger
  /// \note Only calls to the logger that have _rate_limit suffix will be rate limited
  /// \sa Logger::set_rate_limit
  void set_log_rate_limit(std::chrono::duration<float> rate_limit) {
    logger_.set_rate_limit(rate_limit);
  }

protected:
  BaseComponent() = default;

  BaseComponent(std::string_view tag, Logger::Verbosity level = Logger::Verbosity::WARN)
      : logger_({.tag = tag, .level = level}) {}

  BaseComponent(const Logger::Config &logger_config)
      : logger_(logger_config) {}

  /// The logger for this component
  Logger logger_ = espp::Logger({.tag = "BaseComponent", .level = Logger::Verbosity::INFO});
};
} // namespace espp
