#pragma once

#include <chrono>
#include <string>

#include "logger.hpp"

namespace espp {
/// Base class for all components
/// Provides a logger and some basic logging configuration
class BaseComponent {
public:
  /// Get the name of the component
  /// \return A const reference to the name of the component
  /// \note This is the tag of the logger
  const std::string &get_name() const { return logger_.get_tag(); }

  /// Set the tag for the logger
  /// \param tag The tag to use for the logger
  void set_log_tag(const std::string_view &tag) { logger_.set_tag(tag); }

  /// Get the log level for the logger
  /// \return The verbosity level of the logger
  /// \sa Logger::Verbosity
  /// \sa Logger::set_verbosity
  espp::Logger::Verbosity get_log_level() const { return logger_.get_verbosity(); }

  /// Set the log level for the logger
  /// \param level The verbosity level to use for the logger
  /// \sa Logger::Verbosity
  /// \sa Logger::set_verbosity
  void set_log_level(espp::Logger::Verbosity level) { logger_.set_verbosity(level); }

  /// Set the log verbosity for the logger
  /// \param level The verbosity level to use for the logger
  /// \note This is a convenience method that calls set_log_level
  /// \sa set_log_level
  /// \sa Logger::Verbosity
  /// \sa Logger::set_verbosity
  void set_log_verbosity(espp::Logger::Verbosity level) { set_log_level(level); }

  /// Get the log verbosity for the logger
  /// \return The verbosity level of the logger
  /// \note This is a convenience method that calls get_log_level
  /// \sa get_log_level
  /// \sa Logger::Verbosity
  /// \sa Logger::get_verbosity
  espp::Logger::Verbosity get_log_verbosity() const { return get_log_level(); }

  /// Set the rate limit for the logger
  /// \param rate_limit The rate limit to use for the logger
  /// \note Only calls to the logger that have _rate_limit suffix will be rate limited
  /// \sa Logger::set_rate_limit
  void set_log_rate_limit(std::chrono::duration<float> rate_limit) {
    logger_.set_rate_limit(rate_limit);
  }

protected:
  BaseComponent() = default;

  explicit BaseComponent(std::string_view tag,
                         espp::Logger::Verbosity level = espp::Logger::Verbosity::WARN)
      : logger_({.tag = tag, .level = level}) {}

  explicit BaseComponent(const espp::Logger::Config &logger_config)
      : logger_(logger_config) {}

  /// The logger for this component
  Logger logger_ = espp::Logger({.tag = "BaseComponent", .level = espp::Logger::Verbosity::INFO});
};
} // namespace espp
