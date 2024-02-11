#pragma once

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
  void set_log_level(Logger::Verbosity level) { logger_.set_verbosity(level); }

  /// Set the log verbosity for the logger
  /// \param level The verbosity level to use for the logger
  /// \note This is a convenience method that calls set_log_level
  /// \sa set_log_level
  void set_log_verbosity(Logger::Verbosity level) { set_log_level(level); }

protected:
  BaseComponent() = default;

  BaseComponent(std::string_view tag, Logger::Verbosity level)
      : logger_({.tag = tag, .level = level}) {}

  /// The logger for this component
  Logger logger_ = espp::Logger({.tag = "BaseComponent", .level = Logger::Verbosity::INFO});
};
} // namespace espp
