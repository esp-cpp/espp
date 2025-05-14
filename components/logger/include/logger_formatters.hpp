#pragma once

// libfmt printing for verbosity levels
template <> struct fmt::formatter<espp::Logger::Verbosity> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::Logger::Verbosity &v, FormatContext &ctx) const {
    switch (v) {
    case espp::Logger::Verbosity::DEBUG:
      return fmt::formatter<std::string>::format("DEBUG", ctx);
    case espp::Logger::Verbosity::INFO:
      return fmt::formatter<std::string>::format("INFO", ctx);
    case espp::Logger::Verbosity::WARN:
      return fmt::formatter<std::string>::format("WARN", ctx);
    case espp::Logger::Verbosity::ERROR:
      return fmt::formatter<std::string>::format("ERROR", ctx);
    case espp::Logger::Verbosity::NONE:
      return fmt::formatter<std::string>::format("NONE", ctx);
    }
    return ctx.out();
  }
};
