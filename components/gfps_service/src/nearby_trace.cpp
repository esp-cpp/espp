#include "gfps.hpp"

static espp::Logger logger({.tag = "embedded", .level = espp::Logger::Verbosity::WARN});

// Generates conditional trace line. This is usually wrapped in a macro to
// provide compiler parameters.
//
// level    - Debug level, higher is less messages.
// filename - Name of file calling trace.
// lineno   - Source line of trace call.
// fmt      - printf() style format string (%).
// ...      - A series of parameters indicated by the fmt string.
void nearby_platform_Trace(nearby_platform_TraceLevel level, const char *filename, int lineno,
                           const char *fmt, ...) {
  bool print = true;
  switch (level) {
  case kTraceLevelVerbose:
    fmt::print(fg(fmt::color::dark_gray), "[GFPS/V]{}:{}: ", filename, lineno);
    break;
  case kTraceLevelDebug:
    fmt::print(fg(fmt::color::gray), "[GFPS/D]{}:{}: ", filename, lineno);
    break;
  case kTraceLevelInfo:
    fmt::print(fg(fmt::terminal_color::green), "[GFPS/I]{}:{}: ", filename, lineno);
    break;
  case kTraceLevelWarning:
    fmt::print(fg(fmt::terminal_color::yellow), "[GFPS/W]{}:{}: ", filename, lineno);
    break;
  case kTraceLevelError:
    fmt::print(fg(fmt::terminal_color::red), "[GFPS/E]{}:{}: ", filename, lineno);
    break;
  default:
    logger.error("Unknown trace level: {}", (int)level);
    fmt::print(fg(fmt::terminal_color::red), "[GFPS/U]{}:{}: ", filename, lineno);
    print = false;
    break;
  }
  if (print) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
  }
}

// Processes assert. Processes a failed assertion.
//
// filename - Name of file calling assert.
// lineno   - Source line of assert call
// reason   - String message indicating reason for assert.
void nearby_platfrom_CrashOnAssert(const char *filename, int lineno, const char *reason) {
  logger.error("Assert failed: {}", reason);
  logger.error("File: {}, line: {}", filename, lineno);
  abort();
}

// Initializes trace module.
void nearby_platform_TraceInit(void) {}
