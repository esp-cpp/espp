#pragma once

#include "format.hpp"

// for printing of BaseConfig using libfmt
template <> struct fmt::formatter<espp::Task::BaseConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Task::BaseConfig &config, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(),
        "Task::BaseConfig{{name: '{}', stack_size_bytes: {}, priority: {}, core_id: {}}}",
        config.name, config.stack_size_bytes, config.priority, config.core_id);
  }
};

// for printing of Task::Config using libfmt
template <> struct fmt::formatter<espp::Task::Config> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Task::Config &config, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "Task::Config{{callback: '{}', task_config: {}, log_level: {}}}",
                          config.callback, config.task_config, config.log_level);
  }
};
