#include "cli.hpp"

bool espp::Cli::configured_ = false;
espp::Cli::console_handle_t espp::Cli::console_ = {nullptr, nullptr, nullptr};
