#include "logger.hpp"

using namespace espp;

std::chrono::steady_clock::time_point Logger::start_time_ = std::chrono::steady_clock::now();
