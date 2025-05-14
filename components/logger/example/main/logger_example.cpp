#include <chrono>
#include <functional>
#include <thread>

#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // we get access to fmt since we include logger.hpp
  fmt::print("Stating logger example!\n");
  {
    //! [Logger example]
    float num_seconds_to_run = 10.0f;
    // create loggers
    auto logger = espp::Logger({.tag = "Cool Logger", .level = espp::Logger::Verbosity::DEBUG});
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    while (elapsed < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
      auto remaining = num_seconds_to_run - elapsed;
      logger.debug("debug: {:%Y-%m-%d %H:%M:%S} - {:%Y-%m-%d %H:%M:%S} = {}", now, start, elapsed);
      logger.info("elapsed: {:.3f}s", elapsed);
      logger.warn("remaining: {:.3f}s", remaining);
      if (remaining < 0) {
        logger.error("You overstayed your welcome by {:.03}s!", -remaining);
      }
      std::this_thread::sleep_for(500ms);
    }
    //! [Logger example]
  }

  {
    //! [logger copy/move example]
    // create loggers
    std::vector<espp::Logger> loggers;
    for (int i = 0; i < 5; i++) {
      loggers.emplace_back(espp::Logger(
          {.tag = fmt::format("Logger {}", i),
           .include_time = i % 2 == 0,
           .level = i % 2 == 0 ? espp::Logger::Verbosity::DEBUG : espp::Logger::Verbosity::INFO}));
    }
    // copy loggers
    std::vector<espp::Logger> loggers_copy(loggers);
    // move loggers
    std::vector<espp::Logger> loggers_move(std::move(loggers));
    // use the loggers and make sure they are valid
    for (auto &logger : loggers_copy) {
      logger.info("Logger copy: {}", logger.get_verbosity());
    }
    for (auto &logger : loggers_move) {
      logger.info("Logger move: {}", logger.get_verbosity());
    }
    // loggers_copy and loggers_move are valid, but loggers is not
    //! [logger copy/move example]
  }

  {
    //! [Cursor Commands example]
    float num_seconds_to_run = 10.0f;
    // create loggers
    auto logger = espp::Logger({.tag = "Cursor Commands", .level = espp::Logger::Verbosity::DEBUG});
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    logger.info("Long running log... {}", elapsed);
    while (elapsed < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
      auto remaining = num_seconds_to_run - elapsed;
      logger.move_up();
      logger.clear_line();
      logger.info("Long running log... {}", remaining);
      std::this_thread::sleep_for(100ms);
    }
    //! [Cursor Commands example]
  }

  {
    //! [MultiLogger example]
    // create loggers
    auto logger1 = espp::Logger(
        {.tag = "Thread 1", .rate_limit = 500ms, .level = espp::Logger::Verbosity::INFO});
    auto logger2 = espp::Logger(
        {.tag = "Thread 2", .rate_limit = 1s, .level = espp::Logger::Verbosity::DEBUG});
    // lambda for logging to those two loggers from multiple threads
    auto logger_fn = [](espp::Logger *logger) {
      size_t loop_iteration{0};
      while (true) {
        // log - note: debug shouldn't be shown!
        logger->debug("some debug info: {}", loop_iteration);
        logger->info("some info: {}", loop_iteration);
        logger->warn("some warning: {}", loop_iteration);
        logger->error("some error: {}", loop_iteration);
        logger->info_rate_limited("some rate limited info: {}", loop_iteration);
        // update loop variables
        loop_iteration++;
        // sleep
        std::this_thread::sleep_for(300ms);
      }
    };
    // start two threads, binding the lambda to each logger
    auto logger1_thread = std::thread(std::bind(logger_fn, &logger1));
    auto logger2_thread = std::thread(std::bind(logger_fn, &logger2));
    uint8_t level{static_cast<uint8_t>(espp::Logger::Verbosity::DEBUG)};
    // every 1 second, change the loggers' verbosity
    while (true) {
      // update the loggers' verbosity
      level++;
      if (level > static_cast<uint8_t>(espp::Logger::Verbosity::NONE)) {
        level = static_cast<uint8_t>(espp::Logger::Verbosity::DEBUG);
      }
      espp::Logger::Verbosity verbosity = static_cast<espp::Logger::Verbosity>(level);
      logger1.set_verbosity(verbosity);
      logger2.set_verbosity(verbosity);
      // sleep
      std::this_thread::sleep_for(1s);
    }
    //! [MultiLogger example]
  }
}
