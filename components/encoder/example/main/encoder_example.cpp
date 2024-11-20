#include <chrono>
#include <vector>

#include "abi_encoder.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  size_t num_seconds_to_run = 10;

  {
    fmt::print("Reading Rotational ABI Encoder for {} seconds\n", num_seconds_to_run);
    //! [abi encoder rotational example]
    espp::AbiEncoder<espp::EncoderType::ROTATIONAL> encoder({
        .a_gpio = 9,
        .b_gpio = 10,
        .high_limit = 8192,
        .low_limit = -8192,
        .counts_per_revolution = 4096,
    });
    encoder.start();
    auto task_fn = [&encoder](std::mutex &m, std::condition_variable &cv) {
      auto count = encoder.get_count();
      auto radians = encoder.get_radians();
      auto degrees = encoder.get_degrees();
      fmt::print("CRD: [{}, {:.3f}, {:.1f}]\n", count, radians, degrees);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 250ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Abi Encoder"},
                            .log_level = espp::Logger::Verbosity::INFO});
    task.start();
    //! [abi encoder rotational example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  {
    fmt::print("Reading Linear ABI Encoder for {} seconds\n", num_seconds_to_run);
    //! [abi encoder linear example]
    espp::AbiEncoder<espp::EncoderType::LINEAR> encoder({
        .a_gpio = 9,
        .b_gpio = 10,
        .high_limit = 8192,
        .low_limit = -8192,
    });
    encoder.start();
    auto task_fn = [&encoder](std::mutex &m, std::condition_variable &cv) {
      // EncoderType::LINEAR only supports get_count
      auto count = encoder.get_count();
      fmt::print("Count: {}\n", count);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 250ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Abi Encoder"},
                            .log_level = espp::Logger::Verbosity::INFO});
    task.start();
    //! [abi encoder linear example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("Encoder example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
