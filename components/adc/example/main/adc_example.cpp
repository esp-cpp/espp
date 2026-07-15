#include <chrono>
#include <vector>

#include "continuous_adc.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// The ADC unit and channels come from the "ADC Example Configuration" menu
// (menuconfig), which has selections for specific boards (setting them to
// pins which are usable on that board) as well as a custom option. Which GPIO
// a channel maps to is chip specific.
static constexpr adc_unit_t EXAMPLE_ADC_UNIT =
    (CONFIG_EXAMPLE_ADC_UNIT == 1) ? ADC_UNIT_1 : ADC_UNIT_2;
static constexpr adc_channel_t EXAMPLE_ADC_CHANNEL_1 =
    static_cast<adc_channel_t>(CONFIG_EXAMPLE_ADC_CHANNEL_1);
static constexpr adc_channel_t EXAMPLE_ADC_CHANNEL_2 =
    static_cast<adc_channel_t>(CONFIG_EXAMPLE_ADC_CHANNEL_2);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Adc Example", .level = espp::Logger::Verbosity::INFO});
  size_t num_seconds_to_run = 5;

  logger.info("Using ADC unit {} channels {} and {}", CONFIG_EXAMPLE_ADC_UNIT,
              (int)EXAMPLE_ADC_CHANNEL_1, (int)EXAMPLE_ADC_CHANNEL_2);

  // NOTE: the continuous section runs first: on ESP32-P4, initializing and
  // then deleting the oneshot ADC driver before using the continuous (DMA)
  // driver currently causes the continuous conversions to produce all-zero
  // samples (esp-idf issue; verified on hardware with IDF v6.0.1)
  {
    logger.info("Reading continuous adc for {} seconds", num_seconds_to_run);
    //! [continuous adc example]
    // NOTE: the unit must support continuous (DMA) mode; on ESP32 / C3 / S3
    // that is only unit 1
    std::vector<espp::AdcConfig> channels{{.unit = EXAMPLE_ADC_UNIT,
                                           .channel = EXAMPLE_ADC_CHANNEL_1,
                                           .attenuation = ADC_ATTEN_DB_12},
                                          {.unit = EXAMPLE_ADC_UNIT,
                                           .channel = EXAMPLE_ADC_CHANNEL_2,
                                           .attenuation = ADC_ATTEN_DB_12}};
    // this initializes the DMA and filter task for the continuous adc. The
    // conversion mode is derived automatically from the channel units; you
    // can also set it explicitly with e.g. .convert_mode =
    // ADC_CONV_SINGLE_UNIT_1 (chip-dependent: BOTH_UNIT, ALTER_UNIT,
    // SINGLE_UNIT_1, SINGLE_UNIT_2)
    espp::ContinuousAdc adc({.sample_rate_hz = 20 * 1000,
                             .channels = channels,
                             .window_size_bytes = 1024,
                             .log_level = espp::Logger::Verbosity::WARN});
    adc.start();
    auto task_fn = [&adc, &channels](std::mutex &m, std::condition_variable &cv) {
      for (auto &conf : channels) {
        auto maybe_mv = adc.get_mv(conf);
        if (maybe_mv.has_value()) {
          fmt::print("{}: {} mV\n", conf, maybe_mv.value());
        } else {
          fmt::print("{}: no value!\n", conf);
        }
        auto maybe_rate = adc.get_rate(conf);
        if (maybe_rate.has_value()) {
          fmt::print("{}: {} Hz\n", conf, maybe_rate.value());
        } else {
          fmt::print("{}: no rate!\n", conf);
        }
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Read ADC"},
                            .log_level = espp::Logger::Verbosity::INFO});
    task.start();

    // test stopping and starting the adc
    std::this_thread::sleep_for(3s);
    logger.info("Stopping ADC");
    adc.stop();
    std::this_thread::sleep_for(3s);
    logger.info("Starting ADC");
    adc.start();

    //! [continuous adc example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  {
    logger.info("Reading oneshot adc for {} seconds", num_seconds_to_run);
    //! [oneshot adc example]
    std::vector<espp::AdcConfig> channels{{.unit = EXAMPLE_ADC_UNIT,
                                           .channel = EXAMPLE_ADC_CHANNEL_1,
                                           .attenuation = ADC_ATTEN_DB_12},
                                          {.unit = EXAMPLE_ADC_UNIT,
                                           .channel = EXAMPLE_ADC_CHANNEL_2,
                                           .attenuation = ADC_ATTEN_DB_12}};
    espp::OneshotAdc adc({
        .unit = EXAMPLE_ADC_UNIT,
        .channels = channels,
    });
    auto task_fn = [&adc, &channels](std::mutex &m, std::condition_variable &cv) {
      static bool use_individual_functions = false;
      if (use_individual_functions) {
        // this iteration, we'll use the read_mv function for each channel
        for (auto &conf : channels) {
          auto maybe_mv = adc.read_mv(conf);
          if (maybe_mv.has_value()) {
            fmt::print("{}: {} mV\n", conf, maybe_mv.value());
          } else {
            fmt::print("{}: no value!\n", conf);
          }
        }
      } else {
        // this iteration, we'll use the read_all_mv function to read all
        // configured channels
        auto voltages = adc.read_all_mv();
        for (const auto &mv : voltages) {
          fmt::print("{} mV\n", mv);
        }
      }
      use_individual_functions = !use_individual_functions;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Oneshot ADC"},
                            .log_level = espp::Logger::Verbosity::INFO});
    task.start();
    //! [oneshot adc example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  logger.info("complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
