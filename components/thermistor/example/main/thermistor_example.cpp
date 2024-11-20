#include <chrono>
#include <thread>
#include <vector>

#include "continuous_adc.hpp"
#include "logger.hpp"
#include "task.hpp"
#include "thermistor.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // create a logger for this example
  static espp::Logger logger({.tag = "Thermistor Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("starting");
  {
    //! [thermistor validation example]

    // create a lambda function for getting the voltage from the thermistor
    // this is just a simple voltage divider
    auto compute_voltage = [](float r1, float r2 = 10000, float v_in = 3300.0) -> float {
      return v_in * (r2 / (r1 + r2));
    };

    // create a table of datasheet info for the thermistor
    // this is used to calculate the temperature
    // the first value is the temperature in C
    // the second value is the resistance scale factor (R/R0)
    // This data is from the datasheet for the thermistor
    // https://product.tdk.com/system/files/dam/doc/product/sensor/ntc/chip-ntc-thermistor/data_sheet/50/db/ntc/ntc_smd_standard_series_0402.pdf
    // we chose a 10k NTC, which corresponds to the 8502 series column on
    // page 5 of the datasheet
    auto datasheet_info = std::vector<std::tuple<float, float>>{
        {0, 3.2657},  {10, 1.9907}, {20, 1.2494}, {25, 1},        {30, .80552},
        {40, .53229}, {50, .35981}, {75, .14763}, {100, .067488},
    };

    int i = 0;
    // create a lambda function for getting the voltage from the thermistor
    // using the datasheet info, this will return the voltage for the
    // current temperature (based on the index)
    auto get_voltage = [&compute_voltage, &i, &datasheet_info]() -> float {
      auto [temp, r_scale] = datasheet_info[i];
      return compute_voltage(r_scale * 10000);
    };

    // create a thermistor object
    espp::Thermistor thermistor({.divider_config = espp::Thermistor::ResistorDividerConfig::UPPER,
                                 .beta = 3940, // 25/50C beta since we're not planning on hot temps
                                 .nominal_resistance_ohms = 10000,
                                 .fixed_resistance_ohms = 10000,
                                 .supply_mv = 3300,
                                 .read_mv = get_voltage,
                                 .log_level = espp::Logger::Verbosity::INFO});

    // see how bad the error is
    for (i = 0; i < datasheet_info.size(); i++) {
      auto [actual_celsius, r_scale] = datasheet_info[i];
      auto actual_kelvin = actual_celsius + 273.15;
      // read the temperature
      auto kelvin = thermistor.get_kelvin();
      auto celsius = thermistor.get_celsius();
      // use kelvin for calculating error so we don't have to worry about division by zero
      auto error = std::abs(kelvin - actual_kelvin);
      logger.info("measured voltage: {:.2f} mV, calculated resistance: {:.2f} Ohm, calculated "
                  "temp: {:.2f} C, actual: {:.2f} C; error: {:.2f} C",
                  get_voltage(), thermistor.get_resistance(), celsius, actual_celsius, error);
    }
    //! [thermistor validation example]
  }

  //! [thermistor adc example]

  // create a continuous ADC which will sample and filter the thermistor
  // voltage on ADC1 channel 7
  std::vector<espp::AdcConfig> channels{
      {.unit = ADC_UNIT_1, .channel = ADC_CHANNEL_7, .attenuation = ADC_ATTEN_DB_11}};
  // this initailizes the DMA and filter task for the continuous adc
  espp::ContinuousAdc adc(
      {.sample_rate_hz = 20 * 1000,
       .channels = channels,
       .convert_mode = ADC_CONV_SINGLE_UNIT_1, // or BOTH_UNIT, ALTER_UNIT, SINGLE_UNIT_1
       .window_size_bytes = 1024,
       .log_level = espp::Logger::Verbosity::WARN});
  adc.start();

  // make a lambda function for getting the latest voltage from the adc
  auto get_voltage = [&adc, &channels]() -> float {
    auto maybe_mv = adc.get_mv(channels[0]);
    if (maybe_mv.has_value()) {
      return maybe_mv.value();
    } else {
      return 0;
    }
  };

  // create a thermistor object (based on the datasheet from
  // https://product.tdk.com/system/files/dam/doc/product/sensor/ntc/chip-ntc-thermistor/catalog/tpd_commercial_ntc-thermistor_ntcg_en.pdf
  // From the table (page 7):
  // clang-format off
  // Part Number.    | R25(Ω) | Tolerance | B25/50(Κ) | B25/75(Κ) | B25/85(Κ) | B25/100(K) | Current (mA) | Operating Temp Range
  // NTCG103JF103FT1 | 10,000 |   +/–1%   | 3380      | 3422      | 3435      | 3453 +/–1% | 0.31         | –40 to 152
  // clang-format on
  espp::Thermistor thermistor({.divider_config = espp::Thermistor::ResistorDividerConfig::UPPER,
                               .beta = 3380,
                               .nominal_resistance_ohms = 10000,
                               .fixed_resistance_ohms = 10000,
                               .supply_mv = 3300,
                               .read_mv = get_voltage,
                               .log_level = espp::Logger::Verbosity::INFO});

  fmt::print("% time(s), temp(C)\n");
  auto task_fn = [&thermistor](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();

    // read the temperature
    auto temp = thermistor.get_celsius();

    // print out the temperature
    fmt::print("{:.3f}, {}\n", elapsed, temp);

    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, now + 10ms);
    }
    // don't want to stop the task
    return false;
  };
  auto task = espp::Task({.callback = task_fn,
                          .task_config = {.name = "Read Thermistor"},
                          .log_level = espp::Logger::Verbosity::INFO});
  task.start();
  //! [thermistor adc example]

  // wait forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
