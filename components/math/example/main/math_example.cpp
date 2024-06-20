#include <chrono>
#include <thread>
#include <vector>

#include "bezier.hpp"
#include "fast_math.hpp"
#include "gaussian.hpp"
#include "logger.hpp"
#include "range_mapper.hpp"
#include "vector2d.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "math_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("=== bezier ===");
  {
    //! [bezier example]
    using Bezier = espp::Bezier<espp::Vector2f>;
    std::array<espp::Vector2f, 4> control_points = {espp::Vector2f(6, 220), espp::Vector2f(62, 115),
                                                    espp::Vector2f(176, 151),
                                                    espp::Vector2f(217, 50)};
    Bezier bezier(Bezier::Config{.control_points = control_points});
    std::array<float, 4> weights = {0.5f, 2.0f, 0.5f, 2.0f};
    Bezier rational_bezier(
        Bezier::WeightedConfig{.control_points = control_points, .weights = weights});
    // NOTE: this is the built-in log format for pyqtgraph
    fmt::print("\"bezier_x\",\"bezier_y\",\"rational bezier_x\",\"rational bezier_y\"\n");
    float t = 0;
    while (t < 1.0f) {
      auto p0 = bezier(t);
      auto p1 = rational_bezier(t);
      fmt::print("{},{},{},{}\n", p0.x(), p0.y(), p1.x(), p1.y());
      t += 0.05f;
    }
    //! [bezier example]
  }

  logger.info("=== fast math ===");
  {
    //! [fast_ln example]
    float x = 3.0f;
    auto slow = log(x);
    auto fast = espp::fast_ln(x);
    auto diff = std::abs(slow - fast);
    fmt::print("ln({}) = {} (slow), {} (fast), diff = {}\n", x, slow, fast, diff);
    //! [fast_ln example]
  }
  {
    //! [fast_sqrt example]
    float x = 3.0f;
    auto slow = sqrt(x);
    auto fast = espp::fast_sqrt(x);
    auto diff = std::abs(slow - fast);
    fmt::print("sqrt({}) = {} (slow), {} (fast), diff = {}\n", x, slow, fast, diff);
    //! [fast_sqrt example]
  }
  {
    //! [fast_sin example]
    float x = 3.0f;
    auto slow = sin(x);
    auto fast = espp::fast_sin(x);
    auto diff = std::abs(slow - fast);
    fmt::print("sin({}) = {} (slow), {} (fast), diff = {}\n", x, slow, fast, diff);
    //! [fast_sin example]
  }
  {
    //! [fast_cos example]
    float x = 3.0f;
    auto slow = cos(x);
    auto fast = espp::fast_cos(x);
    auto diff = std::abs(slow - fast);
    fmt::print("cos({}) = {} (slow), {} (fast), diff = {}\n", x, slow, fast, diff);
    //! [fast_cos example]
  }

  logger.info("=== gaussian ===");
  {
    //! [gaussian example]
    std::array<float, 4> gammas = {
        0.10f,
        0.15f,
        0.20f,
        0.25f,
    };
    espp::Gaussian gaussian({
        .gamma = gammas[0],
        .alpha = 1.0f, // default
        .beta = 0.5f,  // default
    });
    float t = 0;
    fmt::print("% t");
    for (auto g : gammas) {
      fmt::print(", gaussian({})", g);
    }
    fmt::print("\n");
    float increment = 0.05f;
    int num_increments = 1.0f / increment;
    for (int i = 0; i <= num_increments; i++) {
      fmt::print("{}", t);
      for (auto g : gammas) {
        // update the gamma
        gaussian.gamma(g);
        // evaluate it
        float v = gaussian(t);
        // print it
        fmt::print(", {}", v);
      }
      fmt::print("\n");
      t += increment;
    }
    //! [gaussian example]
  }

  logger.info("=== range mapper ===");
  {
    //! [range_mapper example]
    static constexpr float deadband = 12.0f;
    static constexpr float min = 0.0f;
    static constexpr float center = 127.0f;
    static constexpr float max = 255.0f;
    // Default will have output range [-1, 1]
    espp::RangeMapper<float> rm(
        {.center = center, .center_deadband = deadband, .minimum = min, .maximum = max});
    // You can explicitly set output center/range. In this case the output will
    // be in the range [0, 1024]
    espp::RangeMapper<float> rm2({.center = center,
                                  .center_deadband = deadband,
                                  .minimum = min,
                                  .maximum = max,
                                  .output_center = 512,
                                  .output_range = 512});
    // You can also use a non-centered input distribution.
    espp::FloatRangeMapper rm3({.center = center / 2,
                                .center_deadband = deadband,
                                .minimum = min,
                                .maximum = max,
                                .output_center = 512,
                                .output_range = 512});
    // You can even invert the ouput distribution, and add a deadband around the
    // min/max values
    espp::FloatRangeMapper rm4({
        .center = center,
        .center_deadband = deadband,
        .minimum = min,
        .maximum = max,
        .range_deadband = deadband,
        .invert_output = true,
    });
    // make a vector of float values min - 10 to max + 10 in increments of 5
    std::vector<float> vals;
    for (float v = min - 10; v <= max + 10; v += 5) {
      vals.push_back(v);
    }
    // test the mapping and unmapping
    fmt::print(
        "% value, mapped [0;255] to [-1;1], unmapped [-1;1] to [0;255], mapped [0;255] to "
        "[0;1024], unmapped [0;1024] to [0;255], mapped [0;255] to [1024;0], unmapped [1024;0] to "
        "[0;255], mapped [0;255] to inverted [1;-1], unmapped inverted [1;-1] to [0;255]\n");
    for (const auto &v : vals) {
      fmt::print("{}, {}, {}, {}, {}, {}, {}, {}, {}\n", v, rm.map(v), rm.unmap(rm.map(v)),
                 rm2.map(v), rm2.unmap(rm2.map(v)), rm3.map(v), rm3.unmap(rm3.map(v)), rm4.map(v),
                 rm4.unmap(rm4.map(v)));
    }
    //! [range_mapper example]
  }

  logger.info("=== vector2d ===");
  {
    //! [vector2d example]
    fmt::print("--- uint8_t vector ---\n");
    espp::Vector2u8 v8(2, 3);
    fmt::print("original:       {}\n", v8);
    auto v8_2 = v8;
    v8 = uint8_t(2) * v8; // NOTE: need explicit cast to avoid ambiguity with auto type
    v8 /= 2;
    v8 += espp::Vector2u8(0, 1);
    v8 -= espp::Vector2u8(0, 1);
    fmt::print("should be same: {}, {}\n", v8, v8_2);
    // for good measure, print all the comparisons
    fmt::print("       v == v2: {}\n", v8 == v8_2);
    fmt::print("       v != v2: {}\n", v8 != v8_2);
    fmt::print("       v <  v2: {}\n", v8 < v8_2);
    fmt::print("       v <= v2: {}\n", v8 <= v8_2);
    fmt::print("       v >  v2: {}\n", v8 > v8_2);
    fmt::print("       v >= v2: {}\n", v8 >= v8_2);
    fmt::print("magnitude:      {}\n", v8.magnitude());
    fmt::print("normalized:     {}\n", v8.normalized());

    fmt::print("--- float vector ---\n");
    espp::Vector2f v(1, 1);
    fmt::print("original:       {}\n", v);
    auto v2 = v;
    v = 2.0f * v;
    v /= 2.0f;
    v += espp::Vector2f(0, 1);
    v -= espp::Vector2f(0, 1);
    fmt::print("should be same: {}, {}\n", v, v2);
    // for good measure, print all the comparisons
    fmt::print("       v == v2: {}\n", v == v2);
    fmt::print("       v != v2: {}\n", v != v2);
    fmt::print("       v <  v2: {}\n", v < v2);
    fmt::print("       v <= v2: {}\n", v <= v2);
    fmt::print("       v >  v2: {}\n", v > v2);
    fmt::print("       v >= v2: {}\n", v >= v2);
    fmt::print("magnitude:      {}\n", v.magnitude());
    fmt::print("normalized:     {}\n", v.normalized());
    fmt::print("norm mag:       {}\n", v.normalized().magnitude());
    fmt::print("rotated pi/2:   {}\n", v.rotated(M_PI_2));
    //! [vector2d example]
  }

  logger.info("=== Linear Interpolation ===");
  {
    //! [lerp example]
    // simple lerp
    float a = 0.0f;
    float b = 1.0f;
    float t = 0.5f;
    float v = espp::lerp(a, b, t);
    float inv_lerp = espp::inv_lerp(a, b, v);
    fmt::print("lerp({}, {}, {}) = {}\n", a, b, t, v);
    fmt::print("inv_lerp({}, {}, {}) = {}\n", a, b, v, inv_lerp);

    // more complex lerp
    a = 3540.0f;
    b = 4350.0f;
    t = 0.5f;
    v = espp::lerp(a, b, t);
    inv_lerp = espp::inv_lerp(a, b, v);
    fmt::print("lerp({}, {}, {}) = {}\n", a, b, t, v);
    fmt::print("inv_lerp({}, {}, {}) = {}\n", a, b, v, inv_lerp);
    //! [lerp example]
  }

  logger.info("=== Piecewise Linear Interpolation ===");
  {
    //! [piecewise linear example]
    std::vector<std::pair<float, float>> points = {
        // clang-format off
      // battery voltage (mV), battery percent (%)
      {3500, 0},
      {3700, 30},
      {4000, 50},
      {4350, 75},
      {4500, 100},
        // clang-format on
    };
    fmt::print("points: {}\n", points);
    for (int i = 3000; i < 5000; i += 100) {
      float percent = espp::piecewise_linear(points, (float)i);
      fmt::print("battery voltage: {} mV -> {:.2f} %\n", i, percent);
    }
    //! [piecewise linear example]
  }

  logger.info("Math example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
