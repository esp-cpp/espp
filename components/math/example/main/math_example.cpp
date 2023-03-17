#include <chrono>
#include <vector>
#include <thread>

#include "bezier.hpp"
#include "fast_math.hpp"
#include "format.hpp"
#include "gaussian.hpp"
#include "range_mapper.hpp"
#include "vector2d.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("bezier:\n");
  {
    //! [bezier example]
    using Bezier = espp::Bezier<espp::Vector2f>;
    std::array<espp::Vector2f, 4> control_points = {
      espp::Vector2f(6, 220),
      espp::Vector2f(62, 115),
      espp::Vector2f(176, 151),
      espp::Vector2f(217, 50)
    };
    Bezier bezier(Bezier::Config{
        .control_points = control_points
      });
    std::array<float, 4> weights = {
      0.5f, 2.0f, 0.5f, 2.0f
    };
    Bezier rational_bezier(Bezier::WeightedConfig{
        .control_points = control_points,
        .weights = weights
      });
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

  fmt::print("fast_math:\n");
  {
    //! [fast_ln example]
    float x = 3.0f;
    auto slow = log(x);
    auto fast = espp::fast_ln(x);
    auto diff = std::abs(slow - fast);
    fmt::print("ln({}) = {} (slow), {} (fast), diff = {}\n",
               x, slow, fast, diff);
    //! [fast_ln example]
  }
  {
    //! [fast_sqrt example]
    float x = 3.0f;
    auto slow = sqrt(x);
    auto fast = espp::fast_sqrt(x);
    auto diff = std::abs(slow - fast);
    fmt::print("sqrt({}) = {} (slow), {} (fast), diff = {}\n",
               x, slow, fast, diff);
    //! [fast_sqrt example]
  }
  {
    //! [fast_sin example]
    float x = 3.0f;
    auto slow = sin(x);
    auto fast = espp::fast_sin(x);
    auto diff = std::abs(slow - fast);
    fmt::print("sin({}) = {} (slow), {} (fast), diff = {}\n",
               x, slow, fast, diff);
    //! [fast_sin example]
  }
  {
    //! [fast_cos example]
    float x = 3.0f;
    auto slow = cos(x);
    auto fast = espp::fast_cos(x);
    auto diff = std::abs(slow - fast);
    fmt::print("cos({}) = {} (slow), {} (fast), diff = {}\n",
               x, slow, fast, diff);
    //! [fast_cos example]
  }

  fmt::print("Gaussian:\n");
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
    for (int i=0; i<=num_increments; i++) {
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

  fmt::print("RangeMapper:\n");
  {
    //! [range_mapper example]
    // Default will have output range [-1, 1]
    espp::RangeMapper<float> rm({
        .center = 127,
        .deadband = 12,
        .minimum = 0,
        .maximum = 255
      });
    // You can explicitly set output center/range. In this case the output will
    // be in the range [0, 1024]
    espp::RangeMapper<float> rm2({
        .center = 127,
        .deadband = 12,
        .minimum = 0,
        .maximum = 255,
        .output_center = 512,
        .output_range = 512
      });
    // You can also invert the input distribution, such that input values are
    // compared against the input min/max instead of input center. NOTE: this
    // also showcases the use of a non-centered input distribution.
    espp::FloatRangeMapper rm3({
        .center = 0,
        .deadband = 12,
        .minimum = 0,
        .maximum = 255,
        .invert_input = true,
        .output_center = 0,
        .output_range = 1024
      });
    // You can even invert the ouput distribution
    espp::FloatRangeMapper rm4({
        .center = 127,
        .deadband = 12,
        .minimum = 0,
        .maximum = 255,
        .invert_output = true,
      });
    auto vals = std::array<float, 13>{
      -10, 0, 10, 50, 100, 120, 127, 135, 150, 200, 250, 255, 275
    };
    fmt::print("Mapping [0, 255] -> [-1, 1]\n");
    for (const auto& v : vals) {
      fmt::print("{} -> {}\n", v, rm.map(v));
    }
    fmt::print("Mapping [0, 255] -> [0, 1024]\n");
    for (const auto& v : vals) {
      fmt::print("{} -> {}\n", v, rm2.map(v));
    }
    fmt::print("Mapping Inverted [0, 255] -> [1024, 0]\n");
    for (const auto& v : vals) {
      fmt::print("{} -> {}\n", v, rm3.map(v));
    }
    fmt::print("Mapping [0, 255] -> Inverted [1, -1]\n");
    for (const auto& v : vals) {
      fmt::print("{} -> {}\n", v, rm4.map(v));
    }
    //! [range_mapper example]
  }

  fmt::print("Vector2d:\n");
  {
    //! [vector2d example]
    espp::Vector2f v(1,1);
    fmt::print("original:       {}\n", v.to_string());
    v = 2.0f * v;
    v /= 2.0f;
    v += espp::Vector2f(0, 1);
    v -= espp::Vector2f(0, 1);
    fmt::print("should be same: {}\n", v.to_string());
    fmt::print("magnitude:      {}\n", v.magnitude());
    fmt::print("normalized:     {}\n", v.normalized().to_string());
    fmt::print("norm mag:       {}\n", v.normalized().magnitude());
    fmt::print("rotated pi/2:   {}\n", v.rotated(M_PI_2).to_string());
    //! [vector2d example]
  }

  fmt::print("Math example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
