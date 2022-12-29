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
    //! [bezier unweighted example]
    espp::Vector2f p0(6, 220);
    espp::Vector2f p1(62, 115);
    espp::Vector2f p2(176, 151);
    espp::Vector2f p3(217, 50);
    fmt::print("% x, y\n");
    float t = 0;
    while (t < 1.0f) {
      auto p = espp::bezier(t, p0, p1, p2, p3);
      fmt::print("{},{}\n", p.x(), p.y());
      t += 0.05f;
    }
    //! [bezier unweighted example]
  }
  {
    //! [bezier weighted example]
    espp::Vector2f p0(6, 220);
    espp::Vector2f p1(62, 115);
    espp::Vector2f p2(176, 151);
    espp::Vector2f p3(217, 50);
    float w0 = 1.0f;
    float w1 = 1.0f;
    float w2 = 1.0f;
    float w3 = 1.0f;
    fmt::print("% x, y\n");
    float t = 0;
    while (t < 1.0f) {
      auto p = espp::bezier(t, p0, p1, p2, p3, w0, w1, w2, w3);
      fmt::print("{},{}\n", p.x(), p.y());
      t += 0.05f;
    }
    //! [bezier weighted example]
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
    espp::RangeMapper<float> rm({
        .center = 127,
        .deadband = 12,
        .minimum = 0,
        .maximum = 255
      });
    auto vals = std::array<float, 9>{
      0, 10, 50, 100, 127, 150, 200, 250, 255
    };
    for (auto v : vals) {
      fmt::print("{} -> {}\n", v, rm.map(v));
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
