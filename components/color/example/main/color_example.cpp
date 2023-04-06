#include <chrono>
#include <thread>
#include <vector>

#include "color.hpp"
#include "format.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting color example!\n");
    //! [color example]
    for (float h = 0.0f; h < 360.0f; h += 10.0f) {
      espp::Hsv hsv(h, 1.0f, 1.0f);
      espp::Rgb rgb(hsv);
      // NOTE: hue values: red = 0, green = 120, blue = 240
      fmt::print("HSV {} --> RGB {}\n", hsv, rgb);
    }
    // Test color blending
    espp::Rgb white(1., 1., 1.);
    espp::Rgb black(0., 0., 0.);
    espp::Rgb red(1., 0., 0.);
    espp::Rgb green(0., 1., 0.);
    espp::Rgb blue(0., 0., 1.);
    // red + black = dark red
    auto dark_red = red + black;
    fmt::print("Dark Red: {}\n", dark_red);
    // red + green = dark yellow
    auto dark_yellow = red + green;
    fmt::print("Dark Yellow: {}\n", dark_yellow);
    // red + white = pink
    auto pink = red + white;
    fmt::print("Pink: {}\n", pink);
    // black + white = grey
    auto grey = black + white;
    fmt::print("Grey: {}\n", grey);
    //! [color example]
  }

  fmt::print("Cli example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
