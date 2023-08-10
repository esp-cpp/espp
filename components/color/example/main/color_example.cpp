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
    // make a function to colorize the output using the RGB values
    auto color_code = [](const espp::Rgb &rgb) {
      return fg(fmt::rgb(255 * rgb.r, 255 * rgb.g, 255 * rgb.b));
    };
    fmt::print("HSV to RGB:\n");
    fmt::print("-----------\n");
    // Test RGB to HSV conversion
    for (float h = 0.0f; h < 360.0f; h += 10.0f) {
      espp::Hsv hsv(h, 1.0f, 1.0f);
      // NOTE: hue values: red = 0, green = 120, blue = 240
      auto rgb = hsv.rgb();
      fmt::print(color_code(rgb), "HSV {} --> RGB {}\n", hsv, rgb);
    }

    // test HSV white conversion
    espp::Hsv white_hsv(0.0f, 0.0f, 1.0f);
    fmt::print(color_code(white_hsv.rgb()), "White HSV: {} --> RGB {}\n", white_hsv,
               white_hsv.rgb());

    // test HSV grey conversion
    espp::Hsv grey_hsv(0.0f, 0.0f, 0.5f);
    fmt::print(color_code(grey_hsv.rgb()), "Grey HSV: {} --> RGB {}\n", grey_hsv, grey_hsv.rgb());

    espp::Rgb white(1., 1., 1.);
    espp::Rgb black(0., 0., 0.);
    espp::Rgb red(1., 0., 0.);
    espp::Rgb green(0., 1., 0.);
    espp::Rgb blue(0., 0., 1.);

    // test color blending in RGB space
    fmt::print("Color blending:\n");
    fmt::print("---------------\n");
    // red + black = dark red
    auto dark_red = red + black;
    fmt::print(color_code(dark_red), "Dark Red: {}\n", dark_red);
    // red + green = dark yellow
    auto dark_yellow = red + green;
    fmt::print(color_code(dark_yellow), "Dark Yellow: {}\n", dark_yellow);
    // red + white = pink
    auto pink = red + white;
    fmt::print(color_code(pink), "Pink: {}\n", pink);
    // black + white = grey
    auto grey = black + white;
    fmt::print(color_code(grey), "Grey: {}\n", grey);

    // test rgb to hsv conversion
    fmt::print("RGB to HSV:\n");
    fmt::print("-----------\n");
    fmt::print(color_code(red), "Red rgb to hsv: {}\n", red.hsv());
    fmt::print(color_code(green), "Green rgb to hsv: {}\n", green.hsv());
    fmt::print(color_code(blue), "Blue rgb to hsv: {}\n", blue.hsv());
    fmt::print(color_code(white), "White rgb to hsv: {}\n", white.hsv());
    fmt::print(color_code(black), "Black rgb to hsv: {}\n", black.hsv());
    fmt::print(color_code(dark_red), "Dark Red rgb to hsv: {}\n", dark_red.hsv());
    fmt::print(color_code(dark_yellow), "Dark Yellow rgb to hsv: {}\n", dark_yellow.hsv());
    fmt::print(color_code(pink), "Pink rgb to hsv: {}\n", pink.hsv());
    fmt::print(color_code(grey), "Grey rgb to hsv: {}\n", grey.hsv());

    //! [color example]
  }

  fmt::print("Color example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
