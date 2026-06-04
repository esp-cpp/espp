#include <chrono>
#include <thread>

#include "adrc.hpp"

using namespace std::chrono_literals;

namespace {
struct FirstOrderPlant {
  float pole{-1.2f};
  float gain{1.0f};
  float disturbance{0.0f};
  float y{0.0f};

  float update(float input, float dt) {
    y += dt * (pole * y + gain * input + disturbance);
    return y;
  }
};

struct SecondOrderPlant {
  float natural_frequency{3.5f};
  float damping_ratio{0.45f};
  float gain{1.0f};
  float disturbance{0.0f};
  float x1{0.0f};
  float x2{0.0f};

  float update(float input, float dt) {
    auto acceleration = -2.0f * damping_ratio * natural_frequency * x2 -
                        natural_frequency * natural_frequency * x1 + gain * input + disturbance;
    x1 += dt * x2;
    x2 += dt * acceleration;
    return x1;
  }
};
} // namespace

extern "C" void app_main(void) {
  constexpr float dt = 0.001f;
  constexpr int num_steps = 5000;
  constexpr int print_interval = 100;
  fmt::print("ADRC example\n");

  {
    fmt::print("Linear first-order ADRC example\n");
    //! [adrc linear first order example]
    espp::LinearAdrcFirstOrder controller({
        .b0 = 1.0f,
        .controller_bandwidth = 8.0f,
        .observer_bandwidth = 24.0f,
        .output_min = -4.0f,
        .output_max = 4.0f,
    });
    FirstOrderPlant plant;
    for (int i = 0; i < num_steps; ++i) {
      auto time = i * dt;
      auto reference = time >= 0.25f ? 1.0f : 0.0f;
      plant.disturbance = time >= 2.5f ? -0.8f : 0.0f;
      auto control = controller.update(reference, plant.y, dt);
      auto output = plant.update(control, dt);
      if (i % print_interval == 0 || i == num_steps - 1) {
        auto state = controller.get_state();
        fmt::print("t={:0.2f}s ref={:0.2f} y={:0.3f} u={:0.3f} z2={:0.3f}\n", time, reference,
                   output, state.output, state.z2);
      }
    }
    //! [adrc linear first order example]
  }

  {
    fmt::print("Linear second-order ADRC example\n");
    //! [adrc linear second order example]
    espp::LinearAdrcSecondOrder controller({
        .b0 = 1.0f,
        .controller_bandwidth = 10.0f,
        .observer_bandwidth = 36.0f,
        .output_min = -8.0f,
        .output_max = 8.0f,
    });
    SecondOrderPlant plant;
    for (int i = 0; i < num_steps; ++i) {
      auto time = i * dt;
      auto reference = time >= 0.25f ? 1.0f : 0.0f;
      plant.disturbance = time >= 2.5f ? 1.2f : 0.0f;
      auto control = controller.update(reference, plant.x1, dt);
      auto output = plant.update(control, dt);
      if (i % print_interval == 0 || i == num_steps - 1) {
        auto state = controller.get_state();
        fmt::print("t={:0.2f}s ref={:0.2f} y={:0.3f} u={:0.3f} z3={:0.3f}\n", time, reference,
                   output, state.output, state.z3);
      }
    }
    //! [adrc linear second order example]
  }

  {
    fmt::print("Han first-order ADRC example\n");
    //! [adrc han first order example]
    espp::HanAdrcFirstOrder controller({
        .b0 = 1.0f,
        .controller_gain = 7.0f,
        .observer_bandwidth = 22.0f,
        .observer_alpha = 0.5f,
        .controller_alpha = 0.8f,
        .fal_delta = 0.01f,
        .use_tracking_differentiator = true,
        .tracking_config =
            {
                .tracking_bandwidth = 45.0f,
                .filter_factor = 5.0f,
            },
        .output_min = -4.0f,
        .output_max = 4.0f,
    });
    FirstOrderPlant plant;
    for (int i = 0; i < num_steps; ++i) {
      auto time = i * dt;
      auto reference = time >= 0.25f ? 1.0f : 0.0f;
      plant.disturbance = time >= 2.5f ? -0.8f : 0.0f;
      auto control = controller.update(reference, plant.y, dt);
      auto output = plant.update(control, dt);
      if (i % print_interval == 0 || i == num_steps - 1) {
        auto state = controller.get_state();
        fmt::print("t={:0.2f}s ref={:0.2f} td={:0.3f} y={:0.3f} u={:0.3f}\n", time, reference,
                   state.td_reference, output, state.output);
      }
    }
    //! [adrc han first order example]
  }

  {
    fmt::print("Han second-order ADRC example\n");
    //! [adrc han second order example]
    espp::HanAdrcSecondOrder controller({
        .b0 = 1.0f,
        .position_gain = 30.0f,
        .rate_gain = 6.0f,
        .observer_bandwidth = 32.0f,
        .observer_alpha1 = 0.5f,
        .observer_alpha2 = 0.25f,
        .controller_alpha1 = 0.8f,
        .controller_alpha2 = 1.5f,
        .fal_delta = 0.01f,
        .use_tracking_differentiator = true,
        .tracking_config =
            {
                .tracking_bandwidth = 60.0f,
                .filter_factor = 5.0f,
            },
        .output_min = -8.0f,
        .output_max = 8.0f,
    });
    SecondOrderPlant plant;
    for (int i = 0; i < num_steps; ++i) {
      auto time = i * dt;
      auto reference = time >= 0.25f ? 1.0f : 0.0f;
      plant.disturbance = time >= 2.5f ? 1.2f : 0.0f;
      auto control = controller.update(reference, plant.x1, dt);
      auto output = plant.update(control, dt);
      if (i % print_interval == 0 || i == num_steps - 1) {
        auto state = controller.get_state();
        fmt::print("t={:0.2f}s ref={:0.2f} td={:0.3f} y={:0.3f} u={:0.3f} z3={:0.3f}\n", time,
                   reference, state.td_reference, output, state.output, state.z3);
      }
    }
    //! [adrc han second order example]
  }

  fmt::print("ADRC example complete!\n");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
