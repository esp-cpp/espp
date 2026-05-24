#include <chrono>
#include <cstdio>
#include <thread>

#include "touch.hpp"

using namespace std::chrono_literals;

namespace {
class FakeTouchDevice : public espp::ITouchDevice {
public:
  bool update(std::error_code &ec) override {
    ec.clear();
    state_.num_touch_points = state_.num_touch_points == 0 ? 1 : 0;
    state_.btn_state = state_.num_touch_points;
    state_.points[0] = {.x = 120, .y = 64};
    return true;
  }

  espp::TouchState touch_state() const override { return state_; }

  bool has_home_button() const override { return true; }

protected:
  espp::TouchState state_;
};
} // namespace

extern "C" void app_main(void) {
  {
    std::printf("Starting touch example\n");
    //! [touch example]
    FakeTouchDevice touch;
    std::error_code ec;
    touch.update(ec);
    auto state = touch.touch_state();
    auto primary = state.primary_point();
    std::printf("num_touch_points=%u x=%u y=%u btn_state=%u\n", state.num_touch_points, primary.x,
                primary.y, state.btn_state);
    //! [touch example]
  }

  std::printf("Touch example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
