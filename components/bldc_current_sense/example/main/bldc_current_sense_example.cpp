#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include "bldc_concepts.hpp"
#include "bldc_driver.hpp"
#include "current_sense.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

// Verify at compile time that espp::CurrentSense satisfies the
// CurrentSensorConcept that espp::BldcMotor requires for its current sense.
static_assert(espp::CurrentSensorConcept<espp::CurrentSense<espp::BldcDriver>>,
              "CurrentSense must satisfy CurrentSensorConcept");

// ---------------------------------------------------------------------------
// Simple software model of the analog front-end so the example can run without
// any motor hardware. It produces phase currents (in amps) from a commanded
// (Id, Iq) at a given electrical angle, plus a fixed per-channel bias to
// exercise the offset calibration. Phase C is reported as NAN to exercise the
// two-phase reconstruction path.
// ---------------------------------------------------------------------------
struct SimFrontEnd {
  std::atomic<float> id{0.0f};
  std::atomic<float> iq{0.0f};
  std::atomic<float> angle{0.0f};
  // analog bias of the sense channels (what offset calibration should remove)
  float bias_a{1.65f};
  float bias_b{1.66f};

  espp::PhaseCurrent read() const {
    const float th = angle.load();
    const float a = id.load() * std::cos(th) - iq.load() * std::sin(th); // alpha
    const float b = id.load() * std::sin(th) + iq.load() * std::cos(th); // beta
    // inverse (amplitude-invariant) Clarke -> phase currents
    espp::PhaseCurrent pc;
    pc.a = a + bias_a;
    pc.b = (-0.5f * a + espp::_SQRT3_2 * b) + bias_b;
    pc.c = NAN; // phase C not measured on this "board"
    return pc;
  }
};

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "BLDC Current Sense Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [bldc current sense example]
  static SimFrontEnd front_end;

  espp::CurrentSense<espp::BldcDriver> current_sense({
      // the read callback returns the latest sampled phase currents (amps). On
      // real hardware you would trigger this read from
      // BldcDriver::register_pwm_sample_callback() so it is PWM-synchronized.
      .read_phase_currents = []() -> espp::PhaseCurrent { return front_end.read(); },
      .driver = nullptr, // no driver needed for this offline example
      .calibration_samples = 200,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // 1) Calibrate the zero-current offsets while the "motor" is at rest.
  front_end.id = 0.0f;
  front_end.iq = 0.0f;
  bool ok = current_sense.driver_align(1.0f);
  logger.info("Calibration {}", ok ? "succeeded" : "failed");

  // 2) Command a known (Id, Iq) at a few electrical angles and confirm the
  //    current sense recovers it (Id ~ 0, Iq ~ commanded), proving the
  //    offset removal + Clarke/Park + phase reconstruction pipeline.
  const float commanded_iq = 2.5f;
  front_end.id = 0.0f;
  front_end.iq = commanded_iq;
  for (float angle = 0.0f; angle < espp::_2PI; angle += espp::_PI_3) {
    front_end.angle = angle;
    auto dq = current_sense.get_foc_currents(angle);
    float dc = current_sense.get_dc_current(angle);
    logger.info(
        "angle={:5.2f} rad -> Id={:6.3f} A, Iq={:6.3f} A (commanded {:.3f}), |Idc|={:6.3f} A",
        angle, dq.d, dq.q, commanded_iq, dc);
  }
  //! [bldc current sense example]

  logger.info("Example complete!");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
