#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <sdkconfig.h>
#include <vector>

#include "esp_core_dump.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"

#include "format.hpp"

#include "bldc_driver.hpp"
#include "bldc_haptics.hpp"
#include "bldc_motor.hpp"
#include "i2c.hpp"
#include "mt6701.hpp"
#include "ota.hpp"
#include "task.hpp"
#include "usb_device.hpp"

#include "haptics_usb_protocol.hpp"

#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI
#include "motorgo-mini.hpp"
#elif CONFIG_EXAMPLE_HARDWARE_MOTORGO_AXIS
#include "motorgo-axis.hpp"
#endif

using namespace std::chrono_literals;

// The MotorGo boards route the magnetic encoder over an SSI bus, while the
// test-stand / custom wiring uses an I2C MT6701.
#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI || CONFIG_EXAMPLE_HARDWARE_MOTORGO_AXIS
using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
#else
using Encoder = espp::Mt6701<>;
#endif
using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;
using BldcHaptics = espp::BldcHaptics<BldcMotor>;

// Which MotorGo channel to drive (index 0 == "Motor 1", index 1 == "Motor 2").
#if CONFIG_EXAMPLE_MOTOR_CHANNEL_2
static constexpr size_t example_motor_index = 1;
#else
static constexpr size_t example_motor_index = 0;
#endif

// The USB telemetry / web dial needs the continuous knob value, i.e. the detent
// index PLUS the fractional progress towards the neighboring detents. The
// detent center and active config are protected in espp::BldcHaptics, so expose
// them with a thin subclass.
class HapticKnob : public BldcHaptics {
public:
  using BldcHaptics::BldcHaptics;

  /// Shaft angle (radians) of the center of the current detent.
  float detent_center() const { return current_detent_center_; }

  /// Thread-safe copy of the active detent config.
  espp::detail::DetentConfig config_copy() {
    std::unique_lock<std::mutex> lk(detent_mutex_);
    return detent_config_;
  }
};

// The detent / haptic mode presets exposed over USB (indices are the wire
// `mode index`; keep PROTOCOL.md and the webapp in sync when editing).
struct Preset {
  const char *name;
  const espp::detail::DetentConfig *config;
};
static constexpr size_t kDefaultPresetIndex = 4; // coarse values / strong detents
static const std::array<Preset, 9> kPresets = {{
    {"Unbounded, no detents", &espp::detail::UNBOUNDED_NO_DETENTS},
    {"Bounded, no detents", &espp::detail::BOUNDED_NO_DETENTS},
    {"Multi-rev, no detents", &espp::detail::MULTI_REV_NO_DETENTS},
    {"On/off, strong detents", &espp::detail::ON_OFF_STRONG_DETENTS},
    {"Coarse values, strong detents", &espp::detail::COARSE_VALUES_STRONG_DETENTS},
    {"Fine values, no detents", &espp::detail::FINE_VALUES_NO_DETENTS},
    {"Fine values, with detents", &espp::detail::FINE_VALUES_WITH_DETENTS},
    {"Magnetic detents", &espp::detail::MAGNETIC_DETENTS},
    {"Return to center, with detents", &espp::detail::RETURN_TO_CENTER_WITH_DETENTS},
}};

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLDC Haptics Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB-controlled BLDC haptics example");

  namespace proto = haptics_proto;

  // --------------------------------------------------------------------------
  // Last-crash report. TinyUSB owns the S3's only USB PHY, so there is no live
  // USB-Serial-JTAG console and a panic backtrace cannot be watched directly;
  // instead panics core-dump to flash (see sdkconfig/partitions) and THIS boot
  // summarizes the previous crash - over the CDC banner below, the console,
  // and the GET_CRASH protocol command (shown in the web console's log).
  // --------------------------------------------------------------------------
  std::string crash_report;
  {
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const char *reset_names[] = {"UNKNOWN", "POWERON",  "EXT", "SW",        "PANIC",
                                 "INT_WDT", "TASK_WDT", "WDT", "DEEPSLEEP", "BROWNOUT",
                                 "SDIO",    "USB",      "JTAG"};
    const auto reason_index = static_cast<size_t>(reset_reason);
    const char *reason_name =
        reason_index < std::size(reset_names) ? reset_names[reason_index] : "?";
    logger.info("Reset reason: {} ({})", reason_name, static_cast<int>(reset_reason));
    if (esp_core_dump_image_check() == ESP_OK) {
      esp_core_dump_summary_t summary = {};
      if (esp_core_dump_get_summary(&summary) == ESP_OK) {
        crash_report = fmt::format("last reset: {} | crashed task '{}' PC=0x{:08x}", reason_name,
                                   summary.exc_task, summary.exc_pc);
        crash_report += " | backtrace:";
        const auto depth =
            std::min<uint32_t>(summary.exc_bt_info.depth, std::size(summary.exc_bt_info.bt));
        for (uint32_t i = 0; i < depth; i++)
          crash_report += fmt::format(" 0x{:08x}", summary.exc_bt_info.bt[i]);
        if (summary.exc_bt_info.corrupted)
          crash_report += " (corrupted)";
        crash_report +=
            "\ndecode with: xtensa-esp32s3-elf-addr2line -pfiaC -e build/bldc_haptics.elf <addrs>";
        logger.error("Previous crash detected: {}", crash_report);
      }
    } else if (reset_reason == ESP_RST_BROWNOUT || reset_reason == ESP_RST_INT_WDT ||
               reset_reason == ESP_RST_TASK_WDT) {
      // no core dump is written for these, but the reason itself is the story
      crash_report = fmt::format(
          "last reset: {} (no core dump: {})", reason_name,
          reset_reason == ESP_RST_BROWNOUT ? "brownout - check motor/USB power" : "watchdog reset");
      logger.error("Previous abnormal reset: {}", crash_report);
    }
  }

  // --------------------------------------------------------------------------
  // Motor / driver setup (board-dependent)
  // --------------------------------------------------------------------------
  std::shared_ptr<espp::BldcDriver> driver;
  std::shared_ptr<BldcMotor> motor;

#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI || CONFIG_EXAMPLE_HARDWARE_MOTORGO_AXIS
#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI
  using Board = espp::MotorGoMini;
  logger.info("Using MotorGo Mini, motor channel {}", example_motor_index + 1);
#else
  using Board = espp::MotorGoAxis;
  logger.info("Using MotorGo Axis, motor channel {}", example_motor_index + 1);
#endif
  // Both MotorGo boards expose the same symmetric, index-based API, so the
  // rest of the setup is identical regardless of which board is selected.
  auto &board = Board::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);
  board.initialize_encoders(); // start the encoder update task(s)
  board.initialize_motors();   // create the motor driver(s)
  auto motor_config = board.default_motor_config(example_motor_index);
  // tweak motor_config here if desired (PID gains, current limit, etc.)
  motor = board.initialize_motor(example_motor_index, motor_config);
  driver = board.motor_driver(example_motor_index);
#else
  logger.info("Using test-stand / custom wiring (I2C MT6701 + TMC6300)");
  // Objects which must outlive the motor for the standalone (I2C) wiring.
  std::unique_ptr<espp::I2c> i2c;
  std::shared_ptr<Encoder> standalone_encoder;
  // make the I2C that we'll use to communicate with the mt6701 (magnetic encoder)
  logger.info("initializing i2c driver...");
  i2c = std::make_unique<espp::I2c>(espp::I2c::Config{
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
      .clk_speed = 1 * 1000 * 1000, // MT6701 supports 1 MHz I2C
  });

  // now make the mt6701 which decodes the data
  std::error_code ec;
  auto encoder_device =
      i2c->add_device<uint8_t>({.device_address = Encoder::DEFAULT_ADDRESS,
                                .timeout_ms = static_cast<int>(i2c->config().timeout_ms),
                                .scl_speed_hz = i2c->config().clk_speed,
                                .log_level = espp::Logger::Verbosity::WARN},
                               ec);
  if (!encoder_device) {
    logger.error("Failed to initialize MT6701 I2C device: {}", ec.message());
    return;
  }
  static constexpr float core_update_period = 0.001f; // seconds
  standalone_encoder = std::make_shared<Encoder>(
      Encoder::Config{.write = espp::make_i2c_addressed_write(encoder_device),
                      .read = espp::make_i2c_addressed_read(encoder_device),
                      .velocity_filter = nullptr, // no filtering
                      .update_period = std::chrono::duration<float>(core_update_period),
                      .log_level = espp::Logger::Verbosity::WARN});

  // now make the bldc driver
  driver = std::make_shared<espp::BldcDriver>(
      espp::BldcDriver::Config{// this pinout is configured for the TinyS3 connected to the
                               // TMC6300-BOB in the BLDC Motor Test Stand
                               .gpio_a_h = 1,
                               .gpio_a_l = 2,
                               .gpio_b_h = 3,
                               .gpio_b_l = 4,
                               .gpio_c_h = 5,
                               .gpio_c_l = 21,
                               .gpio_enable = 34, // connected to the VIO/~Stdby pin of TMC6300-BOB
                               .gpio_fault = 36,  // connected to the nFAULT pin of TMC6300-BOB
                               .power_supply_voltage = 5.0f,
                               .limit_voltage = 5.0f,
                               .log_level = espp::Logger::Verbosity::WARN});

  // now make the bldc motor
  motor = std::make_shared<BldcMotor>(BldcMotor::Config{
      // measured by setting it into ANGLE_OPENLOOP and then counting how many
      // spots you feel when rotating it.
      .num_pole_pairs = 7,
      .phase_resistance =
          5.0f, // tested by running velocity_openloop and seeing if the veloicty is ~correct
      .kv_rating =
          320, // tested by running velocity_openloop and seeing if the velocity is ~correct
      .current_limit = 1.0f,        // Amps
      .zero_electric_offset = 0.0f, // set to zero to always calibrate, since this is a test
      .sensor_direction =
          espp::detail::SensorDirection::UNKNOWN, // set to unknown to always calibrate, since
                                                  // this is a test
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = driver,
      .sensor = standalone_encoder,
      .velocity_pid_config =
          {
              .kp = 0.010f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 7.000f,
              .ki = 0.300f,
              .kd = 0.010f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .log_level = espp::Logger::Verbosity::WARN});
#endif

  // --------------------------------------------------------------------------
  // Haptic engine
  // --------------------------------------------------------------------------
  //! [bldc_haptics_example_1]
  auto haptic_motor = HapticKnob({.motor = motor,
                                  .kp_factor = 2,
                                  .kd_factor_min = 0.01,
                                  .kd_factor_max = 0.04,
                                  .log_level = espp::Logger::Verbosity::INFO});

  auto detent_config = *kPresets[kDefaultPresetIndex].config;
  haptic_motor.update_detent_config(detent_config);
  // this will start the haptic motor thread which will run in the background.
  // If we want to change the detent config we can call update_detent_config()
  // and it will update the detent config in the background thread.
  haptic_motor.start();
  //! [bldc_haptics_example_1]
  logger.info("Haptics running with preset '{}'", kPresets[kDefaultPresetIndex].name);

  std::atomic<uint8_t> mode_index{kDefaultPresetIndex};
  std::atomic<bool> enabled{true};
  std::atomic<bool> streaming{false};
  std::atomic<uint16_t> stream_period_ms{20}; // 50 Hz default telemetry

  // Continuous knob value: detent index plus fractional progress towards the
  // neighboring detents. Position DEcreases as the shaft angle increases (see
  // the snap logic in BldcHaptics::motor_task), hence the minus sign.
  auto continuous_value = [&]() -> float {
    const float width = haptic_motor.config_copy().position_width;
    const float position = haptic_motor.get_position();
    if (width <= 0.0f)
      return position;
    const float angle_to_center = motor->get_shaft_angle() - haptic_motor.detent_center();
    return position - angle_to_center / width;
  };

  auto status_flags = [&]() -> uint8_t {
    uint8_t flags = 0;
    if (enabled)
      flags |= proto::flags::kEnabled;
    if (driver->is_faulted())
      flags |= proto::flags::kFaulted;
    if (streaming)
      flags |= proto::flags::kStreaming;
    return flags;
  };

  // --------------------------------------------------------------------------
  // OTA engine (transport-agnostic; fed from the USB protocol below)
  // --------------------------------------------------------------------------
  espp::Ota ota({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO});

  const auto running = ota.running_app_description();
  logger.info("Running '{}' version '{}' (built {} {}) from partition '{}'", running.project_name,
              running.version, running.date, running.time, ota.running_partition_label());

  // With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, an app booted right after an
  // OTA update is PENDING_VERIFY: it must prove it is healthy and mark itself
  // valid, or the bootloader rolls back on the next reset. Getting this far
  // (motor + haptics up) is this example's health check.
  if (ota.is_pending_verify()) {
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update)");
    std::error_code ota_ec;
    if (ota.mark_app_valid(ota_ec)) {
      logger.info("Self-check passed -> image marked VALID; rollback cancelled");
    } else {
      logger.error("Marking app valid failed ({}) -> rolling back", ota_ec.message());
      ota.mark_app_invalid_and_rollback(ota_ec); // reboots into the old image
    }
  }

  // --------------------------------------------------------------------------
  // USB vendor / WebUSB interface
  // --------------------------------------------------------------------------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = 0x0d34; // distinct from the espp default so the webapp filter is specific
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp BLDC Haptics";
  usb_cfg.log_level = espp::Logger::Verbosity::INFO;
  // CDC alongside the vendor interface: a plain serial port any terminal can
  // attach to (e.g. `screen /dev/tty.usbmodem*`). It carries the boot banner +
  // last-crash report on every connect - NOT a live console (espp logs go to
  // the default console, and a panic kills TinyUSB before it could print);
  // the flash core dump + next-boot report above is the backtrace path.
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp BLDC Haptics (debug)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp BLDC Haptics (WebUSB)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/haptics_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  // The vendor TX path is written to from two tasks (protocol worker replies +
  // telemetry), so serialize the writes.
  std::mutex usb_tx_mutex;
  auto usb_send = [&](const std::vector<uint8_t> &frame) {
    if (frame.empty())
      return;
    std::lock_guard<std::mutex> lk(usb_tx_mutex);
    usb.write_vendor(frame);
  };

  // RX bytes arrive in the TinyUSB task context: queue them and dispatch from
  // the worker task below (esp_ota_begin's flash erase can take seconds and
  // must not block the USB stack). The protocol is one-command-in-flight, so a
  // well-behaved host queues at most ~one frame; cap the queue anyway so a
  // misbehaving host cannot exhaust device RAM while the worker blocks in the
  // flash operations.
  std::mutex usb_rx_mutex;
  std::condition_variable usb_rx_cv;
  std::deque<std::vector<uint8_t>> usb_rx_queue;
  size_t usb_rx_queued_bytes = 0;
  bool usb_rx_overflow = false;
  static constexpr size_t kMaxQueuedRxBytes = 8 * proto::stream::kMaxFrameSize;
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) {
    {
      std::lock_guard<std::mutex> lock(usb_rx_mutex);
      if (usb_rx_queued_bytes + data.size() > kMaxQueuedRxBytes) {
        // Overflow: drop everything (partial frames are useless once bytes are
        // missing) and let the worker abort + resynchronize + reply.
        usb_rx_queue.clear();
        usb_rx_queued_bytes = 0;
        usb_rx_overflow = true;
      } else {
        usb_rx_queue.emplace_back(data.begin(), data.end());
        usb_rx_queued_bytes += data.size();
      }
    }
    usb_rx_cv.notify_one();
  });

  std::error_code usb_ec;
  const bool usb_ok = usb.initialize(usb_ec);
  if (!usb_ok) {
    // Not fatal for the haptics themselves: the knob keeps running standalone,
    // but everything USB-dependent (protocol worker, telemetry, OTA, CDC
    // console) is skipped below so no task ever touches a dead USB stack.
    logger.error("Failed to initialize USB device: {}; continuing WITHOUT USB "
                 "(web console / OTA / telemetry unavailable; haptics still run)",
                 usb_ec.message());
  } else {
    // Route the SYSTEM console (stdout/stderr - all espp/fmt and esp_log
    // output) to the CDC interface: TinyUSB owns the S3's only USB PHY, so
    // this replaces the unusable USB-Serial-JTAG console. Attach any serial
    // terminal (e.g. `screen /dev/tty.usbmodem*`) for live logs. Panic
    // backtraces still cannot appear live (TinyUSB dies with the panic) -
    // those are captured by the flash core dump and summarized on the next
    // boot (see crash_report above / GET_CRASH).
    if (esp_tusb_init_console(TINYUSB_CDC_ACM_0) != ESP_OK)
      logger.warn("Could not route the console to USB CDC");
  }

  // --------------------------------------------------------------------------
  // Protocol frame handling (runs in the worker task)
  // --------------------------------------------------------------------------
  proto::stream::StreamParser parser;
  bool restart_pending = false;

  auto reply_ok = [&](uint32_t value) { usb_send(proto::stream::make_ok(value)); };
  auto reply_error = [&](const std::error_code &err, const std::string &context) {
    usb_send(proto::stream::make_error(static_cast<uint32_t>(err.value()),
                                       context + ": " + err.message()));
  };
  auto reply_errc = [&](std::errc errc, const std::string &context) {
    reply_error(std::make_error_code(errc), context);
  };

  auto send_info = [&]() {
    std::vector<uint8_t> payload;
    payload.push_back(proto::kProtocolVersion);
    const auto app = ota.running_app_description();
    proto::put_str(payload, app.project_name);
    proto::put_str(payload, app.version);
    proto::put_str(payload, app.date + " " + app.time);
    proto::put_str(payload, app.idf_version);
    usb_send(proto::build(proto::Msg::Info, payload));
  };

  auto send_status = [&]() {
    std::vector<uint8_t> payload;
    payload.push_back(mode_index);
    payload.push_back(status_flags());
    proto::put_i32(payload, static_cast<int32_t>(haptic_motor.get_position()));
    proto::put_f32(payload, continuous_value());
    proto::put_f32(payload, motor->get_shaft_angle());
    proto::put_f32(payload, motor->get_shaft_velocity());
    proto::put_u16(payload, stream_period_ms);
    usb_send(proto::build(proto::Msg::Status, payload));
  };

  auto send_modes = [&]() {
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(kPresets.size()));
    for (size_t i = 0; i < kPresets.size(); i++) {
      const auto &cfg = *kPresets[i].config;
      payload.push_back(static_cast<uint8_t>(i));
      proto::put_i32(payload, static_cast<int32_t>(cfg.min_position));
      proto::put_i32(payload, static_cast<int32_t>(cfg.max_position));
      proto::put_f32(payload, cfg.position_width);
      proto::put_f32(payload, cfg.detent_strength);
      proto::put_f32(payload, cfg.end_strength);
      proto::put_f32(payload, cfg.snap_point);
      payload.push_back(static_cast<uint8_t>(cfg.detent_positions.size()));
      for (const int detent : cfg.detent_positions)
        proto::put_i32(payload, detent);
      proto::put_str(payload, kPresets[i].name);
    }
    usb_send(proto::build(proto::Msg::Modes, payload));
  };

  auto handle_frame = [&](const proto::stream::Frame &frame) {
    std::error_code ec;
    switch (static_cast<proto::Msg>(frame.type)) {
    // --- OTA subset ----------------------------------------------------------
    case proto::Msg::OtaBegin: {
      const auto image_size = proto::stream::parse_u32_payload(frame);
      if (!image_size.has_value()) {
        reply_errc(std::errc::invalid_argument, "malformed OTA BEGIN");
        break;
      }
      if (ota.begin(*image_size, ec))
        reply_ok(0);
      else
        reply_error(ec, "OTA begin failed");
      break;
    }
    case proto::Msg::OtaData:
      if (!ota.session_active()) {
        reply_errc(std::errc::operation_not_permitted, "no update session (send BEGIN first)");
        break;
      }
      if (ota.write(frame.payload, ec))
        reply_ok(static_cast<uint32_t>(ota.bytes_written()));
      else
        reply_error(ec, "OTA write failed"); // write() aborted the session on failure
      break;
    case proto::Msg::OtaFinish: {
      if (!ota.session_active()) {
        reply_errc(std::errc::operation_not_permitted, "no update session (send BEGIN first)");
        break;
      }
      const auto written = static_cast<uint32_t>(ota.bytes_written());
      if (ota.finish(ec)) {
        reply_ok(written);
        restart_pending = true; // reply first; the worker restarts shortly
      } else {
        reply_error(ec, "OTA finish (validate/activate) failed");
      }
      break;
    }
    case proto::Msg::OtaAbort: {
      if (!ota.session_active()) {
        reply_errc(std::errc::operation_not_permitted, "no update session to abort");
        break;
      }
      const auto written = static_cast<uint32_t>(ota.bytes_written());
      if (ota.abort(ec))
        reply_ok(written);
      else
        reply_error(ec, "OTA abort failed");
      break;
    }
    // --- Haptics commands ----------------------------------------------------
    case proto::Msg::GetInfo:
      send_info();
      break;
    case proto::Msg::GetStatus:
      send_status();
      break;
    case proto::Msg::GetModes:
      send_modes();
      break;
    case proto::Msg::GetCrash: {
      std::vector<uint8_t> payload(crash_report.begin(), crash_report.end());
      usb_send(proto::build(proto::Msg::Crash, payload));
      break;
    }
    case proto::Msg::SetMode: {
      if (frame.payload.size() != 1 || frame.payload[0] >= kPresets.size()) {
        reply_errc(std::errc::invalid_argument, "SET_MODE needs a valid u8 mode index");
        break;
      }
      const uint8_t index = frame.payload[0];
      haptic_motor.update_detent_config(*kPresets[index].config);
      mode_index = index;
      logger.info("Mode changed to {} ('{}')", index, kPresets[index].name);
      reply_ok(index);
      break;
    }
    case proto::Msg::SetPosition: {
      const auto position = proto::get_i32_at(frame.payload, 0);
      if (!position.has_value() || frame.payload.size() != 4) {
        reply_errc(std::errc::invalid_argument, "SET_POSITION needs an i32 position");
        break;
      }
      haptic_motor.set_position(*position);
      reply_ok(static_cast<uint32_t>(static_cast<int32_t>(haptic_motor.get_position())));
      break;
    }
    case proto::Msg::SetEnabled: {
      if (frame.payload.size() != 1) {
        reply_errc(std::errc::invalid_argument, "SET_ENABLED needs a u8 0/1");
        break;
      }
      const bool enable = frame.payload[0] != 0;
      if (enable)
        haptic_motor.start();
      else
        haptic_motor.stop();
      enabled = enable;
      logger.info("Haptics {}", enable ? "enabled" : "disabled");
      reply_ok(enable ? 1 : 0);
      break;
    }
    case proto::Msg::PlayHaptic: {
      const auto strength = proto::get_f32_at(frame.payload, 0);
      if (!strength.has_value() || frame.payload.size() != 4) {
        reply_errc(std::errc::invalid_argument, "PLAY_HAPTIC needs an f32 strength");
        break;
      }
      if (!enabled) {
        reply_errc(std::errc::operation_not_permitted, "haptics are disabled");
        break;
      }
      const float clamped = std::clamp(*strength, 0.0f, 10.0f);
      //! [bldc_haptics_example_2]
      haptic_motor.play_haptic(espp::detail::HapticConfig{
          .strength = clamped,
          .frequency = 200.0f, // Hz, NOTE: frequency is unused for now
          .duration = 1s       // NOTE: duration is unused for now
      });
      //! [bldc_haptics_example_2]
      reply_ok(0);
      break;
    }
    case proto::Msg::SetStreaming: {
      const auto period = proto::get_u16_at(frame.payload, 1);
      if (frame.payload.size() != 3 || !period.has_value()) {
        reply_errc(std::errc::invalid_argument, "SET_STREAMING needs u8 enable + u16 period_ms");
        break;
      }
      const uint16_t period_ms = std::clamp<uint16_t>(*period == 0 ? 20 : *period, 5, 1000);
      stream_period_ms = period_ms;
      streaming = frame.payload[0] != 0;
      logger.info("Telemetry streaming {} (period {} ms)", streaming ? "on" : "off", period_ms);
      reply_ok(period_ms);
      break;
    }
    default:
      reply_errc(std::errc::not_supported, "unknown message type");
      break;
    }
  };

  espp::Task usb_task(
      {.callback = [&](std::mutex &, std::condition_variable &) -> bool {
         std::vector<std::vector<uint8_t>> chunks;
         bool overflowed = false;
         {
           std::unique_lock<std::mutex> lock(usb_rx_mutex);
           usb_rx_cv.wait_for(lock, 100ms,
                              [&] { return !usb_rx_queue.empty() || usb_rx_overflow; });
           chunks.assign(std::make_move_iterator(usb_rx_queue.begin()),
                         std::make_move_iterator(usb_rx_queue.end()));
           usb_rx_queue.clear();
           usb_rx_queued_bytes = 0;
           overflowed = usb_rx_overflow;
           usb_rx_overflow = false;
         }
         if (overflowed) {
           // Bytes were dropped: any in-flight frame / OTA image is unusable.
           std::error_code abort_ec;
           ota.abort(abort_ec);
           parser.reset();
           usb_send(proto::stream::make_error(
               static_cast<uint32_t>(std::make_error_code(std::errc::no_buffer_space).value()),
               "RX overflow: frames dropped -- wait for OK replies between frames"));
           return false; // dropped chunks are gone; skip parse
         }
         for (const auto &chunk : chunks)
           for (const auto &frame : parser.feed(chunk))
             handle_frame(frame);
         if (restart_pending) {
           // give the final OK reply time to reach the host
           std::this_thread::sleep_for(750ms);
           ota.restart();
         }
         return false; // don't stop the task
       },
       .task_config = {.name = "haptics_usb", .stack_size_bytes = 8192}});
  if (usb_ok)
    usb_task.start();

  // --------------------------------------------------------------------------
  // Telemetry streaming task
  // --------------------------------------------------------------------------
  espp::Task telemetry_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         const auto start = std::chrono::steady_clock::now();
         // pause the stream while an OTA transfer runs so the bulk IN pipe
         // carries only the flow-controlled OTA replies
         if (streaming && usb.is_vendor_connected() && !ota.session_active()) {
           std::vector<uint8_t> payload;
           proto::put_u32(payload, static_cast<uint32_t>(esp_timer_get_time() / 1000));
           payload.push_back(mode_index);
           payload.push_back(status_flags());
           proto::put_i32(payload, static_cast<int32_t>(haptic_motor.get_position()));
           proto::put_f32(payload, continuous_value());
           proto::put_f32(payload, motor->get_shaft_angle());
           proto::put_f32(payload, motor->get_shaft_velocity());
           usb_send(proto::build(proto::Msg::Telemetry, payload));
         }
         {
           std::unique_lock<std::mutex> lk(m);
           cv.wait_until(lk, start + std::chrono::milliseconds(stream_period_ms.load()));
         }
         return false; // don't stop the task
       },
       // 8 KB: write_vendor's rate-limited TX-full warning goes through fmt,
       // which alone can use a few KB of stack - 4 KB overflowed (= reboot)
       // when the host stopped draining the IN endpoint.
       .task_config = {.name = "haptics_telem", .stack_size_bytes = 8192}});
  if (usb_ok)
    telemetry_task.start();

  if (usb_ok)
    logger.info("Ready: connect the native USB port and open the web console "
                "(example/webapp/index.html or https://{})",
                vendor.landing_page_url);
  else
    logger.warn("Ready (haptics only): USB failed to initialize, so the web "
                "console / OTA / telemetry are unavailable this boot");

  bool was_faulted = false;
  bool cdc_was_connected = false;
  while (true) {
    std::this_thread::sleep_for(1s);
    // A terminal attaching to the CDC console missed the boot output; re-log
    // the previous-crash summary for it once per connection.
    const bool cdc_connected = usb.is_cdc_connected();
    if (cdc_connected && !cdc_was_connected && !crash_report.empty())
      logger.error("Previous abnormal reset: {}", crash_report);
    cdc_was_connected = cdc_connected;
    const bool faulted = driver->is_faulted();
    if (faulted != was_faulted) {
      was_faulted = faulted;
      if (faulted)
        logger.error("Motor driver FAULT asserted");
      else
        logger.info("Motor driver fault cleared");
    }
  }
}
