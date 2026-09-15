#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "switch_pro.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

//! [switch_pro example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "SwitchProExample", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Nintendo Switch Pro (NS1) USB controller example");

  // The protocol engine: owns the controller state + the USB HID handshake.
  espp::SwitchPro controller({.log_level = espp::Logger::Verbosity::WARN});

  // Outgoing HID reports are produced from two contexts (the TinyUSB task, when a
  // host OUTPUT report arrives, and our periodic sender). Funnel them through one
  // queue drained by a single sender task so we never call write_hid_report() from
  // the TinyUSB task (where spinning for the endpoint to drain would deadlock).
  struct OutReport {
    uint8_t id;
    std::vector<uint8_t> data;
  };
  std::deque<OutReport> tx_queue;
  std::mutex tx_mutex;
  std::condition_variable tx_cv;
  auto enqueue = [&](espp::SwitchPro::ReportData rd) {
    {
      std::lock_guard<std::mutex> lock(tx_mutex);
      // cap the queue so a misbehaving host cannot grow it without bound
      if (tx_queue.size() < 16)
        tx_queue.push_back({rd.first, std::move(rd.second)});
    }
    tx_cv.notify_one();
  };

  // Configure the USB device as a single HID interface advertising the Switch Pro
  // report descriptor, with an interrupt-OUT endpoint so we receive the host's
  // OUTPUT reports (the handshake). VID/PID/strings are Nintendo's Pro Controller
  // identifiers -- required for a real Switch to bind it; EMULATION / testing only.
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.vid = espp::SwitchPro::vid;
  usb_cfg.pid = espp::SwitchPro::pid;
  usb_cfg.manufacturer = espp::SwitchPro::manufacturer_name;
  usb_cfg.product = espp::SwitchPro::product_name;
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;

  espp::UsbDevice::HidFunction hid;
  hid.interface_name = "Switch Pro Controller";
  hid.report_descriptor = controller.get_report_descriptor();
  hid.has_out_endpoint = true; // receive host OUTPUT reports (the handshake)
  hid.poll_interval_ms = 8;    // the real Pro Controller polls at 8 ms (full-speed)
  hid.on_receive = [&](std::span<const uint8_t> data) {
    // TinyUSB task context: compute the reply and queue it (don't send here).
    if (data.empty())
      return;
    if (auto reply = controller.on_hid_report(data[0], data.data(), data.size()))
      enqueue(std::move(*reply));
  };
  usb_cfg.hid = hid;

  espp::UsbDevice usb(usb_cfg);

  // On mount, kick off the handshake: the controller proactively sends its
  // device-init (0x81) report.
  usb.set_mount_callback([&]() {
    if (auto init = controller.on_attach())
      enqueue(std::move(*init));
  });

  std::error_code ec;
  if (!usb.initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    return;
  }
  logger.info("USB HID Switch Pro controller ready; connect it to a Nintendo Switch.");

  std::atomic<bool> running{true};

  // Sender task: drain queued replies first (retrying, since this is our own task
  // and may block), otherwise stream the standard input report once the host has
  // enabled reports.
  std::thread sender([&]() {
    while (running.load()) {
      OutReport rep;
      bool have = false;
      {
        std::unique_lock<std::mutex> lock(tx_mutex);
        tx_cv.wait_for(lock, 15ms, [&]() { return !tx_queue.empty() || !running.load(); });
        if (!tx_queue.empty()) {
          rep = std::move(tx_queue.front());
          tx_queue.pop_front();
          have = true;
        }
      }
      if (have) {
        std::error_code send_ec;
        for (int i = 0; i < 20 && !usb.write_hid_report(rep.id, rep.data, send_ec); ++i)
          std::this_thread::sleep_for(1ms);
      } else if (controller.is_ready()) {
        auto std_report = controller.get_input_report();
        if (!std_report.empty()) {
          std::error_code send_ec;
          usb.write_hid_report(controller.input_report_id(), std_report, send_ec); // best-effort
        }
      }
    }
  });

  // Demo: once ready, cycle A / B / X / Y (500 ms each) and sweep the left stick,
  // so a connected Switch shows live input.
  using InputReport = espp::SwitchPro::InputReport;
  int step = 0;
  while (true) {
    if (controller.is_ready()) {
      const int which = step % 4;
      const float angle = (step % 20) / 20.0f * 2.0f * 3.14159265f;
      controller.update_input_report([&](InputReport &r) {
        r.reset();
        r.set_button_a(which == 0);
        r.set_button_b(which == 1);
        r.set_button_x(which == 2);
        r.set_button_y(which == 3);
        r.set_left_joystick(0.5f * std::cos(angle), 0.5f * std::sin(angle));
      });
      ++step;
    }
    std::this_thread::sleep_for(250ms);
  }

  running.store(false);
  tx_cv.notify_all();
  sender.join();
}
//! [switch_pro example]
