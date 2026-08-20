#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "sdkconfig.h"

#include "esp_http_server.h"
#include "nvs_flash.h"

#include "detail/ota_stream_protocol.hpp"
#include "logger.hpp"
#include "ota.hpp"
#include "task.hpp"
#include "usb_device.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

/////////////////////////////////////////////////////////////////////////////
// HTTP transport: esp_http_server handlers streaming into espp::Ota.
//
// NOTE: this HTTP push path is transport-agnostic on the network side too: the
// exact same esp_http_server + handlers work unchanged over the espp
// `ethernet` component (or any other esp_netif) -- Ethernet needs no separate
// code path, only bringing up its netif instead of (or in addition to)
// espp::WifiSta below.
/////////////////////////////////////////////////////////////////////////////

// Tiny inline upload page served on GET /ota so any browser on the LAN can
// update the board: pick a .bin, POST it with upload progress (XHR).
static constexpr char kUploadPage[] = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>espp OTA upload</title>
<style>
  body{font-family:system-ui,sans-serif;max-width:32rem;margin:2rem auto;padding:0 1rem;background:#f4f6f9;color:#1c2330}
  @media(prefers-color-scheme:dark){body{background:#0b0e14;color:#e6eaf2}}
  progress{width:100%;height:1rem}button{padding:.4rem 1rem}#msg{white-space:pre-wrap}
</style></head><body>
<h2>espp OTA firmware upload</h2>
<p>Pick the new firmware image (e.g. <code>build/ota_example.bin</code>) and upload; the device validates, activates and reboots into it.</p>
<input type="file" id="f" accept=".bin"> <button id="b">Upload</button><br>
<label for="t">OTA token (only if configured on the device):</label>
<input type="password" id="t" autocomplete="off" placeholder="leave empty if none">
<progress id="p" value="0" max="1" hidden></progress>
<p id="msg"></p>
<script>
"use strict";
const f=document.getElementById("f"),b=document.getElementById("b"),p=document.getElementById("p"),msg=document.getElementById("msg");
b.addEventListener("click",()=>{
  const file=f.files&&f.files[0];
  if(!file){msg.textContent="Choose a .bin file first.";return;}
  if(file.size===0){msg.textContent="File is empty.";return;}
  const xhr=new XMLHttpRequest();
  xhr.open("POST","/ota");
  const tok=document.getElementById("t").value;
  if(tok)xhr.setRequestHeader("Authorization","Bearer "+tok);
  xhr.upload.onprogress=(e)=>{if(e.lengthComputable){p.hidden=false;p.value=e.loaded/e.total;}};
  xhr.onload=()=>{msg.textContent=xhr.status===200?"Success: "+xhr.responseText+"\ndevice is restarting...":"Error "+xhr.status+": "+xhr.responseText;};
  xhr.onerror=()=>{msg.textContent="Upload failed (connection error).";};
  b.disabled=true;xhr.onloadend=()=>{b.disabled=false;};
  msg.textContent="Uploading "+file.size+" bytes...";
  xhr.send(file);
});
</script></body></html>
)HTML";

static esp_err_t ota_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, kUploadPage, HTTPD_RESP_USE_STRLEN);
}

// Reply with a JSON error, mapping the espp::Ota std::error_code to a
// reasonable HTTP status. Deliberately does NOT abort: every failure path
// that reaches here either never owned a session (empty body; begin() failed
// -- note busy means ANOTHER transport's session is live and must not be
// killed) or the engine already tore it down itself (write()/finish() abort/
// end their session on failure). The socket-error path below, which DOES own
// a live session, aborts explicitly.
static esp_err_t ota_post_fail(httpd_req_t *req, espp::Ota *ota, const std::error_code &ec,
                               const char *context) {
  (void)ota;
  const char *status = "500 Internal Server Error";
  if (ec == std::errc::device_or_resource_busy)
    status = "409 Conflict"; // another update session is active
  else if (ec == std::errc::no_space_on_device || ec == std::errc::file_too_large)
    status = "413 Payload Too Large"; // image larger than the partition / declared size
  else if (ec == std::errc::illegal_byte_sequence || ec == std::errc::file_exists ||
           ec == std::errc::invalid_argument)
    status = "400 Bad Request"; // not a valid / acceptable image
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  const std::string body =
      std::string("{\"status\":\"error\",\"message\":\"") + context + ": " + ec.message() + "\"}";
  httpd_resp_send(req, body.c_str(), body.size());
  return ESP_OK;
}

// POST /ota: stream the raw request body (the .bin image) chunk-by-chunk into
// espp::Ota, using Content-Length as the image size. e.g.:
//   curl --data-binary @build/ota_example.bin http://<ip>/ota
static esp_err_t ota_post_handler(httpd_req_t *req) {
  auto *ota = static_cast<espp::Ota *>(req->user_ctx);
  std::error_code ec;
  // Optional bearer-token gate (CONFIG_EXAMPLE_OTA_HTTP_TOKEN). This is
  // transport-level gating for the demo only -- real deployments should
  // enable secure boot / signed images so the bootloader rejects unauthorized
  // firmware regardless of how it arrives.
  if constexpr (sizeof(CONFIG_EXAMPLE_OTA_HTTP_TOKEN) > 1) {
    static constexpr char kExpected[] = "Bearer " CONFIG_EXAMPLE_OTA_HTTP_TOKEN;
    char auth[128] = {};
    const bool ok =
        httpd_req_get_hdr_value_str(req, "Authorization", auth, sizeof(auth)) == ESP_OK &&
        strcmp(auth, kExpected) == 0;
    if (!ok) {
      httpd_resp_set_status(req, "401 Unauthorized");
      httpd_resp_set_type(req, "application/json");
      httpd_resp_send(req,
                      "{\"status\":\"error\",\"message\":\"missing or invalid "
                      "Authorization: Bearer token\"}",
                      HTTPD_RESP_USE_STRLEN);
      return ESP_OK;
    }
  }
  if (req->content_len == 0) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return ota_post_fail(req, ota, ec, "empty request body");
  }
  if (!ota->begin(req->content_len, ec))
    return ota_post_fail(req, ota, ec, "begin failed");
  std::vector<uint8_t> buf(4096);
  size_t remaining = req->content_len;
  while (remaining > 0) {
    const int received =
        httpd_req_recv(req, reinterpret_cast<char *>(buf.data()), std::min(remaining, buf.size()));
    if (received == HTTPD_SOCK_ERR_TIMEOUT)
      continue; // retry
    if (received <= 0) {
      // socket error / client went away: no response possible, just clean up
      std::error_code abort_ec;
      ota->abort(abort_ec);
      return ESP_FAIL;
    }
    if (!ota->write(std::span<const uint8_t>(buf.data(), static_cast<size_t>(received)), ec))
      return ota_post_fail(req, ota, ec, "write failed"); // write() already aborted the session
    remaining -= static_cast<size_t>(received);
  }
  if (!ota->finish(ec))
    return ota_post_fail(req, ota, ec, "finish (validate/activate) failed");
  char body[192];
  snprintf(body, sizeof(body),
           "{\"status\":\"ok\",\"bytes\":%u,\"boot_partition\":\"%s\",\"restarting\":true}",
           static_cast<unsigned>(req->content_len), ota->boot_partition_label().c_str());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
  // give the response time to flush, then boot the new image
  std::this_thread::sleep_for(750ms);
  ota->restart();
  return ESP_OK; // not reached
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "OtaExample", .level = espp::Logger::Verbosity::INFO});

  //! [ota_example]
  namespace proto = espp::detail::ota_stream;

  // --- The OTA engine (transport-agnostic) -----------------------------------
  espp::Ota ota({.reject_same_version = false,
                 .progress_callback =
                     [&logger](size_t written, size_t total) {
                       // log every ~64 KiB so a big image doesn't spam the log
                       if (total > 0 && (written % (64 * 1024)) < 4096)
                         logger.info("OTA progress: {} / {} bytes", written, total);
                     },
                 .log_level = espp::Logger::Verbosity::INFO});

  const auto running = ota.running_app_description();
  logger.info("Running '{}' version '{}' (built {} {}) from partition '{}' ({} bytes)",
              running.project_name, running.version, running.date, running.time,
              ota.running_partition_label(), ota.running_partition_size());
  logger.info("Next update will target partition '{}' ({} bytes)", ota.update_partition_label(),
              ota.update_partition_size());

  // --- Rollback handling ------------------------------------------------------
  // With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, an app booted right after an
  // OTA update is in the PENDING_VERIFY state: it must prove it is healthy and
  // mark itself valid, or the bootloader ROLLS BACK to the previous image on
  // the next reset. Run the application's self-checks here; this example's
  // trivial check is that we made it this far with some free heap.
  if (ota.is_pending_verify()) {
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update)");
    const bool self_check_passed = esp_get_free_heap_size() > 10 * 1024;
    std::error_code ec;
    if (self_check_passed && ota.mark_app_valid(ec)) {
      logger.info("Self-check passed -> image marked VALID; rollback cancelled");
    } else {
      logger.error("Self-check failed ({}) -> rolling back to the previous image", ec.message());
      ota.mark_app_invalid_and_rollback(ec); // reboots into the old image
    }
  }

  // --- Transport 1: USB vendor / WebUSB (espp::UsbDevice) --------------------
  // The vendor interface carries the framed OTA stream protocol (see
  // detail/ota_stream_protocol.hpp); the hosted web app
  // https://esp-cpp.github.io/espp/apps/ota_console.html speaks it in the
  // browser. RX bytes arrive in the TinyUSB task context, so they are queued
  // and dispatched from a worker task below (esp_ota_begin's flash erase can
  // take seconds and must not block the USB stack).
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp OTA";
  usb_cfg.log_level = espp::Logger::Verbosity::INFO;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp OTA (WebUSB)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/ota_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  std::mutex usb_rx_mutex;
  std::condition_variable usb_rx_cv;
  std::deque<std::vector<uint8_t>> usb_rx_queue;
  size_t usb_rx_queued_bytes = 0;
  bool usb_rx_overflow = false;
  // The protocol is one-frame-in-flight (the host waits for OK/ERROR before
  // the next DATA), so a well-behaved host queues at most ~one frame while the
  // worker is busy. Cap the queue anyway: the worker can legitimately block
  // for seconds inside esp_ota_begin()/end() (flash erase / SHA validation),
  // and a misbehaving host that pipelines OUT transfers must not be able to
  // exhaust device RAM. 8 max-size frames of headroom is far more than the
  // protocol ever needs.
  static constexpr size_t kMaxQueuedRxBytes = 8 * espp::detail::ota_stream::kMaxFrameSize;
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) {
    // TinyUSB task context: just queue the bytes and wake the worker.
    {
      std::lock_guard<std::mutex> lock(usb_rx_mutex);
      if (usb_rx_queued_bytes + data.size() > kMaxQueuedRxBytes) {
        // Overflow: drop everything (partial frames are useless once bytes
        // are missing) and let the worker abort + resynchronize + reply.
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
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {}", usb_ec.message());

  proto::StreamParser parser;
  bool restart_pending = false;
  // The OTA engine serializes sessions across ALL transports, but that alone
  // is not enough here: without ownership tracking a USB DATA/FINISH/ABORT
  // could append to / activate / cancel a session that HTTP started. Set only
  // after a successful USB BEGIN; cleared on every terminal path (FINISH and
  // ABORT end the session in all outcomes, and a failed write() aborts it).
  // If the host unplugs mid-session the flag stays set, so a reconnecting
  // host's ABORT is still honored (BEGIN would correctly fail busy first).
  bool usb_owns_session = false;
  auto handle_usb_frame = [&](const proto::Frame &frame) {
    std::error_code ec;
    auto reply_error = [&](const std::error_code &err, const std::string &context) {
      usb.write_vendor(
          proto::make_error(static_cast<uint32_t>(err.value()), context + ": " + err.message()));
    };
    switch (frame.type) {
    case proto::MessageType::Begin: {
      const auto image_size = proto::parse_u32_payload(frame);
      if (!image_size.has_value()) {
        reply_error(std::make_error_code(std::errc::invalid_argument), "malformed BEGIN");
        break;
      }
      if (ota.begin(*image_size, ec)) {
        usb_owns_session = true;
        usb.write_vendor(proto::make_ok(0));
      } else {
        // busy = another transport's session; ownership stays false
        reply_error(ec, "begin failed");
      }
      break;
    }
    case proto::MessageType::Data:
      if (!usb_owns_session) {
        reply_error(std::make_error_code(std::errc::operation_not_permitted),
                    "no USB-owned update session (send BEGIN first)");
        break;
      }
      if (ota.write(frame.payload, ec)) {
        usb.write_vendor(proto::make_ok(static_cast<uint32_t>(ota.bytes_written())));
      } else {
        usb_owns_session = false; // write() aborted the session on failure
        reply_error(ec, "write failed");
      }
      break;
    case proto::MessageType::Finish: {
      if (!usb_owns_session) {
        reply_error(std::make_error_code(std::errc::operation_not_permitted),
                    "no USB-owned update session (send BEGIN first)");
        break;
      }
      const auto written = static_cast<uint32_t>(ota.bytes_written());
      usb_owns_session = false; // finish() ends the session in all outcomes
      if (ota.finish(ec)) {
        usb.write_vendor(proto::make_ok(written));
        restart_pending = true; // reply first; the worker restarts shortly
      } else {
        reply_error(ec, "finish (validate/activate) failed");
      }
      break;
    }
    case proto::MessageType::Abort: {
      if (!usb_owns_session) {
        reply_error(std::make_error_code(std::errc::operation_not_permitted),
                    "no USB-owned update session to abort");
        break;
      }
      const auto written = static_cast<uint32_t>(ota.bytes_written());
      usb_owns_session = false; // session over either way
      if (ota.abort(ec))
        usb.write_vendor(proto::make_ok(written));
      else
        reply_error(ec, "abort failed");
      break;
    }
    default:
      reply_error(std::make_error_code(std::errc::not_supported), "unknown message type");
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
           // Bytes were dropped: any in-flight frame/image is
           // unusable. Abort a USB-owned session, resync the
           // parser, and tell the host to start over.
           if (usb_owns_session) {
             std::error_code abort_ec;
             ota.abort(abort_ec);
             usb_owns_session = false;
           }
           parser.reset();
           usb.write_vendor(proto::make_error(
               static_cast<uint32_t>(std::make_error_code(std::errc::no_buffer_space).value()),
               "RX overflow: frames dropped; transfer aborted -- wait for OK "
               "replies between frames and restart the update"));
           return false; // dropped chunks are gone; skip parse
         }
         for (const auto &chunk : chunks)
           for (const auto &frame : parser.feed(chunk))
             handle_usb_frame(frame);
         if (restart_pending) {
           // give the final OK reply time to reach the host
           std::this_thread::sleep_for(750ms);
           ota.restart();
         }
         return false; // don't stop the task
       },
       .task_config = {.name = "ota_usb", .stack_size_bytes = 8192}});
  usb_task.start();

  // --- Transports 2 & 3: WiFi (or Ethernet) + HTTP push -----------------------
  // NVS is required by the WiFi stack.
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  espp::WifiSta wifi_sta({.ssid = CONFIG_ESP_WIFI_SSID,
                          .password = CONFIG_ESP_WIFI_PASSWORD,
                          .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                          .on_connected = nullptr,
                          .on_disconnected = nullptr,
                          .on_got_ip =
                              [&logger](ip_event_got_ip_t *eventdata) {
                                logger.info("got IP: {}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
                                logger.info("  browser upload page: http://{}.{}.{}.{}/ota",
                                            IP2STR(&eventdata->ip_info.ip));
                                logger.info("  curl --data-binary @build/ota_example.bin "
                                            "http://{}.{}.{}.{}/ota",
                                            IP2STR(&eventdata->ip_info.ip));
                              },
                          .log_level = espp::Logger::Verbosity::WARN});

  // The HTTP server binds to every netif, so this exact same code serves OTA
  // over the espp `ethernet` component as well -- to use Ethernet, simply
  // bring up its netif (see the ethernet example) instead of WifiSta above.
  httpd_handle_t http_server = nullptr;
  httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
  http_cfg.stack_size = 8192; // OTA handler streams through a 4 KiB buffer
  if (httpd_start(&http_server, &http_cfg) == ESP_OK) {
    const httpd_uri_t get_uri = {
        .uri = "/ota", .method = HTTP_GET, .handler = ota_get_handler, .user_ctx = nullptr};
    const httpd_uri_t post_uri = {
        .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler, .user_ctx = &ota};
    httpd_register_uri_handler(http_server, &get_uri);
    httpd_register_uri_handler(http_server, &post_uri);
    logger.info("HTTP OTA server ready: GET /ota (upload page), POST /ota (raw image)");
    if constexpr (sizeof(CONFIG_EXAMPLE_OTA_HTTP_TOKEN) <= 1) {
      logger.warn("POST /ota is UNAUTHENTICATED (demo default): any peer that can reach this "
                  "device can install structurally-valid firmware. Set EXAMPLE_OTA_HTTP_TOKEN in "
                  "menuconfig to require a bearer token, and enable secure boot / signed images "
                  "for real deployments.");
    }
  } else {
    logger.error("Failed to start HTTP server");
  }
  //! [ota_example]

  logger.info("OTA example ready; transports: USB vendor/WebUSB, HTTP POST /ota (WiFi/Ethernet)");
  while (true) {
    std::this_thread::sleep_for(10s);
    if (ota.session_active())
      logger.info("update in progress: {} / {} bytes", ota.bytes_written(), ota.image_size());
  }
}
