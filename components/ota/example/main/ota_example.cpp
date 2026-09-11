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

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "sdkconfig.h"

#include "esp_http_server.h"
#include "esp_vfs.h"
#include "nvs_flash.h"

#include "detail/ota_stream_protocol.hpp"
#include "dispatcher.hpp"
#include "logger.hpp"
#include "ota.hpp"
#include "task.hpp"
#include "usb_device.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

/////////////////////////////////////////////////////////////////////////////
// Console -> USB-CDC routing.
//
// The native USB port is handed to TinyUSB for the OTA vendor interface, so the
// ESP console runs on UART0 (primary) with USB-Serial-JTAG as an early-boot
// secondary (see sdkconfig.defaults). Once TinyUSB is up we ALSO add a USB-CDC
// interface and route the console to it, so a single native USB cable carries
// BOTH the OTA vendor stream and the logs.
//
// espp::Logger writes with fmt::print(...) to stdout (not ESP_LOG's vprintf),
// and printf / ESP_LOG default to stdout too, so redirecting *stdout* captures
// all of them. We register a tiny write-only VFS device whose write() tees each
// chunk to (a) the original UART0 console fd — kept as a permanent fallback so
// `idf.py monitor` on UART0 keeps working and nothing is lost when no CDC host
// is attached — and (b) the USB-CDC interface, but only when a host has it open
// (DTR) AND the whole chunk fits the TX FIFO right now. That last check keeps
// logging non-blocking: write_cdc() would otherwise sleep up to 250 ms waiting
// for an absent / slow reader to drain. Dropped console bytes are harmless.
/////////////////////////////////////////////////////////////////////////////
namespace {
espp::UsbDevice *g_console_usb = nullptr;
int g_console_fallback_fd = -1; // fd of the primary (UART0) console, kept as a tee

int cdc_vfs_open(const char *, int, int) { return 0; }
int cdc_vfs_close(int) { return 0; }
int cdc_vfs_fstat(int, struct stat *st) {
  *st = {};
  st->st_mode = S_IFCHR; // a character device (console), so stdio uses line/no buffering
  return 0;
}

ssize_t cdc_vfs_write(int, const void *data, size_t size) {
  if (g_console_fallback_fd >= 0)
    ::write(g_console_fallback_fd, data, size); // always keep UART0 as a tee
  // Mirror to USB-CDC when the interface is mounted and the whole chunk fits the
  // TX FIFO right now. cdc_write_available() returns 0 unless the device is
  // mounted, so this never blocks and never partially writes. We intentionally
  // do NOT gate on DTR (is_cdc_connected()): a plain serial monitor frequently
  // does not assert DTR, yet a debug console should still emit — the host's CDC
  // driver buffers what we send and hands it over once a reader attaches. If
  // nothing is draining, the FIFO fills and we simply drop the chunk (harmless).
  if (g_console_usb && g_console_usb->cdc_write_available() >= size) {
    std::error_code ec;
    g_console_usb->write_cdc({static_cast<const uint8_t *>(data), size}, ec);
  }
  return static_cast<ssize_t>(size);
}

// Redirect stdout to the CDC-teeing VFS device. Call once, after the USB device
// (with a CDC function) has initialized. Best-effort: on any failure the console
// simply stays on UART0.
void route_console_to_cdc(espp::UsbDevice &usb) {
  fflush(stdout);
  g_console_usb = &usb;
  // Open the primary console (UART0) directly to keep it as a tee. (ESP-IDF's
  // libc has no dup(), so we can't dup stdout's fd; the uart VFS exposes the
  // device by path instead.) On failure we simply don't tee to UART0.
  g_console_fallback_fd = open("/dev/uart/0", O_WRONLY);
  esp_vfs_t vfs = {};
  vfs.flags = ESP_VFS_FLAG_DEFAULT;
  // The classic (context-pointer-less) esp_vfs_t members are marked deprecated
  // in IDF v6, but are perfect for this tiny write-only console sink; use them
  // deliberately and suppress the deprecation notice.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  vfs.open = &cdc_vfs_open;
  vfs.write = &cdc_vfs_write;
  vfs.close = &cdc_vfs_close;
  vfs.fstat = &cdc_vfs_fstat;
#pragma GCC diagnostic pop
  if (esp_vfs_register("/dev/cdc", &vfs, nullptr) != ESP_OK)
    return;
  if (freopen("/dev/cdc", "w", stdout) == nullptr)
    return;
  setvbuf(stdout, nullptr, _IONBF, 0); // push each log line to CDC promptly
}
} // namespace

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
  .card{background:#fff;border:1px solid #d6dae2;border-radius:.5rem;padding:.75rem 1rem;margin:1rem 0}
  @media(prefers-color-scheme:dark){.card{background:#141a24;border-color:#2a3240}}
  .badge{display:inline-block;padding:.1rem .5rem;border-radius:1rem;font-size:.85rem;font-weight:600}
  .ok{background:#1f8f4e22;color:#1f8f4e}.warn{background:#c8860022;color:#c88600}.muted{opacity:.7}
  #fw{font-family:ui-monospace,monospace}
</style></head><body>
<h2>espp OTA firmware upload</h2>

<div class="card">
  <div>Currently running: <span id="fw" class="muted">loading…</span></div>
  <div id="state" style="margin-top:.35rem"></div>
  <div style="margin-top:.5rem">
    <button id="mv" hidden>Mark running image valid</button>
    <button id="rb" hidden>Roll back to previous image</button>
    <button id="refresh" title="Refresh status">↻</button>
  </div>
</div>

<p>Pick the new firmware image (e.g. <code>build/ota_example.bin</code>) and upload; the device validates, activates and reboots into it.</p>
<input type="file" id="f" accept=".bin"> <button id="b">Upload</button><br>
<label for="t">OTA token (only if configured on the device):</label>
<input type="password" id="t" autocomplete="off" placeholder="leave empty if none">
<progress id="p" value="0" max="1" hidden></progress>
<p id="msg"></p>
<script>
"use strict";
const $=(id)=>document.getElementById(id);
const f=$("f"),b=$("b"),p=$("p"),msg=$("msg"),fw=$("fw"),state=$("state"),mv=$("mv"),rb=$("rb");
function authHeader(xhr){const tok=$("t").value;if(tok)xhr.setRequestHeader("Authorization","Bearer "+tok);}
function loadStatus(){
  fetch("/status").then(r=>r.json()).then(s=>{
    fw.textContent=s.project+" "+s.version;fw.classList.remove("muted");
    if(!s.rollback_supported){state.innerHTML='<span class="badge muted">rollback not supported</span>';mv.hidden=true;rb.hidden=true;return;}
    if(s.pending_verify){state.innerHTML='<span class="badge warn">PENDING VERIFY</span> — rolls back on the next reset unless confirmed.';}
    else{state.innerHTML='<span class="badge ok">confirmed</span>';}
    mv.hidden=!s.pending_verify;rb.hidden=false;
  }).catch(()=>{fw.textContent="(status unavailable)";});
}
function postAction(url,pending){
  const xhr=new XMLHttpRequest();xhr.open("POST",url);authHeader(xhr);
  xhr.onload=()=>{msg.textContent=(xhr.status===200?"OK: ":"Error "+xhr.status+": ")+xhr.responseText;setTimeout(loadStatus,300);};
  xhr.onerror=()=>{msg.textContent="Request failed (connection error).";};
  msg.textContent=pending;xhr.send();
}
mv.addEventListener("click",()=>postAction("/mark-valid","Marking image valid…"));
rb.addEventListener("click",()=>{
  if(!confirm("Roll back to the previous image and reboot?"))return;
  const xhr=new XMLHttpRequest();xhr.open("POST","/rollback");authHeader(xhr);
  // On success the device reboots with no reply -> the connection drops (onerror).
  // A completed response means rollback was refused (e.g. no image to roll back to).
  xhr.onload=()=>{msg.textContent="Rollback refused: "+xhr.responseText;};
  xhr.onerror=()=>{msg.textContent="Device is rolling back and rebooting…";};
  msg.textContent="Requesting rollback…";xhr.send();
});
$("refresh").addEventListener("click",loadStatus);
b.addEventListener("click",()=>{
  const file=f.files&&f.files[0];
  if(!file){msg.textContent="Choose a .bin file first.";return;}
  if(file.size===0){msg.textContent="File is empty.";return;}
  const xhr=new XMLHttpRequest();
  xhr.open("POST","/ota");authHeader(xhr);
  xhr.upload.onprogress=(e)=>{if(e.lengthComputable){p.hidden=false;p.value=e.loaded/e.total;}};
  xhr.onload=()=>{msg.textContent=xhr.status===200?"Success: "+xhr.responseText+"\ndevice is restarting...":"Error "+xhr.status+": "+xhr.responseText;};
  xhr.onerror=()=>{msg.textContent="Upload failed (connection error).";};
  b.disabled=true;xhr.onloadend=()=>{b.disabled=false;};
  msg.textContent="Uploading "+file.size+" bytes...";
  xhr.send(file);
});
loadStatus();
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
// Optional bearer-token gate (CONFIG_EXAMPLE_OTA_HTTP_TOKEN) shared by every
// mutating endpoint (POST /ota, /mark-valid, /rollback). Returns true when the
// request is authorized (or no token is configured); otherwise sends a 401 JSON
// response and returns false. This is transport-level gating for the demo only
// -- real deployments should enable secure boot / signed images so the
// bootloader rejects unauthorized firmware regardless of how it arrives.
static bool ota_http_authorized(httpd_req_t *req) {
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
      return false;
    }
  }
  return true;
}

// JSON boolean literal for a runtime flag. Kept as a function (not an inline
// `b ? "true" : "false"`) so a caller whose argument is a compile-time constant
// in a given build config -- e.g. the rollback flags below when
// CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is off -- does not become a
// known-true/false *condition* that static analysis flags.
static const char *json_bool(bool b) { return b ? "true" : "false"; }

// GET /status: report the running firmware + rollback state as JSON, so the
// upload page can show what is running and whether it still needs confirming.
// Session-independent; mirrors the vendor GET_STATUS reply.
static esp_err_t ota_status_handler(httpd_req_t *req) {
  const auto *ota = static_cast<const espp::Ota *>(req->user_ctx);
  bool rollback_supported = false, pending = false;
#if defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
  rollback_supported = true;
  pending = ota->is_pending_verify();
#endif
  const auto desc = ota->running_app_description();
  char body[256];
  snprintf(body, sizeof(body),
           "{\"project\":\"%s\",\"version\":\"%s\",\"pending_verify\":%s,"
           "\"rollback_supported\":%s}",
           desc.project_name.c_str(), desc.version.c_str(), json_bool(pending),
           json_bool(rollback_supported));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// POST /mark-valid: HOST-driven confirmation of the running image (cancel the
// pending rollback). The app must not confirm itself; this lets an operator do
// it from the LAN page after checking the device is healthy.
static esp_err_t ota_mark_valid_handler(httpd_req_t *req) {
  if (!ota_http_authorized(req))
    return ESP_OK;
  auto *ota = static_cast<espp::Ota *>(req->user_ctx);
  std::error_code ec;
  if (!ota->mark_app_valid(ec))
    return ota_post_fail(req, ota, ec, "mark valid failed");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, "{\"status\":\"ok\",\"message\":\"image marked valid; rollback cancelled\"}",
                  HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// POST /rollback: reject the running image and reboot into the previous one.
// mark_app_invalid_and_rollback() does NOT return on success (it reboots), so
// there is deliberately NO "ok" response: the client sees the connection drop as
// the device reboots, and the page treats that as success. Only a *failure*
// (e.g. no valid image to roll back to) returns here and gets an explicit error
// response — so we never report "rolling back" for a rollback that was refused.
static esp_err_t ota_rollback_handler(httpd_req_t *req) {
  if (!ota_http_authorized(req))
    return ESP_OK;
  auto *ota = static_cast<espp::Ota *>(req->user_ctx);
  std::error_code ec;
  ota->mark_app_invalid_and_rollback(ec);                // reboots on success (never returns)
  return ota_post_fail(req, ota, ec, "rollback failed"); // only reached on failure
}

static esp_err_t ota_post_handler(httpd_req_t *req) {
  auto *ota = static_cast<espp::Ota *>(req->user_ctx);
  std::error_code ec;
  if (!ota_http_authorized(req))
    return ESP_OK;
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
  // OTA update is in the PENDING_VERIFY state: it will ROLL BACK to the previous
  // image on the next reset unless it is confirmed. This example demonstrates
  // HOST-DRIVEN confirmation: it deliberately does NOT mark itself valid here.
  // Instead it stays pending and lets the host confirm it (MARK_VALID over the
  // OTA protocol) once the host has verified the device is healthy — a broken
  // build could otherwise self-validate right before failing. The ota-console web
  // app / `espp-ota` CLI do this after reconnecting.
  //
  // (If your own product prefers device self-validation, run your health checks
  // here and call ota.mark_app_valid() / ota.mark_app_invalid_and_rollback().)
  if (ota.is_pending_verify()) {
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update). Waiting for the "
                "host to confirm it (MARK_VALID); it rolls back on the next reset if not.");
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
  // Add a CDC-ACM function so the SAME native USB cable also carries the log
  // console once TinyUSB is up (routed below, after initialize()). CDC uses 1
  // interrupt IN + 1 bulk IN + 1 bulk OUT; with the vendor function's bulk IN +
  // OUT that is 3 IN / 2 OUT endpoints — within the ESP32-S3 budget.
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp OTA console";
  usb_cfg.cdc = cdc;
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
  if (!usb.initialize(usb_ec)) {
    logger.error("Failed to initialize USB device: {}", usb_ec.message());
  } else {
    // TinyUSB now owns the native USB port (vendor OTA + CDC). Route the console
    // to the CDC interface so one cable carries logs too; UART0 stays teed as a
    // fallback (see route_console_to_cdc). Log the handoff on UART0 first.
    logger.info("Routing console to USB-CDC (single cable: OTA + logs; UART0 stays teed).");
    route_console_to_cdc(usb);
    logger.info("Console is now also on USB-CDC.");
  }

  // Route the vendor stream through a Dispatcher: OTA occupies module id 0 (its
  // opcodes are 0x0X). Other protocols (e.g. a crash-dump service on module 4)
  // could register alongside on the same stream and would be routed
  // independently; frames for unregistered modules are ignored rather than
  // mis-handled as malformed OTA frames.
  espp::Dispatcher dispatcher;
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
    // The device only handles requests; OTA replies (OK/ERROR/PROGRESS) share
    // module 0, so ignore any reply-flagged frame (e.g. a loopback echo) rather
    // than treating it as an unknown request.
    if (frame.is_reply())
      return;
    std::error_code ec;
    auto reply_error = [&](const std::error_code &err, const std::string &context) {
      usb.write_vendor(
          proto::make_error(static_cast<uint32_t>(err.value()), context + ": " + err.message()));
    };
    switch (static_cast<proto::MessageType>(frame.type)) {
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
    case proto::MessageType::GetStatus: {
      // Report rollback status + the running firmware (so the host can show what
      // is now running before confirming it). Session-independent (no BEGIN).
      uint8_t flags = 0;
#if defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
      flags |= proto::kStatusRollbackSupported;
      if (ota.is_pending_verify())
        flags |= proto::kStatusPendingVerify;
#endif
      const auto desc = ota.running_app_description();
      usb.write_vendor(proto::make_status(flags, desc.version, desc.project_name));
      break;
    }
    case proto::MessageType::MarkValid:
      // The HOST confirms the running image after its own health checks — the app
      // must not confirm itself. Cancels the pending rollback.
      if (ota.mark_app_valid(ec))
        usb.write_vendor(proto::make_ok(0));
      else
        reply_error(ec, "mark valid failed");
      break;
    case proto::MessageType::MarkInvalid:
      // Reject the running image: roll back to the previous app and reboot.
      // mark_app_invalid_and_rollback() does NOT return on success (the device
      // reboots), so DON'T pre-send OK: the reboot / USB disconnect IS the
      // success signal to the host. It only returns on *failure* (e.g. no valid
      // image to roll back to), so the reply below is reached only then and an
      // ERROR is the sole reply. Sending OK first would let the host report
      // success even when rollback was refused, leaving a stale ERROR on the
      // stream.
      ota.mark_app_invalid_and_rollback(ec);
      reply_error(ec, "rollback failed"); // only reached on failure
      break;
    default:
      reply_error(std::make_error_code(std::errc::not_supported), "unknown message type");
      break;
    }
  };

  // OTA is module id 0. The Dispatcher routes each frame for that module here.
  // Advertise it (name / web app / description) so the browser Device Hub can
  // discover and link it, and answer discovery queries over the vendor stream.
  dispatcher.register_module(
      proto::kModule, [&](const proto::Frame &frame) { handle_usb_frame(frame); },
      {.name = "OTA", .app = "ota_console.html", .description = "Firmware update over USB"});
  dispatcher.set_device_info(usb_cfg.product);
  dispatcher.serve_discovery([&](std::span<const uint8_t> frame) { usb.write_vendor(frame); });

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
           dispatcher.reset();
           usb.write_vendor(proto::make_error(
               static_cast<uint32_t>(std::make_error_code(std::errc::no_buffer_space).value()),
               "RX overflow: frames dropped; transfer aborted -- wait for OK "
               "replies between frames and restart the update"));
           return false; // dropped chunks are gone; skip parse
         }
         for (const auto &chunk : chunks)
           dispatcher.feed(chunk);
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
    // Status + host-driven rollback endpoints backing the upload page's status
    // card (running firmware, pending-verify state, mark-valid / rollback).
    const httpd_uri_t status_uri = {
        .uri = "/status", .method = HTTP_GET, .handler = ota_status_handler, .user_ctx = &ota};
    const httpd_uri_t mark_valid_uri = {.uri = "/mark-valid",
                                        .method = HTTP_POST,
                                        .handler = ota_mark_valid_handler,
                                        .user_ctx = &ota};
    const httpd_uri_t rollback_uri = {
        .uri = "/rollback", .method = HTTP_POST, .handler = ota_rollback_handler, .user_ctx = &ota};
    httpd_register_uri_handler(http_server, &get_uri);
    httpd_register_uri_handler(http_server, &post_uri);
    httpd_register_uri_handler(http_server, &status_uri);
    httpd_register_uri_handler(http_server, &mark_valid_uri);
    httpd_register_uri_handler(http_server, &rollback_uri);
    logger.info("HTTP OTA server ready: GET /ota (upload page + status), POST /ota (raw image), "
                "GET /status, POST /mark-valid, POST /rollback");
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
  size_t ticks = 0;
  while (true) {
    std::this_thread::sleep_for(10s);
    if (ota.session_active()) {
      logger.info("update in progress: {} / {} bytes", ota.bytes_written(), ota.image_size());
      continue;
    }
    // Idle heartbeat: the device otherwise logs only on events (boot / OTA), so
    // print a periodic liveness line. It also makes the USB-CDC console visibly
    // work when you attach a monitor to it after boot.
    logger.info("alive {}s{}", (++ticks) * 10,
                ota.is_pending_verify() ? " (PENDING VERIFY — awaiting host MARK_VALID)" : "");
  }
}
