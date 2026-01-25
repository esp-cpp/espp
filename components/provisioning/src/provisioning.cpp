#include "provisioning.hpp"

#include <algorithm>
#include <cstring>

#include "esp_wifi.h"

using namespace espp;
using namespace std::chrono_literals;

// Helper function to escape JSON strings
static std::string json_escape(const std::string &str) {
  std::string escaped;
  escaped.reserve(str.size());
  for (char c : str) {
    switch (c) {
    case '"':
      escaped += "\\\"";
      break;
    case '\\':
      escaped += "\\\\";
      break;
    case '\b':
      escaped += "\\b";
      break;
    case '\f':
      escaped += "\\f";
      break;
    case '\n':
      escaped += "\\n";
      break;
    case '\r':
      escaped += "\\r";
      break;
    case '\t':
      escaped += "\\t";
      break;
    default:
      if (c < 0x20) {
        // Control characters - escape as \uXXXX
        char buf[7];
        snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
        escaped += buf;
      } else {
        escaped += c;
      }
    }
  }
  return escaped;
}

static const char *HTML_TEMPLATE = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>WiFi Setup</title>
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; }
        .device-name { text-align: center; color: #666; font-size: 14px; margin-bottom: 20px; }
        button { width: 100%; padding: 12px; margin: 10px 0; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; }
        .scan-btn { background: #4CAF50; color: white; }
        .connect-btn { background: #2196F3; color: white; }
        .network { padding: 12px; margin: 5px 0; background: #f9f9f9; border-radius: 5px; cursor: pointer; border: 2px solid transparent; display: flex; justify-content: space-between; align-items: center; }
        .network:hover { border-color: #2196F3; }
        .network.selected { border-color: #2196F3; background: #e3f2fd; }
        .network-name { flex-grow: 1; margin-right: 15px; }
        .signal { color: #666; margin-right: 10px; }
        .delete-btn { background: #f44336; color: white; border: none; padding: 8px 15px; border-radius: 3px; cursor: pointer; font-size: 13px; margin-left: 8px; }
        .delete-btn:hover { background: #d32f2f; }
        .connect-saved-btn { background: #4CAF50; color: white; border: none; padding: 8px 15px; border-radius: 3px; cursor: pointer; font-size: 13px; }
        .connect-saved-btn:hover { background: #45a049; }
        input { width: 100%; padding: 10px; margin: 5px 0; border: 1px solid #ddd; border-radius: 5px; box-sizing: border-box; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; text-align: center; display: none; }
        .status.info { background: #e3f2fd; color: #1976d2; }
        .status.success { background: #c8e6c9; color: #388e3c; }
        .status.error { background: #ffcdd2; color: #d32f2f; }
        .loading { display: inline-block; width: 20px; height: 20px; border: 3px solid #f3f3f3; border-top: 3px solid #2196F3; border-radius: 50%; animation: spin 1s linear infinite; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        .section { margin: 20px 0; padding: 15px; border-top: 2px solid #f0f0f0; }
        .section h3 { margin-top: 0; color: #555; }
        .divider { text-align: center; color: #999; margin: 20px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>WiFi Setup</h1>
        <div class="device-name">%DEVICE_NAME%</div>
        
        <!-- Saved Networks Section -->
        <div class="section" id="saved-section" style="display:none;">
            <h3>Saved Networks</h3>
            <div id="saved-networks"></div>
        </div>
        
        <div class="divider" id="saved-divider" style="display:none;">&horbar; OR &horbar;</div>
        
        <!-- Scan Networks Section -->
        <div class="section">
            <button class="scan-btn" onclick="scanNetworks()">Scan Networks</button>
            <div id="status" class="status"></div>
            <div id="networks"></div>
            <div id="connect-form" style="display:none;">
                <h3>Connect to <span id="selected-ssid"></span></h3>
                <input type="password" id="password" placeholder="Password (leave empty if open)">
                <button class="connect-btn" onclick="connect()">Connect</button>
            </div>
        </div>
        
        <div class="divider">&horbar; OR &horbar;</div>
        
        <!-- Manual Entry Section -->
        <div class="section">
            <h3>Manual Network Entry</h3>
            <input type="text" id="manual-ssid" placeholder="Network Name (SSID)">
            <input type="password" id="manual-password" placeholder="Password (leave empty if open)">
            <button class="connect-btn" onclick="connectManual()">Connect to Network</button>
        </div>
        
        <!-- Complete Setup Button (hidden until successful connection) -->
        <button id="complete-btn" onclick="complete()" style="display:none; background:#28a745; margin-top:20px;">Complete Setup</button>
    </div>
    <script>
        let selectedSSID = '';
        
        // Load saved networks on page load
        window.onload = function() {
            loadSavedNetworks();
        };
        
        function showStatus(msg, type) {
            const status = document.getElementById('status');
            status.textContent = msg;
            status.className = 'status ' + type;
            status.style.display = 'block';
        }
        
        function loadSavedNetworks() {
            fetch('/saved').then(r => r.json()).then(data => {
                if (data.networks && data.networks.length > 0) {
                    const div = document.getElementById('saved-networks');
                    div.innerHTML = '';
                    data.networks.forEach(ssid => {
                        const net = document.createElement('div');
                        net.className = 'network';
                        net.innerHTML = '<span class="network-name">' + ssid + '</span>' +
                            '<button class="connect-saved-btn" onclick="connectSaved(\'' + ssid + '\')">Connect</button>' +
                            '<button class="delete-btn" onclick="deleteSaved(\'' + ssid + '\')">Delete</button>';
                        div.appendChild(net);
                    });
                    document.getElementById('saved-section').style.display = 'block';
                    document.getElementById('saved-divider').style.display = 'block';
                }
            }).catch(err => console.log('No saved networks'));
        }
        
        function connectSaved(ssid) {
            showStatus('Connecting to ' + ssid + '...', 'info');
            fetch('/connect', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ssid: ssid, password: '', use_saved: true})
            }).then(r => r.json()).then(data => {
                if(data.success) {
                    showStatus('Connected successfully! Click "Complete Setup" when ready.', 'success');
                    document.getElementById('complete-btn').style.display = 'inline-block';
                } else {
                    showStatus('Connection failed: ' + data.message, 'error');
                }
            }).catch(() => showStatus('Connection request failed', 'error'));
        }
        
        function deleteSaved(ssid) {
            if (!confirm('Delete saved credentials for ' + ssid + '?')) return;
            fetch('/delete', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ssid: ssid})
            }).then(r => r.json()).then(data => {
                if(data.success) {
                    loadSavedNetworks();
                    showStatus('Deleted ' + ssid, 'success');
                } else {
                    showStatus('Delete failed', 'error');
                }
            }).catch(() => showStatus('Delete request failed', 'error'));
        }
        
        function scanNetworks() {
            showStatus('Scanning for networks...', 'info');
            document.getElementById('networks').innerHTML = '';
            fetch('/scan').then(r => r.json()).then(data => {
                showStatus('Found ' + data.networks.length + ' networks', 'success');
                const div = document.getElementById('networks');
                data.networks.forEach(n => {
                    const net = document.createElement('div');
                    net.className = 'network';
                    net.innerHTML = '<span class="network-name">' + n.ssid + '<span class="signal">' + n.rssi + ' dBm</span></span>';
                    net.onclick = () => {
                        document.querySelectorAll('.network').forEach(e => e.classList.remove('selected'));
                        net.classList.add('selected');
                        selectedSSID = n.ssid;
                        document.getElementById('selected-ssid').textContent = n.ssid;
                        document.getElementById('connect-form').style.display = 'block';
                    };
                    div.appendChild(net);
                });
            }).catch(() => showStatus('Scan failed', 'error'));
        }
        function connect() {
            const pass = document.getElementById('password').value;
            doConnect(selectedSSID, pass);
        }
        function connectManual() {
            const ssid = document.getElementById('manual-ssid').value;
            const pass = document.getElementById('manual-password').value;
            if (!ssid) {
                showStatus('Please enter a network name', 'error');
                return;
            }
            doConnect(ssid, pass);
        }
        function doConnect(ssid, pass) {
            showStatus('Connecting to ' + ssid + '...', 'info');
            fetch('/connect', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ssid: ssid, password: pass})
            }).then(r => r.json()).then(data => {
                if(data.success) {
                    showStatus('Connected successfully! Click "Complete Setup" when ready.', 'success');
                    document.getElementById('complete-btn').style.display = 'inline-block';
                } else {
                    showStatus('Connection failed: ' + data.message, 'error');
                }
            }).catch(() => showStatus('Connection request failed', 'error'));
        }
        function complete() {
            showStatus('Completing provisioning...', 'info');
            fetch('/complete', {method: 'POST'})
            .then(() => showStatus('Provisioning complete! You may close this page.', 'success'))
            .catch(() => showStatus('Complete request failed', 'error'));
        }
    </script>
</body>
</html>
)HTML";

Provisioning::Provisioning(const Config &config)
    : BaseComponent("Provisioning", config.log_level) {
  init(config);
}

Provisioning::~Provisioning() { stop(); }

void Provisioning::init(const Config &config) {
  config_ = config;

  // Append MAC address to SSID if requested
  if (config_.append_mac_to_ssid) {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    config_.ap_ssid = fmt::format("{}-{:02X}{:02X}", config_.ap_ssid, mac[4], mac[5]);
  }

  logger_.info("Initialized with AP SSID: {}", config_.ap_ssid);
}

bool Provisioning::start() {
  if (is_active_) {
    logger_.warn("Provisioning already active");
    return false;
  }

  if (!start_ap()) {
    logger_.error("Failed to start AP");
    return false;
  }

  if (!start_server()) {
    logger_.error("Failed to start HTTP server");
    stop_ap();
    return false;
  }

  is_active_ = true;
  ap_start_time_ = std::chrono::steady_clock::now();
  logger_.info("Provisioning started at http://{}", get_ip_address());
  return true;
}

void Provisioning::stop() {
  if (!is_active_) {
    return;
  }

  stop_server();
  stop_ap();
  is_active_ = false;
  logger_.info("Provisioning stopped");
}

bool Provisioning::start_ap() {
  WifiAp::Config ap_config{.ssid = config_.ap_ssid,
                           .password = config_.ap_password,
                           .max_number_of_stations = 4,
                           .log_level = config_.log_level};

  wifi_ap_ = std::make_unique<WifiAp>(ap_config);

  // WiFi mode is managed by WifiAp and WifiSta classes
  return wifi_ap_ != nullptr;
}

bool Provisioning::start_server() {
  httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
  server_config.server_port = config_.server_port;
  server_config.max_uri_handlers = 8;
  server_config.lru_purge_enable = true;
  server_config.stack_size = 8192;

  if (httpd_start(&server_, &server_config) != ESP_OK) {
    logger_.error("Failed to start HTTP server");
    return false;
  }

  // Root handler
  httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &root);

  // Scan handler
  httpd_uri_t scan = {
      .uri = "/scan", .method = HTTP_GET, .handler = scan_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &scan);

  // Connect handler
  httpd_uri_t connect = {
      .uri = "/connect", .method = HTTP_POST, .handler = connect_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &connect);

  // Complete handler
  httpd_uri_t complete = {
      .uri = "/complete", .method = HTTP_POST, .handler = complete_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &complete);

  // Status handler
  httpd_uri_t status = {
      .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &status);

  // Saved networks handler
  httpd_uri_t saved = {
      .uri = "/saved", .method = HTTP_GET, .handler = saved_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &saved);

  // Delete handler
  httpd_uri_t delete_net = {
      .uri = "/delete", .method = HTTP_POST, .handler = delete_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &delete_net);

  return true;
}

void Provisioning::stop_server() {
  if (server_) {
    httpd_stop(server_);
    server_ = nullptr;
  }
}

void Provisioning::stop_ap() { wifi_ap_.reset(); }

std::string Provisioning::get_ip_address() const { return "192.168.4.1"; }

std::string Provisioning::generate_html() const {
  std::string html = HTML_TEMPLATE;
  size_t pos = html.find("%DEVICE_NAME%");
  if (pos != std::string::npos) {
    html.replace(pos, 13, config_.device_name);
  }
  return html;
}

std::string Provisioning::scan_networks() {
  logger_.info("Starting WiFi scan...");

  // Create a temporary WifiSta for scanning (auto_connect=false so it doesn't try to connect)
  WifiSta::Config scan_config{
      .ssid = "", .password = "", .auto_connect = false, .log_level = Logger::Verbosity::WARN};
  WifiSta temp_sta(scan_config);

  auto ap_records = temp_sta.scan(20);

  if (ap_records.empty()) {
    logger_.info("No networks found");
    return R"({"networks":[]})";
  }

  logger_.info("Found {} networks", ap_records.size());

  // Sort by signal strength
  std::sort(ap_records.begin(), ap_records.end(),
            [](const wifi_ap_record_t &a, const wifi_ap_record_t &b) { return a.rssi > b.rssi; });

  std::string json = R"({"networks":[)";
  for (size_t i = 0; i < ap_records.size(); i++) {
    if (i > 0)
      json += ",";
    std::string ssid = reinterpret_cast<char *>(ap_records[i].ssid);
    json += fmt::format(R"({{"ssid":"{}","rssi":{},"secure":{}}})", json_escape(ssid),
                        ap_records[i].rssi, ap_records[i].authmode != WIFI_AUTH_OPEN);
  }
  json += "]}";
  return json;
}

bool Provisioning::test_connection(const std::string &ssid, const std::string &password) {
  logger_.info("Testing connection to: {}", ssid);

  // Clean up any existing test STA first
  if (test_sta_) {
    logger_.info("Cleaning up previous test STA");
    test_sta_.reset();
    std::this_thread::sleep_for(500ms); // Give WiFi stack time to clean up
  }

  std::atomic<bool> connected{false};
  std::atomic<bool> got_ip{false};
  std::atomic<bool> failed{false};

  WifiSta::Config sta_config{.ssid = ssid,
                             .password = password,
                             .num_connect_retries = 5,
                             .auto_connect = true, // Auto-connect for testing
                             .on_connected =
                                 [&]() {
                                   logger_.info("Connection callback - connected to AP");
                                   connected = true;
                                 },
                             .on_disconnected =
                                 [&]() {
                                   logger_.info("Disconnection callback - failed to connect");
                                   failed = true;
                                 },
                             .on_got_ip =
                                 [&](ip_event_got_ip_t *event) {
                                   logger_.info("Got IP callback - {}.{}.{}.{}",
                                                IP2STR(&event->ip_info.ip));
                                   got_ip = true;
                                 },
                             .log_level = Logger::Verbosity::ERROR};

  logger_.info("Creating test WiFi STA instance");
  test_sta_ = std::make_unique<WifiSta>(sta_config);

  logger_.info("Waiting for connection (max 15 seconds)...");
  // Wait for connection (max 15 seconds)
  auto start = std::chrono::steady_clock::now();
  while (!got_ip && !failed && (std::chrono::steady_clock::now() - start) < 15s) {
    std::this_thread::sleep_for(100ms);
  }

  bool success = got_ip;

  logger_.info("Connection test result: {} (got_ip={}, failed={})", success, got_ip.load(),
               failed.load());

  // Clean up test STA immediately to avoid interfering with AP
  logger_.info("Cleaning up test STA");
  test_sta_.reset();

  // Give WiFi stack time to stabilize
  std::this_thread::sleep_for(200ms);

  return success;
}

// HTTP Handlers
esp_err_t Provisioning::root_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);
  std::string html = prov->generate_html();
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, html.c_str(), html.length());
  return ESP_OK;
}

esp_err_t Provisioning::scan_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);
  std::string json = prov->scan_networks();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

esp_err_t Provisioning::connect_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);

  char buf[512];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  // Parse JSON manually (simple approach)
  std::string data(buf);
  std::string ssid, password;
  bool use_saved = false;

  size_t ssid_pos = data.find("\"ssid\":\"");
  if (ssid_pos != std::string::npos) {
    ssid_pos += 8;
    size_t ssid_end = data.find("\"", ssid_pos);
    ssid = data.substr(ssid_pos, ssid_end - ssid_pos);
  }

  size_t pass_pos = data.find("\"password\":\"");
  if (pass_pos != std::string::npos) {
    pass_pos += 12;
    size_t pass_end = data.find("\"", pass_pos);
    password = data.substr(pass_pos, pass_end - pass_pos);
  }

  size_t saved_pos = data.find("\"use_saved\":true");
  if (saved_pos != std::string::npos) {
    use_saved = true;
  }

  if (ssid.empty()) {
    std::string response = R"({"success":false,"message":"Invalid SSID"})";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response.c_str(), response.length());
    return ESP_OK;
  }

  // If using saved credentials, retrieve password
  if (use_saved) {
    password = prov->get_saved_password(ssid);
  }

  // Test connection
  bool success = prov->test_connection(ssid, password);

  std::string response;
  if (success) {
    response = R"({"success":true})";
    prov->is_provisioned_ = true;
    prov->provisioned_ssid_ = ssid;
    prov->provisioned_password_ = password;

    // Save credentials if not already saved
    if (!use_saved) {
      prov->save_credentials(ssid, password);
    }
  } else {
    response = R"({"success":false,"message":"Failed to connect"})";
  }

  // Send response - don't stop AP or call callback yet
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, response.c_str(), response.length());

  return ESP_OK;
}

esp_err_t Provisioning::complete_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);

  // Mark as completed
  prov->is_completed_ = true;

  // Send response immediately
  httpd_resp_send(req, "OK", 2);

  // Handle completion in background thread
  std::thread([prov]() {
    // Small delay to ensure response transmitted
    std::this_thread::sleep_for(100ms);

    // Call callback with stored credentials
    if (prov->config_.on_provisioned && prov->is_provisioned_) {
      prov->config_.on_provisioned(prov->provisioned_ssid_, prov->provisioned_password_);
    }

    // Auto-shutdown if configured
    if (prov->config_.auto_shutdown_ap) {
      std::this_thread::sleep_for(2s);
      prov->stop();
    }
  }).detach();

  return ESP_OK;
}

esp_err_t Provisioning::status_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);
  std::string response =
      fmt::format(R"({{"active":{},"provisioned":{}}})", prov->is_active_, prov->is_provisioned_);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, response.c_str(), response.length());
  return ESP_OK;
}

esp_err_t Provisioning::saved_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);
  std::string json = prov->get_saved_networks_json();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

esp_err_t Provisioning::delete_handler(httpd_req_t *req) {
  auto *prov = static_cast<Provisioning *>(req->user_ctx);

  char buf[256];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  // Parse SSID from JSON
  std::string data(buf);
  std::string ssid;

  size_t ssid_pos = data.find("\"ssid\":\"");
  if (ssid_pos != std::string::npos) {
    ssid_pos += 8;
    size_t ssid_end = data.find("\"", ssid_pos);
    ssid = data.substr(ssid_pos, ssid_end - ssid_pos);
  }

  if (ssid.empty()) {
    std::string response = R"({"success":false})";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response.c_str(), response.length());
    return ESP_OK;
  }

  prov->delete_credentials(ssid);

  std::string response = R"({"success":true})";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, response.c_str(), response.length());
  return ESP_OK;
}

// Credential storage methods
std::vector<std::string> Provisioning::get_saved_ssids() {
  std::vector<std::string> ssids;

  std::error_code ec;
  espp::NvsHandle nvs("wifi_creds", ec);
  if (ec) {
    return ssids;
  }

  // Get count of saved networks
  uint32_t count = 0;
  nvs.get("count", count, ec);
  if (ec || count == 0) {
    return ssids;
  }

  // Load each SSID by index
  for (uint32_t i = 0; i < count; i++) {
    std::string ssid_key = fmt::format("ssid_{}", i);
    std::string ssid;
    nvs.get(ssid_key, ssid, ec);
    if (!ec && !ssid.empty()) {
      // Remove any trailing null bytes
      ssid.erase(std::find(ssid.begin(), ssid.end(), '\0'), ssid.end());
      if (!ssid.empty()) {
        ssids.push_back(ssid);
      }
    }
  }

  return ssids;
}

std::string Provisioning::get_saved_password(const std::string &ssid) {
  std::error_code ec;
  espp::NvsHandle nvs("wifi_creds", ec);
  if (ec) {
    return "";
  }

  // Find the index for this SSID
  auto ssids = get_saved_ssids();
  auto it = std::find(ssids.begin(), ssids.end(), ssid);
  if (it == ssids.end()) {
    return "";
  }

  size_t index = std::distance(ssids.begin(), it);
  std::string pwd_key = fmt::format("pwd_{}", index);
  std::string password;
  nvs.get(pwd_key, password, ec);

  if (!ec) {
    // Remove any trailing null bytes
    password.erase(std::find(password.begin(), password.end(), '\0'), password.end());
  }

  return ec ? "" : password;
}

void Provisioning::save_credentials(const std::string &ssid, const std::string &password) {
  std::error_code ec;
  espp::NvsHandle nvs("wifi_creds", ec);
  if (ec) {
    logger_.error("Failed to open NVS");
    return;
  }

  logger_.info("Saving credentials for: {}", ssid);

  // Get or create index for this SSID
  auto ssids = get_saved_ssids();
  size_t index = ssids.size();
  auto it = std::find(ssids.begin(), ssids.end(), ssid);
  if (it != ssids.end()) {
    index = std::distance(ssids.begin(), it);
  } else {
    ssids.push_back(ssid);
  }

  // Save SSID and password using index-based keys (keeps keys under 15 chars)
  std::string ssid_key = fmt::format("ssid_{}", index);
  std::string pwd_key = fmt::format("pwd_{}", index);

  nvs.set(ssid_key, ssid, ec);
  if (ec) {
    logger_.error("Failed to save SSID");
    return;
  }

  nvs.set(pwd_key, password, ec);
  if (ec) {
    logger_.error("Failed to save password");
    return;
  }

  // Save count
  nvs.set("count", static_cast<uint32_t>(ssids.size()), ec);
  if (ec) {
    logger_.error("Failed to save count");
    return;
  }

  nvs.commit(ec);
}

void Provisioning::delete_credentials(const std::string &ssid) {
  std::error_code ec;
  espp::NvsHandle nvs("wifi_creds", ec);
  if (ec) {
    logger_.error("Failed to open NVS");
    return;
  }

  logger_.info("Deleting credentials for: {}", ssid);

  // Get current SSIDs
  auto ssids = get_saved_ssids();
  auto it = std::find(ssids.begin(), ssids.end(), ssid);
  if (it == ssids.end()) {
    logger_.warn("SSID not found in saved credentials");
    return;
  }

  size_t index_to_delete = std::distance(ssids.begin(), it);
  ssids.erase(it);

  // Rewrite all credentials with compacted indices
  for (size_t i = index_to_delete; i < ssids.size(); i++) {
    // Move credentials from index i+1 to index i
    std::string old_ssid_key = fmt::format("ssid_{}", i + 1);
    std::string old_pwd_key = fmt::format("pwd_{}", i + 1);
    std::string new_ssid_key = fmt::format("ssid_{}", i);
    std::string new_pwd_key = fmt::format("pwd_{}", i);

    std::string ssid_val, pwd_val;
    nvs.get(old_ssid_key, ssid_val, ec);
    nvs.get(old_pwd_key, pwd_val, ec);

    nvs.set(new_ssid_key, ssid_val, ec);
    nvs.set(new_pwd_key, pwd_val, ec);
  }

  // Erase the last entry (now duplicated)
  if (!ssids.empty()) {
    std::string last_ssid_key = fmt::format("ssid_{}", ssids.size());
    std::string last_pwd_key = fmt::format("pwd_{}", ssids.size());
    nvs.erase(last_ssid_key, ec);
    nvs.erase(last_pwd_key, ec);
  }

  // Update count
  nvs.set("count", static_cast<uint32_t>(ssids.size()), ec);
  if (ec) {
    logger_.error("Failed to update count");
    return;
  }

  nvs.commit(ec);
}

std::string Provisioning::get_saved_networks_json() {
  auto ssids = get_saved_ssids();

  std::string json = R"({"networks":[)";
  for (size_t i = 0; i < ssids.size(); i++) {
    if (i > 0)
      json += ",";
    json += "\"" + json_escape(ssids[i]) + "\"";
  }
  json += "]}";

  return json;
}
