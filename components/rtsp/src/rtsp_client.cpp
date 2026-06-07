#include "rtsp_client.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

#include "generic_depacketizer.hpp"
#include "h264_depacketizer.hpp"

using namespace espp;

namespace {
constexpr auto monitor_poll_interval = std::chrono::milliseconds(250);

auto now_tick() { return std::chrono::steady_clock::now().time_since_epoch().count(); }

std::string trim_copy(std::string_view value) {
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
    value.remove_prefix(1);
  }
  while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
    value.remove_suffix(1);
  }
  return std::string(value);
}

std::string normalize_rtsp_path(std::string_view path) {
  if (path.empty()) {
    return "/";
  }
  if (path.starts_with("rtsp://")) {
    return std::string(path);
  }

  while (!path.empty() && path.front() == '/') {
    path.remove_prefix(1);
  }
  if (path.empty()) {
    return "/";
  }
  return "/" + std::string(path);
}

std::string make_rtsp_url(std::string_view server_address, int port, std::string_view path) {
  if (path.starts_with("rtsp://")) {
    return std::string(path);
  }
  return "rtsp://" + std::string(server_address) + ":" + std::to_string(port) +
         normalize_rtsp_path(path);
}

std::string get_response_header(std::string_view response, std::string_view header_name) {
  auto header = std::string(header_name) + ": ";
  auto pos = response.find(header);
  if (pos == std::string_view::npos) {
    return {};
  }
  pos += header.size();
  auto end = response.find("\r\n", pos);
  if (end == std::string_view::npos) {
    return {};
  }
  return trim_copy(response.substr(pos, end - pos));
}

std::string get_response_body(std::string_view response) {
  auto body_pos = response.find("\r\n\r\n");
  if (body_pos == std::string_view::npos) {
    return {};
  }
  return std::string(response.substr(body_pos + 4));
}

std::string get_rtsp_origin(std::string_view url) {
  if (!url.starts_with("rtsp://")) {
    return {};
  }
  auto path_pos = url.find('/', 7);
  if (path_pos == std::string_view::npos) {
    return std::string(url);
  }
  return std::string(url.substr(0, path_pos));
}

std::string join_rtsp_url(std::string_view base, std::string_view suffix) {
  auto trimmed_suffix = trim_copy(suffix);
  if (trimmed_suffix.empty()) {
    return std::string(base);
  }
  if (trimmed_suffix.starts_with("rtsp://")) {
    return trimmed_suffix;
  }
  if (trimmed_suffix.front() == '/') {
    auto origin = get_rtsp_origin(base);
    if (!origin.empty()) {
      return origin + trimmed_suffix;
    }
    return trimmed_suffix;
  }

  std::string joined(base);
  while (!joined.empty() && joined.back() == '/') {
    joined.pop_back();
  }
  return joined + "/" + trimmed_suffix;
}

bool iequals(std::string_view lhs, std::string_view rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
        std::tolower(static_cast<unsigned char>(rhs[i]))) {
      return false;
    }
  }
  return true;
}

int parse_track_id(std::string_view control_path, int fallback_track_id) {
  auto pos = control_path.find("trackID=");
  if (pos == std::string_view::npos) {
    return fallback_track_id;
  }
  pos += 8;
  auto end = control_path.find_first_not_of("0123456789", pos);
  auto id_string = control_path.substr(pos, end == std::string_view::npos ? end : end - pos);
  if (id_string.empty()) {
    return fallback_track_id;
  }
  return std::stoi(std::string(id_string));
}

} // namespace

RtspClient::RtspClient(const Config &config)
    : BaseComponent("RtspClient", config.log_level)
    , server_address_(config.server_address)
    , rtsp_port_(config.rtsp_port)
    , rtsp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , rtp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , rtcp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , on_jpeg_frame_(config.on_jpeg_frame)
    , on_frame_(config.on_frame)
    , on_connection_lost_(config.on_connection_lost)
    , cseq_(0)
    , path_(make_rtsp_url(server_address_, rtsp_port_, config.path))
    , base_path_(path_) {
  if (on_jpeg_frame_ || on_frame_) {
    auto mjpeg_depacker = std::make_shared<MjpegDepacketizer>(MjpegDepacketizer::Config{});
    if (on_jpeg_frame_) {
      mjpeg_depacker->set_jpeg_frame_callback(on_jpeg_frame_);
    }
    if (on_frame_) {
      mjpeg_depacker->set_frame_callback([this](std::vector<uint8_t> &&data) {
        int track_id = 0;
        if (auto it = payload_type_to_track_id_.find(26); it != payload_type_to_track_id_.end()) {
          track_id = it->second;
        }
        on_frame_(track_id, std::move(data));
      });
    }
    depacketizers_[26] = mjpeg_depacker;
  }
}

RtspClient::~RtspClient() {
  std::error_code ec;
  disconnect(ec);
  if (ec) {
    logger_.error("Error disconnecting: {}", ec.message());
  }
}

void RtspClient::reset_transport_state() {
  playing_ = false;
  rtp_socket_.stop_receiving();
  rtcp_socket_.stop_receiving();
  rtsp_socket_.close();
  rtsp_socket_.reinit();
  session_id_.clear();
  tracks_.clear();
  payload_type_to_track_id_.clear();
  base_path_ = path_;
  video_port_ = 0;
  video_payload_type_ = 0;
  cseq_ = 0;
  play_started_tick_.store(0, std::memory_order_relaxed);
  last_rtp_packet_tick_.store(0, std::memory_order_relaxed);
}

void RtspClient::start_monitor_task() {
  stop_monitor_task();
  using namespace std::placeholders;
  monitor_task_ = std::make_unique<Task>(Task::Config{
      .callback = std::bind(&RtspClient::monitor_task_fn, this, _1, _2, _3),
      .task_config =
          {
              .name = "RtspClientMon",
              .stack_size_bytes = 4 * 1024,
          },
      .log_level = Logger::Verbosity::WARN,
  });
  monitor_task_->start();
}

void RtspClient::stop_monitor_task() {
  if (monitor_task_ && monitor_task_->is_started()) {
    monitor_task_->stop();
  }
  monitor_task_.reset();
}

bool RtspClient::monitor_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  {
    std::unique_lock<std::mutex> lk(m);
    auto stop_requested =
        cv.wait_for(lk, monitor_poll_interval, [&task_notified] { return task_notified; });
    task_notified = false;
    if (stop_requested) {
      return true;
    }
  }

  if (disconnecting_.load(std::memory_order_relaxed) || !playing_.load(std::memory_order_relaxed)) {
    return false;
  }

  if (!rtsp_socket_.is_connected()) {
    notify_connection_lost("RTSP control socket disconnected");
    return true;
  }

  auto last_tick = last_rtp_packet_tick_.load(std::memory_order_relaxed);
  if (last_tick == 0) {
    auto play_start_tick = play_started_tick_.load(std::memory_order_relaxed);
    if (play_start_tick != 0) {
      auto now = std::chrono::steady_clock::now().time_since_epoch().count();
      auto timeout_ticks = initial_rtp_receive_timeout_.count();
      if (timeout_ticks > 0 && now - play_start_tick > timeout_ticks) {
        notify_connection_lost("Timed out waiting for initial RTP packets");
        return true;
      }
    }
    return false;
  }
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  auto timeout_ticks = rtp_receive_timeout_.count();
  if (timeout_ticks > 0 && now - last_tick > timeout_ticks) {
    notify_connection_lost("Timed out waiting for RTP packets");
    return true;
  }
  return false;
}

void RtspClient::notify_connection_lost(std::string_view reason) {
  if (disconnecting_.load(std::memory_order_relaxed) ||
      connection_lost_reported_.exchange(true, std::memory_order_relaxed)) {
    return;
  }
  logger_.warn("{}", reason);
  reset_transport_state();
  if (on_connection_lost_) {
    on_connection_lost_();
  }
}

std::string
RtspClient::send_request(const std::string &method, const std::string &path,
                         const std::unordered_map<std::string, std::string> &extra_headers,
                         std::error_code &ec) {
  std::string request = method + " " + path + " RTSP/1.0\r\n";
  request += "CSeq: " + std::to_string(cseq_) + "\r\n";
  if (!session_id_.empty()) {
    request += "Session: " + session_id_ + "\r\n";
  }
  for (auto &[key, value] : extra_headers) {
    request += key + ": " + value + "\r\n";
  }
  request += "User-Agent: rtsp-client\r\n";
  request += "Accept: application/sdp\r\n";
  request += "\r\n";

  std::string response;
  auto transmit_config = espp::TcpSocket::TransmitConfig{
      .wait_for_response = true,
      .response_size = 16 * 1024,
      .on_response_callback =
          [&response](auto &response_vector) {
            response.assign(response_vector.begin(), response_vector.end());
          },
      .response_timeout = std::chrono::seconds(5),
  };

  logger_.debug("Request:\n{}", request);
  if (!rtsp_socket_.transmit(request, transmit_config)) {
    ec = std::make_error_code(std::errc::io_error);
    logger_.error("Failed to send request");
    return {};
  }

  logger_.debug("Response:\n{}", response);
  if (parse_response(response, ec)) {
    return response;
  }
  return {};
}

void RtspClient::connect(std::error_code &ec) {
  if (ec) {
    return;
  }

  disconnecting_ = false;
  connection_lost_reported_ = false;
  playing_ = false;
  play_started_tick_.store(0, std::memory_order_relaxed);
  last_rtp_packet_tick_.store(0, std::memory_order_relaxed);
  rtsp_socket_.reinit();
  auto did_connect = rtsp_socket_.connect({
      .ip_address = server_address_,
      .port = static_cast<size_t>(rtsp_port_),
  });
  if (!did_connect) {
    ec = std::make_error_code(std::errc::io_error);
    logger_.error("Failed to connect to {}:{}", server_address_, rtsp_port_);
    return;
  }

  send_request("OPTIONS", "*", {}, ec);
}

void RtspClient::disconnect(std::error_code &ec) {
  disconnecting_ = true;
  stop_monitor_task();
  std::error_code teardown_ec;
  if (rtsp_socket_.is_connected()) {
    teardown(teardown_ec);
  }
  reset_transport_state();
  disconnecting_ = false;
  connection_lost_reported_ = false;

  if (!ec && teardown_ec) {
    ec = teardown_ec;
  }
}

void RtspClient::describe(std::error_code &ec) {
  if (ec) {
    return;
  }

  auto response = send_request("DESCRIBE", path_, {}, ec);
  if (ec) {
    return;
  }

  tracks_.clear();
  payload_type_to_track_id_.clear();
  base_path_ = path_;

  auto content_base = get_response_header(response, "Content-Base");
  if (!content_base.empty()) {
    base_path_ = content_base;
  }

  auto sdp = get_response_body(response);
  if (sdp.empty()) {
    ec = std::make_error_code(std::errc::wrong_protocol_type);
    logger_.error("Invalid sdp");
    return;
  }

  std::istringstream stream(sdp);
  std::string line;
  TrackInfo *current_track = nullptr;
  int next_track_id = 0;

  while (std::getline(stream, line)) {
    auto trimmed = trim_copy(line);
    if (trimmed.empty()) {
      continue;
    }

    if (trimmed.starts_with("m=")) {
      auto first_space = trimmed.find(' ');
      auto last_space = trimmed.rfind(' ');
      if (first_space == std::string::npos || last_space == std::string::npos ||
          first_space == last_space) {
        continue;
      }

      TrackInfo track;
      track.media_type = trimmed.substr(2, first_space - 2);
      track.payload_type = std::stoi(trimmed.substr(last_space + 1));
      track.track_id = next_track_id++;
      tracks_.push_back(std::move(track));
      current_track = &tracks_.back();
      continue;
    }

    if (trimmed.starts_with("a=control:")) {
      auto control_value = trimmed.substr(10);
      auto resolved_path = join_rtsp_url(base_path_, control_value);
      if (current_track) {
        current_track->control_path = resolved_path;
        current_track->track_id = parse_track_id(resolved_path, current_track->track_id);
      } else {
        base_path_ = resolved_path;
      }
      continue;
    }

    if (trimmed.starts_with("a=rtpmap:") && current_track) {
      auto payload_space = trimmed.find(' ');
      if (payload_space == std::string::npos) {
        continue;
      }
      auto slash = trimmed.find('/', payload_space + 1);
      if (slash == std::string::npos) {
        continue;
      }
      auto payload_type = std::stoi(trimmed.substr(9, payload_space - 9));
      if (payload_type != current_track->payload_type) {
        continue;
      }
      current_track->encoding_name = trimmed.substr(payload_space + 1, slash - payload_space - 1);
    }
  }

  if (tracks_.empty()) {
    ec = std::make_error_code(std::errc::wrong_protocol_type);
    logger_.error("Invalid sdp");
    return;
  }

  for (auto &track : tracks_) {
    if (track.control_path.empty()) {
      track.control_path = base_path_;
    }
    payload_type_to_track_id_[track.payload_type] = track.track_id;
  }

  video_port_ = 0;
  video_payload_type_ = tracks_.front().payload_type;
  for (const auto &track : tracks_) {
    if (iequals(track.media_type, "video")) {
      video_payload_type_ = track.payload_type;
      break;
    }
  }

  for (const auto &track : tracks_) {
    if (depacketizers_.contains(track.payload_type)) {
      continue;
    }

    if ((track.payload_type == 26 || iequals(track.encoding_name, "JPEG")) &&
        (on_jpeg_frame_ || on_frame_)) {
      auto mjpeg_depacketizer = std::make_shared<MjpegDepacketizer>(MjpegDepacketizer::Config{});
      if (on_jpeg_frame_) {
        mjpeg_depacketizer->set_jpeg_frame_callback(on_jpeg_frame_);
      }
      if (on_frame_) {
        auto payload_type = track.payload_type;
        mjpeg_depacketizer->set_frame_callback([this, payload_type](std::vector<uint8_t> &&data) {
          int track_id = 0;
          if (auto it = payload_type_to_track_id_.find(payload_type);
              it != payload_type_to_track_id_.end()) {
            track_id = it->second;
          }
          on_frame_(track_id, std::move(data));
        });
      }
      depacketizers_[track.payload_type] = std::move(mjpeg_depacketizer);
      continue;
    }

    if (!on_frame_) {
      continue;
    }

    std::shared_ptr<RtpDepacketizer> depacketizer;
    if (iequals(track.encoding_name, "H264")) {
      depacketizer = std::make_shared<H264Depacketizer>(H264Depacketizer::Config{});
    } else {
      depacketizer = std::make_shared<GenericDepacketizer>(GenericDepacketizer::Config{});
    }

    auto payload_type = track.payload_type;
    depacketizer->set_frame_callback([this, payload_type](std::vector<uint8_t> &&data) {
      int track_id = 0;
      if (auto it = payload_type_to_track_id_.find(payload_type);
          it != payload_type_to_track_id_.end()) {
        track_id = it->second;
      }
      on_frame_(track_id, std::move(data));
    });
    depacketizers_[track.payload_type] = std::move(depacketizer);
  }
}

void RtspClient::setup(std::error_code &ec) {
  using namespace std::chrono_literals;
  static constexpr size_t rtp_port = 5000;
  static constexpr size_t rtcp_port = 5001;
  static constexpr auto receive_timeout = 5s;
  setup(rtp_port, rtcp_port, receive_timeout, ec);
}

void RtspClient::setup(size_t rtp_port, size_t rtcp_port,
                       const std::chrono::duration<float> &receive_timeout, std::error_code &ec) {
  if (ec) {
    return;
  }

  auto transport_header =
      "RTP/AVP;unicast;client_port=" + std::to_string(rtp_port) + "-" + std::to_string(rtcp_port);

  if (tracks_.empty()) {
    send_request("SETUP", path_, {{"Transport", transport_header}}, ec);
  } else {
    for (const auto &track : tracks_) {
      auto setup_path = track.control_path.empty() ? base_path_ : track.control_path;
      send_request("SETUP", setup_path, {{"Transport", transport_header}}, ec);
      if (ec) {
        return;
      }
    }
  }
  if (ec) {
    return;
  }

  init_rtp(rtp_port, receive_timeout, ec);
  init_rtcp(rtcp_port, receive_timeout, ec);
}

void RtspClient::add_depacketizer(int payload_type, std::shared_ptr<RtpDepacketizer> depacketizer) {
  depacketizers_[payload_type] = std::move(depacketizer);
}

void RtspClient::play(std::error_code &ec) {
  if (ec) {
    return;
  }
  send_request("PLAY", base_path_, {}, ec);
  if (!ec) {
    playing_ = true;
    connection_lost_reported_ = false;
    play_started_tick_.store(now_tick(), std::memory_order_relaxed);
    last_rtp_packet_tick_.store(0, std::memory_order_relaxed);
    start_monitor_task();
  }
}

void RtspClient::pause(std::error_code &ec) {
  if (ec) {
    return;
  }
  send_request("PAUSE", base_path_, {}, ec);
  if (!ec) {
    playing_ = false;
    stop_monitor_task();
  }
}

void RtspClient::teardown(std::error_code &ec) {
  playing_ = false;
  stop_monitor_task();
  if (!rtsp_socket_.is_connected()) {
    return;
  }
  send_request("TEARDOWN", base_path_, {}, ec);
}

bool RtspClient::parse_response(const std::string &response_data, std::error_code &ec) {
  if (ec) {
    return false;
  }
  if (response_data.empty()) {
    ec = std::make_error_code(std::errc::no_message);
    logger_.error("Empty response");
    return false;
  }
  if (!response_data.starts_with("RTSP/1.0")) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error("Invalid response: '{}'", response_data);
    return false;
  }

  int status_code = std::stoi(response_data.substr(9, 3));
  std::string status_message = response_data.substr(13, response_data.find("\r\n") - 13);
  if (status_code != 200) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error(std::string("Request failed: ") + status_message);
    return false;
  }

  auto session_pos = response_data.find("Session: ");
  if (session_pos != std::string::npos) {
    session_id_ = response_data.substr(session_pos + 9,
                                       response_data.find("\r\n", session_pos) - session_pos - 9);
  }

  cseq_++;
  return true;
}

void RtspClient::init_rtp(size_t rtp_port, const std::chrono::duration<float> &receive_timeout,
                          std::error_code &ec) {
  if (ec) {
    return;
  }
  logger_.debug("Starting rtp socket");
  auto timeout =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(receive_timeout * 3);
  auto minimum_timeout =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::seconds(15));
  rtp_receive_timeout_ = std::max(timeout, minimum_timeout);
  initial_rtp_receive_timeout_ = rtp_receive_timeout_;
  rtp_socket_.set_receive_timeout(receive_timeout);
  auto rtp_task_config = espp::Task::BaseConfig{
      .name = "Rtp",
      .stack_size_bytes = 16 * 1024,
  };
  auto rtp_config = espp::UdpSocket::ReceiveConfig{
      .port = rtp_port,
      .buffer_size = 2 * 1024,
      .on_receive_callback = std::bind(&RtspClient::handle_rtp_packet, this, std::placeholders::_1,
                                       std::placeholders::_2),
  };
  if (!rtp_socket_.start_receiving(rtp_task_config, rtp_config)) {
    ec = std::make_error_code(std::errc::operation_canceled);
    logger_.error("Failed to start receiving rtp packets");
    return;
  }
}

void RtspClient::init_rtcp(size_t rtcp_port, const std::chrono::duration<float> &receive_timeout,
                           std::error_code &ec) {
  if (ec) {
    return;
  }
  logger_.debug("Starting rtcp socket");
  rtcp_socket_.set_receive_timeout(receive_timeout);
  auto rtcp_task_config = espp::Task::BaseConfig{
      .name = "Rtcp",
      .stack_size_bytes = 6 * 1024,
  };
  auto rtcp_config = espp::UdpSocket::ReceiveConfig{
      .port = rtcp_port,
      .buffer_size = 1 * 1024,
      .on_receive_callback = std::bind(&RtspClient::handle_rtcp_packet, this, std::placeholders::_1,
                                       std::placeholders::_2),
  };
  if (!rtcp_socket_.start_receiving(rtcp_task_config, rtcp_config)) {
    ec = std::make_error_code(std::errc::operation_canceled);
    logger_.error("Failed to start receiving rtcp packets");
    return;
  }
}

std::optional<std::vector<uint8_t>>
RtspClient::handle_rtp_packet(std::vector<uint8_t> &data, const espp::Socket::Info &sender_info) {
  logger_.debug("Got RTP packet of size: {}", data.size());
  play_started_tick_.store(0, std::memory_order_relaxed);
  last_rtp_packet_tick_.store(now_tick(), std::memory_order_relaxed);

  RtpPacket packet(std::span<const uint8_t>(data.data(), data.size()));
  int pt = packet.get_payload_type();

  auto it = depacketizers_.find(pt);
  if (it != depacketizers_.end()) {
    it->second->process_packet(packet);
  } else {
    logger_.debug("No depacketizer registered for payload type {}", pt);
  }

  return {};
}

std::optional<std::vector<uint8_t>>
RtspClient::handle_rtcp_packet(std::vector<uint8_t> &data, const espp::Socket::Info &sender_info) {
  [[maybe_unused]] std::string_view packet(reinterpret_cast<char *>(data.data()), data.size());
  return {};
}
