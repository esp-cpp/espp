#include "rtsp_server.hpp"

#include <thread>

using namespace espp;

namespace {

std::string normalize_rtsp_path(std::string_view path) {
  if (path.empty()) {
    return {};
  }
  while (!path.empty() && path.front() == '/') {
    path.remove_prefix(1);
  }
  return std::string(path);
}

std::string make_legacy_mjpeg_sdp(std::string_view session_path, uint32_t session_id,
                                  std::string_view server_address) {
  return "v=0\r\n"
         "o=- " +
         std::to_string(session_id) + " 1 IN IP4 " + std::string(server_address) +
         "\r\n"
         "s=MJPEG Stream\r\n"
         "i=MJPEG Stream\r\n"
         "t=0 0\r\n"
         "a=control:" +
         std::string(session_path) +
         "\r\n"
         "a=mimetype:string;\"video/x-motion-jpeg\"\r\n"
         "m=video 0 RTP/AVP 26\r\n"
         "c=IN IP4 0.0.0.0\r\n"
         "b=AS:256\r\n"
         "a=control:" +
         std::string(session_path) +
         "\r\n"
         "a=udp-only\r\n";
}

#if defined(ESP_PLATFORM)
constexpr size_t rtp_send_burst_size = 4;
constexpr auto rtp_send_burst_delay = std::chrono::milliseconds(1);
constexpr auto initial_backpressure_cooldown = std::chrono::milliseconds(100);
constexpr auto max_backpressure_cooldown = std::chrono::milliseconds(1000);
constexpr auto base_capture_period = std::chrono::milliseconds(100);
constexpr auto max_capture_period = std::chrono::milliseconds(2000);
#endif

constexpr size_t rtp_header_size = 12;
constexpr size_t mjpeg_header_size = 8;
constexpr size_t quant_header_size = 4;
constexpr size_t num_q_tables = 2;
constexpr size_t q_table_size = 64;
constexpr size_t first_packet_overhead =
    mjpeg_header_size + quant_header_size + num_q_tables * q_table_size;
constexpr size_t other_packet_overhead = mjpeg_header_size;
constexpr int max_rfc2435_dimension = 255 * 8;

void serialize_rtp_header(std::vector<uint8_t> &packet, int payload_type, uint16_t sequence_number,
                          uint32_t timestamp, uint32_t ssrc, bool marker) {
  packet[0] = 0x80; // version 2
  packet[1] = static_cast<uint8_t>((marker ? 0x80 : 0x00) | (payload_type & 0x7f));
  packet[2] = static_cast<uint8_t>((sequence_number >> 8) & 0xff);
  packet[3] = static_cast<uint8_t>(sequence_number & 0xff);
  packet[4] = static_cast<uint8_t>((timestamp >> 24) & 0xff);
  packet[5] = static_cast<uint8_t>((timestamp >> 16) & 0xff);
  packet[6] = static_cast<uint8_t>((timestamp >> 8) & 0xff);
  packet[7] = static_cast<uint8_t>(timestamp & 0xff);
  packet[8] = static_cast<uint8_t>((ssrc >> 24) & 0xff);
  packet[9] = static_cast<uint8_t>((ssrc >> 16) & 0xff);
  packet[10] = static_cast<uint8_t>((ssrc >> 8) & 0xff);
  packet[11] = static_cast<uint8_t>(ssrc & 0xff);
}

uint32_t get_elapsed_timestamp(uint32_t clock_rate) {
  static auto start_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
  return static_cast<uint32_t>(elapsed_ms * clock_rate / 1000);
}

} // namespace

RtspServer::RtspServer(const Config &config)
    : BaseComponent("RTSP Server", config.log_level)
    , server_address_(config.server_address)
    , port_(config.port)
    , path_(normalize_rtsp_path(config.path))
    , rtsp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , max_data_size_(config.max_data_size)
    , accept_task_stack_size_bytes_(config.accept_task_stack_size_bytes == 0
                                        ? Config::default_accept_task_stack_size_bytes
                                        : config.accept_task_stack_size_bytes)
    , session_task_stack_size_bytes_(config.session_task_stack_size_bytes == 0
                                         ? Config::default_session_task_stack_size_bytes
                                         : config.session_task_stack_size_bytes)
    , control_task_stack_size_bytes_(
          config.control_task_stack_size_bytes == 0
              ? RtspSession::Config::default_control_task_stack_size_bytes
              : config.control_task_stack_size_bytes) {}

RtspServer::~RtspServer() { stop(); }

void RtspServer::set_session_log_level(Logger::Verbosity log_level) {
  session_log_level_ = log_level;
}

bool RtspServer::start(const std::chrono::duration<float> &accept_timeout) {
  if (accept_task_ && accept_task_->is_started()) {
    logger_.error("Server is already running");
    return false;
  }

  logger_.info("Starting RTSP server on port {}", port_);

  accept_timeout_ = std::chrono::duration_cast<std::chrono::microseconds>(accept_timeout);

  if (!rtsp_socket_.bind(port_)) {
    logger_.error("Failed to bind to port {}", port_);
    return false;
  }

  int max_pending_connections = 5;
  if (!rtsp_socket_.listen(max_pending_connections)) {
    logger_.error("Failed to listen on port {}", port_);
    return false;
  }

  using namespace std::placeholders;
  accept_task_ = std::make_unique<Task>(Task::Config{
      .callback = std::bind(&RtspServer::accept_task_function, this, _1, _2, _3),
      .task_config =
          {
              .name = "RTSP Accept Task",
              .stack_size_bytes = accept_task_stack_size_bytes_,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!accept_task_->start()) {
    logger_.error("Failed to start RTSP accept task");
    accept_task_.reset();
    rtsp_socket_.close();
    return false;
  }
  return true;
}

void RtspServer::stop() {
  logger_.info("Stopping RTSP server");
  // close the listening socket first so any blocking accept() returns
  rtsp_socket_.close();
  // stop the accept task
  if (accept_task_) {
    accept_task_->stop();
  }
  // stop the session task
  if (session_task_) {
    session_task_->stop();
  }
  // clear the list of sessions
  sessions_.clear();
}

void RtspServer::add_track(const TrackConfig &config) {
  auto state = std::make_shared<TrackState>();
  state->track_id = config.track_id;
  state->packetizer = config.packetizer;
  state->ssrc = generate_ssrc();
  tracks_.push_back(state);
  logger_.info("Added track {} with SSRC {}", config.track_id, state->ssrc);
}

bool RtspServer::has_active_sessions() {
  std::lock_guard<std::mutex> lk(session_mutex_);
  for (const auto &[sid, session_ptr] : sessions_) {
    if (session_ptr && session_ptr->is_active() && !session_ptr->is_closed()) {
      return true;
    }
  }
  return false;
}

std::chrono::milliseconds RtspServer::get_capture_cooldown() {
  std::lock_guard<std::mutex> lk(session_mutex_);
#if defined(ESP_PLATFORM)
  auto now = std::chrono::steady_clock::now();
  if (backpressure_until_ > now) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(backpressure_until_ - now);
  }
#endif
  return std::chrono::milliseconds(0);
}

std::chrono::milliseconds RtspServer::get_recommended_capture_period() {
  std::lock_guard<std::mutex> lk(session_mutex_);
#if defined(ESP_PLATFORM)
  auto period = base_capture_period;
  for (size_t i = 0; i < consecutive_backpressure_failures_; i++) {
    period = std::min(period * 2, max_capture_period);
    if (period == max_capture_period) {
      break;
    }
  }
  auto now = std::chrono::steady_clock::now();
  if (backpressure_until_ > now) {
    period = std::max(
        period, std::chrono::duration_cast<std::chrono::milliseconds>(backpressure_until_ - now));
  }
  return period;
#else
  return std::chrono::milliseconds(0);
#endif
}

void RtspServer::send_frame(int track_id, std::span<const uint8_t> frame_data) {
  if (!has_active_sessions()) {
    return;
  }

  // Find the track
  auto track_it = std::find_if(tracks_.begin(), tracks_.end(),
                               [track_id](const auto &t) { return t->track_id == track_id; });
  std::shared_ptr<TrackState> track = track_it != tracks_.end() ? *track_it : nullptr;
  if (!track) {
    logger_.error("No track with id {} found", track_id);
    return;
  }

  // Packetize the frame using the track's codec-specific packetizer
  auto chunks = track->packetizer->packetize(frame_data);
  auto timestamp = get_elapsed_timestamp(track->packetizer->get_clock_rate());
  std::shared_ptr<TrackState::PacketBatch> batch;
  {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    if (track->recycled_batch) {
      batch = std::move(track->recycled_batch);
    } else {
      batch = std::make_shared<TrackState::PacketBatch>();
    }
  }
  if (batch->packets.size() < chunks.size()) {
    batch->packets.resize(chunks.size());
  }
  batch->count = chunks.size();

  for (size_t i = 0; i < chunks.size(); ++i) {
    auto &chunk = chunks[i];
    auto &packet = batch->packets[i];
    packet.resize(rtp_header_size + chunk.data.size());
    serialize_rtp_header(packet, track->packetizer->get_payload_type(), track->sequence_number++,
                         timestamp, track->ssrc, chunk.marker);
    std::memcpy(packet.data() + rtp_header_size, chunk.data.data(), chunk.data.size());
  }

  {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    if (!track->recycled_batch && track->pending_batch) {
      track->recycled_batch = std::move(track->pending_batch);
    }
    track->pending_batch = std::move(batch);
  }
}

void RtspServer::send_frame(const espp::JpegFrame &frame) { send_frame(frame.get_data()); }

void RtspServer::send_frame(std::span<const uint8_t> frame_data) {
  if (!has_active_sessions()) {
    return;
  }

  // Lazily create default MJPEG track for backward compatibility
  if (!default_mjpeg_track_created_) {
    MjpegPacketizer::Config mjpeg_config;
    mjpeg_config.max_payload_size = max_data_size_;
    auto mjpeg_packetizer = std::make_shared<MjpegPacketizer>(mjpeg_config);
    add_track(TrackConfig{.track_id = 0, .packetizer = mjpeg_packetizer});
    default_mjpeg_track_created_ = true;
  }

  // Find track 0
  auto track_it =
      std::find_if(tracks_.begin(), tracks_.end(), [](const auto &t) { return t->track_id == 0; });
  std::shared_ptr<TrackState> track = track_it != tracks_.end() ? *track_it : nullptr;
  if (!track)
    return;

  if (frame_data.size() < 2) {
    logger_.warn("Skipping short JPEG frame ({} bytes)", frame_data.size());
    return;
  }

  // Use the legacy RtpJpegPacket-based packetization to preserve the
  // exact wire format for existing MJPEG users
  JpegHeader frame_header(frame_data);

  auto width = frame_header.get_width();
  auto height = frame_header.get_height();
  auto q0 = frame_header.get_quantization_table(0);
  auto q1 = frame_header.get_quantization_table(1);

  if (!frame_header.is_valid() || width <= 0 || height <= 0 || width > max_rfc2435_dimension ||
      height > max_rfc2435_dimension || q0.size() != q_table_size || q1.size() != q_table_size) {
    logger_.warn("Skipping invalid JPEG frame: valid={}, size={} bytes, {}x{}, q0={}, q1={}",
                 frame_header.is_valid(), frame_data.size(), width, height, q0.size(), q1.size());
    return;
  }

  // if the frame data is larger than the MTU, then we need to break it up
  // into multiple RTP packets
  size_t num_packets =
      std::max<size_t>(1, (frame_data.size() + max_data_size_ - 1) / max_data_size_);
  logger_.debug("Frame data is {} bytes, breaking into {} packets", frame_data.size(), num_packets);
  auto timestamp = get_elapsed_timestamp(90000);
  std::shared_ptr<TrackState::PacketBatch> batch;
  {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    if (track->recycled_batch) {
      batch = std::move(track->recycled_batch);
    } else {
      batch = std::make_shared<TrackState::PacketBatch>();
    }
  }
  if (batch->packets.size() < num_packets) {
    batch->packets.resize(num_packets);
  }
  batch->count = num_packets;

  for (size_t i = 0; i < num_packets; i++) {
    size_t start_index = i * max_data_size_;
    size_t end_index = std::min<size_t>(start_index + max_data_size_, frame_data.size());
    size_t scan_size = end_index - start_index;
    bool include_q_tables = i == 0;
    size_t payload_size =
        (include_q_tables ? first_packet_overhead : other_packet_overhead) + scan_size;
    auto &packet = batch->packets[i];
    packet.resize(rtp_header_size + payload_size);
    serialize_rtp_header(packet, 26, track->sequence_number++, timestamp, track->ssrc,
                         i == num_packets - 1);

    size_t pos = rtp_header_size;
    packet[pos++] = 0; // type_specific
    packet[pos++] = static_cast<uint8_t>((start_index >> 16) & 0xff);
    packet[pos++] = static_cast<uint8_t>((start_index >> 8) & 0xff);
    packet[pos++] = static_cast<uint8_t>(start_index & 0xff);
    packet[pos++] = 0; // fragment type
    packet[pos++] = static_cast<uint8_t>(include_q_tables ? 128 : 96);
    packet[pos++] = static_cast<uint8_t>(width / 8);
    packet[pos++] = static_cast<uint8_t>(height / 8);

    if (include_q_tables) {
      packet[pos++] = 0;
      packet[pos++] = 0;
      packet[pos++] = 0;
      packet[pos++] = static_cast<uint8_t>(num_q_tables * q_table_size);
      std::memcpy(packet.data() + pos, q0.data(), q_table_size);
      pos += q_table_size;
      std::memcpy(packet.data() + pos, q1.data(), q_table_size);
      pos += q_table_size;
    }

    std::memcpy(packet.data() + pos, frame_data.data() + start_index, scan_size);
  }

  {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    if (!track->recycled_batch && track->pending_batch) {
      track->recycled_batch = std::move(track->pending_batch);
    }
    track->pending_batch = std::move(batch);
  }
}

std::string RtspServer::generate_sdp(const std::string &session_path, uint32_t session_id,
                                     const std::string &server_address) const {
  if (tracks_.empty()) {
    return make_legacy_mjpeg_sdp(session_path, session_id, server_address);
  }

  std::string sdp;
  sdp += "v=0\r\n";
  sdp += fmt::format("o=- {} 1 IN IP4 {}\r\n", session_id, server_address);
  sdp += "s=RTSP Stream\r\n";
  sdp += "t=0 0\r\n";
  sdp += fmt::format("a=control:{}\r\n", session_path);

  for (const auto &track : tracks_) {
    sdp += track->packetizer->get_sdp_media_line() + "\r\n";
    sdp += "c=IN IP4 0.0.0.0\r\n";
    sdp += "b=AS:256\r\n";
    sdp += track->packetizer->get_sdp_media_attributes() + "\r\n";
    sdp += fmt::format("a=control:{}/trackID={}\r\n", session_path, track->track_id);
  }

  return sdp;
}

void RtspServer::reap_closed_sessions() {
  std::lock_guard<std::mutex> lk(session_mutex_);
  for (auto it = sessions_.begin(); it != sessions_.end();) {
    auto &session = it->second;
    if (session->is_closed()) {
      logger_.info("Removing session {}", session->get_session_id());
      it = sessions_.erase(it);
    } else {
      ++it;
    }
  }
}

bool RtspServer::accept_task_function(std::mutex &m, std::condition_variable &cv,
                                      bool &task_notified) {
  if (!rtsp_socket_.is_valid()) {
    return true;
  }

  auto num_ready = rtsp_socket_.select(accept_timeout_);
  if (num_ready <= 0) {
    if (!rtsp_socket_.is_valid()) {
      return true;
    }
    if (task_notified) {
      task_notified = false;
      return true;
    }
    return false;
  }

  // accept a new connection
  auto control_socket = rtsp_socket_.accept();
  if (!control_socket) {
    if (!rtsp_socket_.is_valid()) {
      return true;
    }
    logger_.info("Failed to accept new connection");
    // if we were notified, then we should stop the task
    if (task_notified) {
      task_notified = false;
      return true;
    }
    // do not stop the task, just try to accept another connection
    return false;
  }

  logger_.info("Accepted new connection");
  reap_closed_sessions();

  // create a new session
  auto session = std::make_unique<RtspSession>(
      std::move(control_socket),
      RtspSession::Config{.server_address = fmt::format("{}:{}", server_address_, port_),
                          .rtsp_path = path_,
                          .control_task_stack_size_bytes = control_task_stack_size_bytes_,
                          .sdp_generator =
                              [this](const std::string &session_path, uint32_t session_id,
                                     const std::string &server_address) {
                                return generate_sdp(session_path, session_id, server_address);
                              },
                          .log_level = session_log_level_});
  if (session->is_closed()) {
    logger_.error("Failed to initialize RTSP session {}", session->get_session_id());
    return false;
  }

  // add the session to the list of sessions
  auto session_id = session->get_session_id();
  {
    std::lock_guard<std::mutex> lk(session_mutex_);
    sessions_.emplace(session_id, std::move(session));
  }

  // start the session task if it is not already running
  using namespace std::placeholders;
  if (!session_task_ || !session_task_->is_started()) {
    logger_.info("Starting session task");
    session_task_ = std::make_unique<Task>(Task::Config{
        .callback = std::bind(&RtspServer::session_task_function, this, _1, _2, _3),
        .task_config =
            {
                .name = "RtspSessionTask",
                .stack_size_bytes = session_task_stack_size_bytes_,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });
    if (!session_task_->start()) {
      logger_.error("Failed to start RTSP session task");
      session_task_.reset();
      {
        std::lock_guard<std::mutex> lk(session_mutex_);
        auto it = sessions_.find(session_id);
        if (it != sessions_.end()) {
          it->second->teardown();
          sessions_.erase(it);
        }
      }
      return false;
    }
  }
  // we do not want to stop the task
  return task_notified;
}

bool RtspServer::session_task_function(std::mutex &m, std::condition_variable &cv,
                                       bool &task_notified) {
  // sleep between frames
  {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> lk(m);
    auto stop_requested = cv.wait_for(lk, 10ms, [&task_notified] { return task_notified; });
    task_notified = false;
    if (stop_requested) {
      return true;
    }
  }

  reap_closed_sessions();

  // Collect pending packets from all tracks
  struct TrackPackets {
    std::shared_ptr<TrackState> track;
    int track_id;
    std::shared_ptr<TrackState::PacketBatch> batch;
  };
  std::vector<TrackPackets> all_track_packets;

  for (auto &track : tracks_) {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    if (!track->pending_batch || track->pending_batch->count == 0)
      continue;
    all_track_packets.push_back({track, track->track_id, std::move(track->pending_batch)});
  }

  if (all_track_packets.empty()) {
    // no new frames, do not stop the task
    return false;
  }

#if defined(ESP_PLATFORM)
  auto now = std::chrono::steady_clock::now();
  if (now < backpressure_until_) {
    logger_.warn("Skipping {} pending track packet batches while recovering from RTP backpressure",
                 all_track_packets.size());
    return false;
  }
#endif

  logger_.debug("Sending frame data to clients");

  // for each session in sessions_
  // if the session is active
  // send the latest frame to the client
  bool saw_backpressure = false;
  bool sent_packets = false;
  {
    std::lock_guard<std::mutex> lk(session_mutex_);
    for (auto &[sid, session_ptr] : sessions_) {
      if (!session_ptr->is_active() || session_ptr->is_closed())
        continue;
      bool send_failed = false;
#if defined(ESP_PLATFORM)
      size_t burst_packets_sent = 0;
#endif
      for (auto &tp : all_track_packets) {
        for (size_t i = 0; i < tp.batch->count; ++i) {
          auto &packet = tp.batch->packets[i];
          if (!session_ptr->send_rtp_packet(
                  tp.track_id, std::span<const uint8_t>(packet.data(), packet.size()))) {
            logger_.warn("Dropping remaining RTP packets for session {} after send backpressure",
                         session_ptr->get_session_id());
            send_failed = true;
            saw_backpressure = true;
            break;
          }
          sent_packets = true;
#if defined(ESP_PLATFORM)
          if (++burst_packets_sent >= rtp_send_burst_size) {
            burst_packets_sent = 0;
            std::this_thread::sleep_for(rtp_send_burst_delay);
          }
#endif
        }
        if (send_failed)
          break;
      }
    }
#if defined(ESP_PLATFORM)
    if (saw_backpressure) {
      consecutive_backpressure_failures_++;
      auto cooldown = initial_backpressure_cooldown;
      for (size_t i = 1; i < consecutive_backpressure_failures_; i++) {
        cooldown = std::min(cooldown * 2, max_backpressure_cooldown);
        if (cooldown == max_backpressure_cooldown) {
          break;
        }
      }
      backpressure_until_ = std::chrono::steady_clock::now() + cooldown;
      logger_.warn("Applying RTP backpressure cooldown of {} ms", cooldown.count());
    } else if (sent_packets) {
      if (consecutive_backpressure_failures_ > 0) {
        consecutive_backpressure_failures_--;
      }
      if (consecutive_backpressure_failures_ == 0) {
        backpressure_until_ = {};
      }
    }
#endif
  }
  for (auto &tp : all_track_packets) {
    std::lock_guard<std::mutex> lock(tp.track->packets_mutex);
    if (!tp.track->recycled_batch) {
      tp.batch->count = 0;
      tp.track->recycled_batch = std::move(tp.batch);
    }
  }
  reap_closed_sessions();

  // we do not want to stop the task
  return false;
}
