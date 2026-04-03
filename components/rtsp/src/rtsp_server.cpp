#include "rtsp_server.hpp"

using namespace espp;

RtspServer::RtspServer(const Config &config)
    : BaseComponent("RTSP Server", config.log_level)
    , server_address_(config.server_address)
    , port_(config.port)
    , path_(config.path)
    , rtsp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , max_data_size_(config.max_data_size) {}

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

  // ensure the receive timeout is set so that the accept will not block
  // indefinitely and the accept task can be stopped.
  rtsp_socket_.set_receive_timeout(accept_timeout);

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
              .stack_size_bytes = 6 * 1024,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  accept_task_->start();
  return true;
}

void RtspServer::stop() {
  logger_.info("Stopping RTSP server");
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
  // close the RTSP socket
  rtsp_socket_.close();
}

void RtspServer::add_track(const TrackConfig &config) {
  auto state = std::make_shared<TrackState>();
  state->track_id = config.track_id;
  state->packetizer = config.packetizer;
  state->ssrc = generate_ssrc();
  tracks_.push_back(state);
  logger_.info("Added track {} with SSRC {}", config.track_id, state->ssrc);
}

void RtspServer::send_frame(int track_id, std::span<const uint8_t> frame_data) {
  // Find the track
  std::shared_ptr<TrackState> track;
  for (auto &t : tracks_) {
    if (t->track_id == track_id) {
      track = t;
      break;
    }
  }
  if (!track) {
    logger_.error("No track with id {} found", track_id);
    return;
  }

  // Packetize the frame using the track's codec-specific packetizer
  auto chunks = track->packetizer->packetize(frame_data);

  // Wrap each chunk in an RtpPacket
  std::vector<std::unique_ptr<RtpPacket>> packets;
  packets.reserve(chunks.size());

  static auto start_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
  uint32_t timestamp =
      static_cast<uint32_t>(elapsed_ms * track->packetizer->get_clock_rate() / 1000);

  for (auto &chunk : chunks) {
    auto pkt = std::make_unique<RtpPacket>(chunk.data.size());
    pkt->set_version(2);
    pkt->set_marker(chunk.marker);
    pkt->set_payload_type(track->packetizer->get_payload_type());
    pkt->set_sequence_number(track->sequence_number++);
    pkt->set_timestamp(timestamp);
    pkt->set_ssrc(track->ssrc);
    pkt->set_payload(std::span<const uint8_t>(chunk.data.data(), chunk.data.size()));
    pkt->serialize();
    packets.emplace_back(std::move(pkt));
  }

  // Store pending packets for the session task to pick up
  {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    track->pending_packets = std::move(packets);
  }
}

void RtspServer::send_frame(const espp::JpegFrame &frame) {
  // Lazily create default MJPEG track for backward compatibility
  if (!default_mjpeg_track_created_) {
    MjpegPacketizer::Config mjpeg_config;
    mjpeg_config.max_payload_size = max_data_size_;
    auto mjpeg_packetizer = std::make_shared<MjpegPacketizer>(mjpeg_config);
    add_track(TrackConfig{.track_id = 0, .packetizer = mjpeg_packetizer});
    default_mjpeg_track_created_ = true;
  }

  // Find track 0
  std::shared_ptr<TrackState> track;
  for (auto &t : tracks_) {
    if (t->track_id == 0) {
      track = t;
      break;
    }
  }
  if (!track)
    return;

  // Use the legacy RtpJpegPacket-based packetization to preserve the
  // exact wire format for existing MJPEG users
  auto frame_header = frame.get_header();
  auto frame_data = frame.get_data();

  auto width = frame_header.get_width();
  auto height = frame_header.get_height();
  auto q0 = frame_header.get_quantization_table(0);
  auto q1 = frame_header.get_quantization_table(1);

  // if the frame data is larger than the MTU, then we need to break it up
  // into multiple RTP packets
  size_t num_packets = frame_data.size() / max_data_size_ + 1;
  logger_.debug("Frame data is {} bytes, breaking into {} packets", frame_data.size(), num_packets);

  // create num_packets RtpJpegPackets
  // The first packet will have the quantization tables, and the last packet
  // will have the end of image marker and the marker bit set
  std::vector<std::unique_ptr<RtpPacket>> packets;
  packets.reserve(num_packets);
  for (size_t i = 0; i < num_packets; i++) {
    // get the start and end indices for the current packet
    size_t start_index = i * max_data_size_;
    size_t end_index = std::min<size_t>(start_index + max_data_size_, frame_data.size());

    static const int type_specific = 0;
    static const int fragment_type = 0;
    int offset = start_index;

    std::unique_ptr<RtpJpegPacket> packet;
    // if this is the first packet, it has the quantization tables
    if (i == 0) {
      // use the original q value and include the quantization tables
      packet = std::make_unique<espp::RtpJpegPacket>(
          type_specific, fragment_type, 128, width, height, q0, q1,
          frame_data.subspan(start_index, end_index - start_index));
    } else {
      // use a different q value (less than 128) and don't include the
      // quantization tables
      packet = std::make_unique<espp::RtpJpegPacket>(
          type_specific, offset, fragment_type, 96, width, height,
          frame_data.subspan(start_index, end_index - start_index));
    }

    // set the payload type to 26 (JPEG)
    packet->set_payload_type(26);
    // set the sequence number
    packet->set_sequence_number(track->sequence_number++);
    // set the timestamp
    static auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    packet->set_timestamp(timestamp * 90);

    // set the ssrc
    packet->set_ssrc(track->ssrc);

    // if it's the last packet, set the marker bit
    if (i == num_packets - 1) {
      packet->set_marker(true);
    }

    // make sure the packet header has been serialized
    packet->serialize();

    // add the packet to the list of packets
    packets.emplace_back(std::move(packet));
  }

  // store the packets in the track's pending list
  {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    track->pending_packets = std::move(packets);
  }
}

std::string RtspServer::generate_sdp(const std::string &session_path, uint32_t session_id,
                                     const std::string &server_address) const {
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

bool RtspServer::accept_task_function(std::mutex &m, std::condition_variable &cv,
                                      bool &task_notified) {
  // accept a new connection
  auto control_socket = rtsp_socket_.accept();
  if (!control_socket) {
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

  // create a new session
  auto session = std::make_unique<RtspSession>(
      std::move(control_socket),
      RtspSession::Config{.server_address = fmt::format("{}:{}", server_address_, port_),
                          .rtsp_path = path_,
                          .sdp_generator =
                              [this](const std::string &session_path, uint32_t session_id,
                                     const std::string &server_address) {
                                return generate_sdp(session_path, session_id, server_address);
                              },
                          .log_level = session_log_level_});

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
                .stack_size_bytes = 6 * 1024,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });
    session_task_->start();
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

  // Collect pending packets from all tracks
  struct TrackPackets {
    int track_id;
    std::vector<std::unique_ptr<RtpPacket>> packets;
  };
  std::vector<TrackPackets> all_track_packets;

  for (auto &track : tracks_) {
    std::lock_guard<std::mutex> lock(track->packets_mutex);
    if (track->pending_packets.empty())
      continue;
    all_track_packets.push_back({track->track_id, std::move(track->pending_packets)});
  }

  if (all_track_packets.empty()) {
    // no new frames, do not stop the task
    return false;
  }

  logger_.debug("Sending frame data to clients");

  // for each session in sessions_
  // if the session is active
  // send the latest frame to the client
  std::lock_guard<std::mutex> lk(session_mutex_);
  for (auto &[sid, session_ptr] : sessions_) {
    if (!session_ptr->is_active() || session_ptr->is_closed())
      continue;
    for (auto &tp : all_track_packets) {
      for (auto &packet : tp.packets) {
        session_ptr->send_rtp_packet(tp.track_id, *packet);
      }
    }
  }
  // loop over the sessions and erase ones which are closed
  for (auto it = sessions_.begin(); it != sessions_.end();) {
    auto &session = it->second;
    if (session->is_closed()) {
      logger_.info("Removing session {}", session->get_session_id());
      it = sessions_.erase(it);
    } else {
      ++it;
    }
  }

  // we do not want to stop the task
  return false;
}
