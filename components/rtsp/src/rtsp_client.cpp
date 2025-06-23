#include "rtsp_client.hpp"

using namespace espp;

RtspClient::RtspClient(const Config &config)
    : BaseComponent("RtspClient", config.log_level)
    , server_address_(config.server_address)
    , rtsp_port_(config.rtsp_port)
    , rtsp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , rtp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , rtcp_socket_({.log_level = espp::Logger::Verbosity::WARN})
    , on_jpeg_frame_(config.on_jpeg_frame)
    , cseq_(0)
    , path_("rtsp://" + server_address_ + ":" + std::to_string(rtsp_port_) + config.path) {}

RtspClient::~RtspClient() {
  std::error_code ec;
  disconnect(ec);
  if (ec) {
    logger_.error("Error disconnecting: {}", ec.message());
  }
}

std::string
RtspClient::send_request(const std::string &method, const std::string &path,
                         const std::unordered_map<std::string, std::string> &extra_headers,
                         std::error_code &ec) {
  // send the request
  std::string request = method + " " + path + " RTSP/1.0\r\n";
  request += "CSeq: " + std::to_string(cseq_) + "\r\n";
  if (session_id_.size() > 0) {
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
      .response_size = 1024,
      .on_response_callback =
          [&response](auto &response_vector) {
            response.assign(response_vector.begin(), response_vector.end());
          },
      .response_timeout = std::chrono::seconds(5),
  };
  // NOTE: now this call blocks until the response is received
  logger_.debug("Request:\n{}", request);
  if (!rtsp_socket_.transmit(request, transmit_config)) {
    ec = std::make_error_code(std::errc::io_error);
    logger_.error("Failed to send request");
    return {};
  }

  // TODO: how to keep receiving until we get the full response?
  // if (response.find("\r\n\r\n") != std::string::npos) {
  //   break;
  // }

  // parse the response
  logger_.debug("Response:\n{}", response);
  if (parse_response(response, ec)) {
    return response;
  }
  return {};
}

void RtspClient::connect(std::error_code &ec) {
  // exit early if error code is already set
  if (ec) {
    return;
  }

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

  // send the options request
  send_request("OPTIONS", "*", {}, ec);
}

void RtspClient::disconnect(std::error_code &ec) {
  // send the teardown request
  teardown(ec);
  rtsp_socket_.reinit();
}

void RtspClient::describe(std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return;
  }
  // send the describe request
  auto response = send_request("DESCRIBE", path_, {}, ec);
  if (ec) {
    return;
  }
  // sdp response is of the form:
  //     std::regex sdp_regex("m=video (\\d+) RTP/AVP (\\d+)");
  // parse the sdp response and get the video port without using regex
  // this is a very simple sdp parser that only works for this specific case
  auto sdp_start = response.find("m=video");
  if (sdp_start == std::string::npos) {
    ec = std::make_error_code(std::errc::wrong_protocol_type);
    logger_.error("Invalid sdp");
    return;
  }
  auto sdp_end = response.find("\r\n", sdp_start);
  if (sdp_end == std::string::npos) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error("Incomplete sdp");
    return;
  }
  auto sdp = response.substr(sdp_start, sdp_end - sdp_start);
  auto port_start = sdp.find(" ");
  if (port_start == std::string::npos) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error("Could not find port start");
    return;
  }
  auto port_end = sdp.find(" ", port_start + 1);
  if (port_end == std::string::npos) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error("Could not find port end");
    return;
  }
  auto port = sdp.substr(port_start + 1, port_end - port_start - 1);
  video_port_ = std::stoi(port);
  logger_.debug("Video port: {}", video_port_);
  auto payload_type_start = sdp.find(" ", port_end + 1);
  if (payload_type_start == std::string::npos) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error("Could not find payload type start");
    return;
  }
  auto payload_type = sdp.substr(payload_type_start + 1, sdp.size() - payload_type_start - 1);
  video_payload_type_ = std::stoi(payload_type);
  logger_.debug("Video payload type: {}", video_payload_type_);
}

void RtspClient::setup(std::error_code &ec) {
  // default to rtp and rtcp client ports 5000 and 5001
  using namespace std::chrono_literals;
  static constexpr size_t rtp_port = 5000;
  static constexpr size_t rtcp_port = 5001;
  static constexpr auto receive_timeout = 5s;
  setup(rtp_port, rtcp_port, receive_timeout, ec);
}

void RtspClient::setup(size_t rtp_port, size_t rtcp_port,
                       const std::chrono::duration<float> &receive_timeout, std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return;
  }

  // set up the transport header with the rtp and rtcp ports
  auto transport_header =
      "RTP/AVP;unicast;client_port=" + std::to_string(rtp_port) + "-" + std::to_string(rtcp_port);

  // send the setup request (no response is expected)
  send_request("SETUP", path_, {{"Transport", transport_header}}, ec);
  if (ec) {
    return;
  }

  init_rtp(rtp_port, receive_timeout, ec);
  init_rtcp(rtcp_port, receive_timeout, ec);
}

void RtspClient::play(std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return;
  }
  // send the play request
  send_request("PLAY", path_, {}, ec);
}

void RtspClient::pause(std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return;
  }
  // send the pause request
  send_request("PAUSE", path_, {}, ec);
}

void RtspClient::teardown(std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return;
  }
  // send the teardown request
  send_request("TEARDOWN", path_, {}, ec);
}

bool RtspClient::parse_response(const std::string &response_data, std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return false;
  }
  if (response_data.empty()) {
    ec = std::make_error_code(std::errc::no_message);
    logger_.error("Empty response");
    return false;
  }
  // RTP response is of the form:
  //   std::regex response_regex("RTSP/1.0 (\\d+) (.*)\r\n(.*)\r\n\r\n");
  // parse the response but don't use regex since it may be slow on embedded platforms
  // make sure it matches the expected response format
  if (!response_data.starts_with("RTSP/1.0")) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error("Invalid response: '{}'", response_data);
    return false;
  }
  // parse the status code and message
  int status_code = std::stoi(response_data.substr(9, 3));
  std::string status_message = response_data.substr(13, response_data.find("\r\n") - 13);
  if (status_code != 200) {
    ec = std::make_error_code(std::errc::protocol_error);
    logger_.error(std::string("Request failed: ") + status_message);
    return false;
  }
  // parse the session id
  auto session_pos = response_data.find("Session: ");
  if (session_pos != std::string::npos) {
    session_id_ = response_data.substr(session_pos + 9,
                                       response_data.find("\r\n", session_pos) - session_pos - 9);
  }
  // increment the cseq
  cseq_++;
  return true;
}

void RtspClient::init_rtp(size_t rtp_port, const std::chrono::duration<float> &receive_timeout,
                          std::error_code &ec) {
  // exit early if the error code is set
  if (ec) {
    return;
  }
  logger_.debug("Starting rtp socket");
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
  // exit early if the error code is set
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
  // jpeg frame that we are building
  static std::unique_ptr<JpegFrame> jpeg_frame;

  logger_.debug("Got RTP packet of size: {}", data.size());

  std::string_view packet(reinterpret_cast<char *>(data.data()), data.size());
  // parse the rtp packet
  RtpJpegPacket rtp_jpeg_packet(packet);
  auto frag_offset = rtp_jpeg_packet.get_offset();
  if (frag_offset == 0) {
    // first fragment
    logger_.debug("Received first fragment, size: {}, sequence number: {}",
                  rtp_jpeg_packet.get_data().size(), rtp_jpeg_packet.get_sequence_number());
    if (jpeg_frame) {
      // we already have a frame, this is an error
      logger_.warn("Received first fragment but already have a frame");
      jpeg_frame.reset();
    }
    jpeg_frame = std::make_unique<JpegFrame>(rtp_jpeg_packet);
  } else if (jpeg_frame) {
    logger_.debug("Received middle fragment, size: {}, sequence number: {}",
                  rtp_jpeg_packet.get_data().size(), rtp_jpeg_packet.get_sequence_number());
    // middle fragment
    jpeg_frame->append(rtp_jpeg_packet);
  } else {
    // we don't have a frame to append to but we got a middle fragment
    // this is an error
    logger_.warn("Received middle fragment without a frame");
    return {};
  }

  // check if this is the last packet of the frame (the last packet will have
  // the marker bit set)
  if (jpeg_frame && jpeg_frame->is_complete()) {
    // get the jpeg data
    auto jpeg_data = jpeg_frame->get_data();
    logger_.debug("Received jpeg frame of size: {} B", jpeg_data.size());
    // call the on_jpeg_frame callback
    if (on_jpeg_frame_) {
      on_jpeg_frame_(std::move(jpeg_frame));
    }
    logger_.debug("Sent jpeg frame to callback, now jpeg_frame is nullptr? {}",
                  jpeg_frame == nullptr);
  }
  // return an empty vector to indicate that we don't want to send a response
  return {};
}

std::optional<std::vector<uint8_t>>
RtspClient::handle_rtcp_packet(std::vector<uint8_t> &data, const espp::Socket::Info &sender_info) {
  // receive the rtcp packet
  [[maybe_unused]] std::string_view packet(reinterpret_cast<char *>(data.data()), data.size());
  // TODO: parse the rtcp packet
  // return an empty vector to indicate that we don't want to send a response
  return {};
}
