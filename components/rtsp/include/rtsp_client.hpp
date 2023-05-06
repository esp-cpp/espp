#pragma once

#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "logger.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"

#include "jpeg_frame.hpp"

namespace espp {

  /// A class for interacting with an RTSP server using RTP and RTCP over UDP
  class RtspClient {
  public:
    using jpeg_frame_callback_t = std::function<void(std::unique_ptr<JpegFrame> jpeg_frame)>;

    /// Configuration for the RTSP client
    struct Config {
      std::string server_address; ///< The server IP Address to connect to
      int rtsp_port{8554}; ///< The port of the RTSP server
      std::string path{"/mjpeg/1"}; ///< The path to the RTSP stream on the server. Will be appended to the server address and port to form the full path of the form "rtsp://<server_address>:<rtsp_port><path>"
      jpeg_frame_callback_t on_jpeg_frame; ///< The callback to call when a JPEG frame is received
      espp::Logger::Verbosity log_level = espp::Logger::Verbosity::INFO; ///< The verbosity of the logger
    };

    /// Constructor
    /// \param config The configuration for the RTSP client
    explicit RtspClient(const Config& config)
      : server_address_(config.server_address),
        rtsp_port_(config.rtsp_port),
        rtsp_socket_({.log_level = espp::Logger::Verbosity::WARN}),
        rtp_socket_({.log_level = espp::Logger::Verbosity::WARN}),
        rtcp_socket_({.log_level = espp::Logger::Verbosity::WARN}),
        on_jpeg_frame_(config.on_jpeg_frame),
        cseq_(0),
        path_("rtsp://" + server_address_ + ":" + std::to_string(rtsp_port_) + config.path),
        logger_({.tag = "RtspClient", .level = config.log_level}) {
    }

    /// Destructor
    /// Disconnects from the RTSP server
    ~RtspClient() {
      std::error_code ec;
      disconnect(ec);
      if (ec) {
        logger_.error("Error disconnecting: {}", ec.message());
      }
    }

    /// Send an RTSP request to the server
    /// \note This is a blocking call
    /// \note This will parse the response and set the session ID if it is
    ///      present in the response. If the response is not a 200 OK, then
    ///      an error code will be set and the response will be returned.
    ///      If the response is a 200 OK, then the response will be returned
    ///      and the error code will be set to success.
    /// \param method The method to use for connecting.
    ///       Options are "OPTIONS", "DESCRIBE", "SETUP", "PLAY", and "TEARDOWN"
    /// \param path The path to the RTSP stream on the server.
    /// \param extra_headers Any extra headers to send with the request. These
    ///      will be added to the request after the CSeq and Session headers. The
    ///      key is the header name and the value is the header value. For example,
    ///      {"Accept": "application/sdp"} will add "Accept: application/sdp" to the
    ///      request. The "User-Agent" header will be added automatically. The
    ///      "CSeq" and "Session" headers will be added automatically.
    ///      The "Accept" header will be added automatically. The "Transport"
    ///      header will be added automatically for the "SETUP" method. Defaults to
    ///      an empty map.
    /// \param ec The error code to set if an error occurs
    /// \return The response from the server
    std::string send_request(const std::string& method, const std::string& path, const std::unordered_map<std::string, std::string>& extra_headers, std::error_code& ec) {
      // send the request
      std::string request = method + " " + path + " RTSP/1.0\r\n";
      request += "CSeq: " + std::to_string(cseq_) + "\r\n";
      if (session_id_.size() > 0) {
        request += "Session: " + session_id_ + "\r\n";
      }
      for (auto& [key, value] : extra_headers) {
        request += key + ": " + value + "\r\n";
      }
      request += "User-Agent: rtsp-client\r\n";
      request += "Accept: application/sdp\r\n";
      request += "\r\n";
      std::string response;
      auto transmit_config = espp::TcpSocket::TransmitConfig{
        .wait_for_response = true,
        .response_size = 1024,
        .on_response_callback = [&response](auto &response_vector) { response.assign(response_vector.begin(), response_vector.end()); },
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

    /// Connect to the RTSP server
    /// Connects to the RTSP server and sends the OPTIONS request.
    /// \param ec The error code to set if an error occurs
    void connect(std::error_code& ec) {
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

    /// Disconnect from the RTSP server
    /// Disconnects from the RTSP server and sends the TEARDOWN request.
    /// \param ec The error code to set if an error occurs
    void disconnect(std::error_code& ec) {
      // send the teardown request
      teardown(ec);
      rtsp_socket_.reinit();
    }

    /// Describe the RTSP stream
    /// Sends the DESCRIBE request to the RTSP server and parses the response.
    /// \param ec The error code to set if an error occurs
    void describe(std::error_code& ec) {
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

    /// Setup the RTSP stream
    /// \note Starts the RTP and RTCP threads.
    /// Sends the SETUP request to the RTSP server and parses the response.
    /// \note The default ports are 5000 and 5001 for RTP and RTCP respectively.
    /// \param ec The error code to set if an error occurs
    void setup(std::error_code& ec) {
      // default to rtp and rtcp client ports 5000 and 5001
      setup(5000, 50001, ec);
    }

    /// Setup the RTSP stream
    /// Sends the SETUP request to the RTSP server and parses the response.
    /// \note Starts the RTP and RTCP threads.
    /// \param rtp_port The RTP client port
    /// \param rtcp_port The RTCP client port
    /// \param ec The error code to set if an error occurs
    void setup(size_t rtp_port, size_t rtcp_port, std::error_code& ec) {
      // exit early if the error code is set
      if (ec) {
        return;
      }

      // set up the transport header with the rtp and rtcp ports
      auto transport_header =
        "RTP/AVP;unicast;client_port="
        + std::to_string(rtp_port) + "-" + std::to_string(rtcp_port);

      // send the setup request
      auto response = send_request("SETUP", path_, {{"Transport", transport_header}}, ec);
      if (ec) {
        return;
      }

      init_rtp(rtp_port, ec);
      init_rtcp(rtcp_port, ec);
    }

    /// Play the RTSP stream
    /// Sends the PLAY request to the RTSP server and parses the response.
    /// \param ec The error code to set if an error occurs
    void play(std::error_code& ec) {
      // exit early if the error code is set
      if (ec) {
        return;
      }
      // send the play request
      send_request("PLAY", path_ , {}, ec);
    }

    /// Pause the RTSP stream
    /// Sends the PAUSE request to the RTSP server and parses the response.
    /// \param ec The error code to set if an error occurs
    void pause(std::error_code& ec) {
      // exit early if the error code is set
      if (ec) {
        return;
      }
      // send the pause request
      send_request("PAUSE", path_, {}, ec);
    }

    /// Teardown the RTSP stream
    /// Sends the TEARDOWN request to the RTSP server and parses the response.
    /// \param ec The error code to set if an error occurs
    void teardown(std::error_code& ec) {
      // exit early if the error code is set
      if (ec) {
        return;
      }
      // send the teardown request
      send_request("TEARDOWN", path_, {}, ec);
    }

  protected:
    /// Parse the RTSP response
    /// \note Parses response data for the following fields:
    ///  - Status code
    ///  - Status message
    ///  - Session
    /// \note Increments the sequence number on success.
    /// \param response_data The response data to parse
    /// \param ec The error code to set if an error occurs
    /// \return True if the response was parsed successfully, false otherwise
    bool parse_response(const std::string& response_data, std::error_code& ec) {
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
      if (response_data.find("RTSP/1.0") != 0) {
        ec = std::make_error_code(std::errc::protocol_error);
        logger_.error("Invalid response");
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
        session_id_ = response_data.substr(session_pos + 9, response_data.find("\r\n", session_pos) - session_pos - 9);
      }
      // increment the cseq
      cseq_++;
      return true;
    }

    /// Initialize the RTP socket
    /// \note Starts the RTP socket task.
    /// \param rtp_port The RTP client port
    /// \param ec The error code to set if an error occurs
    void init_rtp(size_t rtp_port, std::error_code& ec) {
      // exit early if the error code is set
      if (ec) {
        return;
      }
      logger_.debug("Starting rtp socket");
      auto rtp_task_config = espp::Task::Config{
        .name = "Rtp",
        .callback = nullptr,
        .stack_size_bytes = 12 * 1024,
      };
      auto rtp_config = espp::UdpSocket::ReceiveConfig{
        .port = rtp_port,
        .buffer_size = 6 * 1024,
        .on_receive_callback = std::bind(&RtspClient::handle_rtp_packet, this, std::placeholders::_1, std::placeholders::_2),
      };
      if (!rtp_socket_.start_receiving(rtp_task_config, rtp_config)) {
        ec = std::make_error_code(std::errc::operation_canceled);
        logger_.error("Failed to start receiving rtp packets");
        return;
      }
    }

    /// Initialize the RTCP socket
    /// \note Starts the RTCP socket task.
    /// \param rtcp_port The RTCP client port
    /// \param ec The error code to set if an error occurs
    void init_rtcp(size_t rtcp_port, std::error_code& ec) {
      // exit early if the error code is set
      if (ec) {
        return;
      }
      logger_.debug("Starting rtcp socket");
      auto rtcp_task_config = espp::Task::Config{
        .name = "Rtcp",
        .callback = nullptr,
        .stack_size_bytes = 12 * 1024,
      };
      auto rtcp_config = espp::UdpSocket::ReceiveConfig{
        .port = rtcp_port,
        .buffer_size = 6 * 1024,
        .on_receive_callback = std::bind(&RtspClient::handle_rtcp_packet, this, std::placeholders::_1, std::placeholders::_2),
      };
      if (!rtcp_socket_.start_receiving(rtcp_task_config, rtcp_config)) {
        ec = std::make_error_code(std::errc::operation_canceled);
        logger_.error("Failed to start receiving rtcp packets");
        return;
      }
    }

    /// Handle an RTP packet
    /// \note Parses the RTP packet and appends it to the current JPEG frame.
    /// \note If the packet is the last fragment of the JPEG frame, the frame is sent to the on_jpeg_frame callback.
    /// \note This function is called by the RTP socket task.
    /// \param data The data to handle
    /// \param sender_info The sender info
    /// \return Optional data to send back to the sender
    std::optional<std::vector<uint8_t>> handle_rtp_packet(std::vector<uint8_t> &data, const espp::Socket::Info &sender_info) {
      // jpeg frame that we are building
      static std::unique_ptr<JpegFrame> jpeg_frame;

      std::string_view packet(reinterpret_cast<char*>(data.data()), data.size());
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
        logger_.debug("Sent jpeg frame to callback, now jpeg_frame is nullptr? {}", jpeg_frame == nullptr);
      }
      // return an empty vector to indicate that we don't want to send a response
      return {};
    }

    /// Handle an RTCP packet
    /// \note Parses the RTCP packet and sends a response if necessary.
    /// \note This function is called by the RTCP socket task.
    /// \param data The data to handle
    /// \param sender_info The sender info
    /// \return Optional data to send back to the sender
    std::optional<std::vector<uint8_t>> handle_rtcp_packet(std::vector<uint8_t> &data, const espp::Socket::Info &sender_info) {
      // receive the rtcp packet
      std::string_view packet(reinterpret_cast<char*>(data.data()), data.size());
      // TODO: parse the rtcp packet
      // return an empty vector to indicate that we don't want to send a response
      return {};
    }

    std::string server_address_;
    int rtsp_port_;

    espp::TcpSocket rtsp_socket_;
    espp::UdpSocket rtp_socket_;
    espp::UdpSocket rtcp_socket_;

    jpeg_frame_callback_t on_jpeg_frame_{nullptr};

    int cseq_ = 0;
    int video_port_ = 0;
    int video_payload_type_ = 0;
    std::string path_;
    std::string session_id_;

    espp::Logger logger_;
  };

} // namespace espp
