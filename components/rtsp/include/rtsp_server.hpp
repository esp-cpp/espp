#pragma once

#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "esp_random.h"

#include "logger.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "task.hpp"

#include "rtp_packet.hpp"
#include "rtp_jpeg_packet.hpp"
#include "jpeg_frame.hpp"
#include "rtcp_packet.hpp"

#include "rtsp_session.hpp"

namespace espp {
  /// Class for streaming MJPEG data from a camera using RTSP + RTP
  /// Starts a TCP socket to listen for RTSP connections, and then spawns off a
  /// new RTSP session for each connection.
  /// @see RtspSession
  /// @note This class does not currently send RTCP packets
  ///
  /// \section RtspServer example
  /// \snippet rtsp_example.cpp rtsp_server_example
  class RtspServer {
  public:
    /// @brief Configuration for the RTSP server
    struct Config {
      std::string server_address; ///< The ip address of the server
      int port; ///< The port to listen on
      std::string path; ///< The path to the RTSP stream
      Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< The log level for the RTSP server
    };

    /// @brief Construct an RTSP server
    /// @param config The configuration for the RTSP server
    explicit RtspServer(const Config& config)
      : server_address_(config.server_address)
      , port_(config.port)
      , path_(config.path)
      , rtsp_socket_({.log_level = espp::Logger::Verbosity::WARN})
      , logger_({.tag = "RTSP Server", .level = config.log_level}) {
      // generate a random ssrc
      ssrc_ = esp_random();
    }

    /// @brief Destroy the RTSP server
    ~RtspServer() {
      stop();
    }

    /// @brief Sets the log level for the RTSP sessions created by this server
    /// @note This does not affect the log level of the RTSP server itself
    /// @note This does not change the log level of any sessions that have
    ///       already been created
    /// @param log_level The log level to set
    void set_session_log_level(Logger::Verbosity log_level) {
      session_log_level_ = log_level;
    }

    /// @brief Start the RTSP server
    /// Starts the accept task, session task, and binds the RTSP socket
    /// @return True if the server was started successfully, false otherwise
    bool start() {
      if (accept_task_ && accept_task_->is_started()) {
        logger_.error("Server is already running");
        return false;
      }

      logger_.info("Starting RTSP server on port {}", port_);

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
          .name = "RTSP Accept Task",
          .callback = std::bind(&RtspServer::accept_task_function, this, _1, _2),
          .stack_size_bytes = 6 * 1024,
          .log_level = espp::Logger::Verbosity::WARN,
        });
      accept_task_->start();
      return true;
    }

    /// @brief Stop the FTP server
    /// Stops the accept task, session task, and closes the RTSP socket
    void stop() {
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

    /// @brief Send a frame over the RTSP connection
    /// Converts the full JPEG frame into a series of simplified RTP/JPEG
    /// packets and stores it to be sent over the RTP socket, but does not
    /// actually send it
    /// @note Overwrites any existing frame that has not been sent
    /// @param frame The frame to send
    void send_frame(const JpegFrame& frame) {
      // get the frame scan data
      auto frame_header = frame.get_header();
      auto frame_data = frame.get_scan_data();

      auto width = frame_header.get_width();
      auto height = frame_header.get_height();
      auto q0 = frame_header.get_quantization_table(0);
      auto q1 = frame_header.get_quantization_table(1);

      // if the frame data is larger than the MTU, then we need to break it up
      // into multiple RTP packets
      size_t num_packets = frame_data.size() / 1500 + 1;
      logger_.debug("Frame data is {} bytes, breaking into {} packets", frame_data.size(), num_packets);

      // create num_packets RtpJpegPackets
      // The first packet will have the quantization tables, and the last packet
      // will have the end of image marker and the marker bit set
      std::vector<std::unique_ptr<RtpJpegPacket>> packets;
      packets.reserve(num_packets);
      for (size_t i = 0; i < num_packets; i++) {
        // get the start and end indices for the current packet
        size_t start_index = i * 1500;
        size_t end_index = std::min(start_index + 1500, frame_data.size());

        static const int type_specific = 0;
        static const int fragment_type = 0;
        int offset = i * 1500;

        std::unique_ptr<RtpJpegPacket> packet;
        // if this is the first packet, it has the quantization tables
        if (i == 0) {
          // use the original q value and include the quantization tables
          packet = std::make_unique<RtpJpegPacket>(type_specific, fragment_type,
                                                   128, width, height,
                                                   q0, q1,
                                                   frame_data.substr(start_index, end_index - start_index));
        } else {
          // use a different q value (less than 128) and don't include the
          // quantization tables
          packet = std::make_unique<RtpJpegPacket>(type_specific, offset, fragment_type,
                                                   96, width, height,
                                                   frame_data.substr(start_index, end_index - start_index));
        }

        // set the payload type to 26 (JPEG)
        packet->set_payload_type(26);
        // set the sequence number
        packet->set_sequence_number(sequence_number_++);
        // set the timestamp
        static auto start_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        packet->set_timestamp(timestamp * 90);

        // set the ssrc
        packet->set_ssrc(ssrc_);

        auto mjpeg_header = packet->get_mjpeg_header();
        std::vector<char> mjpeg_vec(mjpeg_header.begin(), mjpeg_header.end());

        // if it's the last packet, set the marker bit
        if (i == num_packets - 1) {
          packet->set_marker(true);
        }

        // make sure the packet header has been serialized
        packet->serialize();

        // add the packet to the list of packets
        packets.emplace_back(std::move(packet));
      }

      // now move the packets into the rtp_packets_ vector
      {
        std::unique_lock<std::mutex> lock(rtp_packets_mutex_);
        // move the new packets into the list
        rtp_packets_ = std::move(packets);
      }
    }

  protected:

    bool accept_task_function(std::mutex& m, std::condition_variable& cv) {
      // accept a new connection
      auto control_socket = rtsp_socket_.accept();
      if (!control_socket) {
        logger_.error("Failed to accept new connection");
        return false;
      }

      logger_.info("Accepted new connection");

      // create a new session
      auto session = std::make_unique<RtspSession>(std::move(control_socket),
                                                   RtspSession::Config{
                                                     .server_address = fmt::format("{}:{}", server_address_, port_),
                                                     .rtsp_path = path_,
                                                     .log_level = session_log_level_
                                                   });

      // add the session to the list of sessions
      auto session_id = session->get_session_id();
      sessions_.emplace(session_id, std::move(session));

      // start the session task if it is not already running
      using namespace std::placeholders;
      if (!session_task_ || !session_task_->is_started()) {
        logger_.info("Starting session task");
        session_task_ = std::make_unique<Task>(Task::Config{
            .name = "RtspSessionTask",
            .callback = std::bind(&RtspServer::session_task_function, this, _1, _2),
            .stack_size_bytes = 6 * 1024,
            .log_level = espp::Logger::Verbosity::WARN,
          });
        session_task_->start();
      }
      // we do not want to stop the task
      return false;
    }

    bool session_task_function(std::mutex& m, std::condition_variable& cv) {
      // sleep between frames
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }

      // when this function returns, the vector of pointers will go out of scope
      // and the pointers will be deleted (which is good because it means we
      // won't send the same frame twice)
      std::vector<std::unique_ptr<RtpJpegPacket>> packets;
      {
        // copy the rtp packets into a local vector
        std::unique_lock<std::mutex> lock(rtp_packets_mutex_);
        if (rtp_packets_.empty()) {
          // if there is not a new frame (no packets), then simply return
          // we do not want to stop the task
          return false;
        }
        // move the packets into the local vector
        packets = std::move(rtp_packets_);
      }

      logger_.debug("Sending frame data to clients");

      // for each session in sessions_
      // if the session is active
      // send the latest frame to the client
      std::lock_guard<std::mutex> lk(session_mutex_);
      for (auto& session : sessions_) {
        [[maybe_unused]] auto session_id = session.first;
        auto& session_ptr = session.second;
        // send the packets to the client
        for (auto& packet : packets) {
          // if the session is not active or is closed, then stop sending
          if (!session_ptr->is_active() || session_ptr->is_closed()) {
            break;
          }
          // if we failed to send a packet, doesn't make sense to send the rest,
          // so stop sending to this client
          if (!session_ptr->send_rtp_packet(*packet)) {
            break;
          }
        }
      }
      // loop over the sessions and erase ones which are closed
      for (auto it = sessions_.begin(); it != sessions_.end();) {
        auto& session = it->second;
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

    uint32_t ssrc_; ///< the ssrc (synchronization source identifier) for the RTP packets
    uint16_t sequence_number_{0}; ///< the sequence number for the RTP packets

    std::string server_address_; ///< the address of the server
    int port_; ///< the port of the RTSP server
    std::string path_; ///< the path of the RTSP server, e.g. rtsp:://<ip>:<port>/<path>

    TcpSocket rtsp_socket_;

    std::mutex rtp_packets_mutex_;
    std::vector<std::unique_ptr<RtpJpegPacket>> rtp_packets_;

    Logger::Verbosity session_log_level_{Logger::Verbosity::WARN};
    std::mutex session_mutex_;
    std::unordered_map<int, std::unique_ptr<RtspSession>> sessions_;

    Logger logger_;
    std::unique_ptr<Task> accept_task_;
    std::unique_ptr<Task> session_task_;
  };
} // namespace espp
