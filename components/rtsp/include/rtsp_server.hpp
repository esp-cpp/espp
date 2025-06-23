#pragma once

#include "socket_msvc.hpp"

#include <memory>
#include <string>
#include <system_error>
#include <vector>

#if defined(ESP_PLATFORM)
#include <esp_random.h>
#else
#include <random>
#endif

#include "base_component.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"

#include "jpeg_frame.hpp"
#include "rtcp_packet.hpp"
#include "rtp_jpeg_packet.hpp"
#include "rtp_packet.hpp"

#include "rtsp_session.hpp"

namespace espp {
/// Class for streaming MJPEG data from a camera using RTSP + RTP
/// Starts a TCP socket to listen for RTSP connections, and then spawns off a
/// new RTSP session for each connection.
/// @see RtspSession
/// @note This class does not currently send RTCP packets
///
/// \section rtsp_server_ex1 RtspServer example
/// \snippet rtsp_example.cpp rtsp_server_example
class RtspServer : public BaseComponent {
public:
  /// @brief Configuration for the RTSP server
  struct Config {
    std::string server_address; ///< The ip address of the server
    int port;                   ///< The port to listen on
    std::string path;           ///< The path to the RTSP stream
    size_t max_data_size =
        1000; ///< The maximum size of RTP packet data for the MJPEG stream. Frames will be broken
              ///< up into multiple packets if they are larger than this. It seems that 1500 works
              ///< well for sending, but is too large for the esp32 (camera-display) to receive
              ///< properly.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the RTSP server
  };

  /// @brief Construct an RTSP server
  /// @param config The configuration for the RTSP server
  explicit RtspServer(const Config &config);

  /// @brief Destroy the RTSP server
  ~RtspServer();

  /// @brief Sets the log level for the RTSP sessions created by this server
  /// @note This does not affect the log level of the RTSP server itself
  /// @note This does not change the log level of any sessions that have
  ///       already been created
  /// @param log_level The log level to set
  void set_session_log_level(espp::Logger::Verbosity log_level);

  /// @brief Start the RTSP server
  /// Starts the accept task, session task, and binds the RTSP socket
  /// @param accept_timeout The timeout for accepting new connections
  /// @return True if the server was started successfully, false otherwise
  bool start(const std::chrono::duration<float> &accept_timeout = std::chrono::seconds(5));

  /// @brief Stop the FTP server
  /// Stops the accept task, session task, and closes the RTSP socket
  void stop();

  /// @brief Send a frame over the RTSP connection
  /// Converts the full JPEG frame into a series of simplified RTP/JPEG
  /// packets and stores it to be sent over the RTP socket, but does not
  /// actually send it
  /// @note Overwrites any existing frame that has not been sent
  /// @param frame The frame to send
  void send_frame(const espp::JpegFrame &frame);

protected:
  bool accept_task_function(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  bool session_task_function(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  uint32_t ssrc_; ///< the ssrc (synchronization source identifier) for the RTP packets
  uint16_t sequence_number_{0}; ///< the sequence number for the RTP packets

  std::string server_address_; ///< the address of the server
  int port_;                   ///< the port of the RTSP server
  std::string path_;           ///< the path of the RTSP server, e.g. rtsp:://<ip>:<port>/<path>

  espp::TcpSocket rtsp_socket_;

  size_t max_data_size_;

  std::mutex rtp_packets_mutex_;
  std::vector<std::unique_ptr<espp::RtpJpegPacket>> rtp_packets_;

  espp::Logger::Verbosity session_log_level_{espp::Logger::Verbosity::WARN};
  std::mutex session_mutex_;
  std::unordered_map<int, std::unique_ptr<espp::RtspSession>> sessions_;

  std::unique_ptr<Task> accept_task_;
  std::unique_ptr<Task> session_task_;
};
} // namespace espp
