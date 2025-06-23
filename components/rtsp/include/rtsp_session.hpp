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

#include "rtcp_packet.hpp"
#include "rtp_packet.hpp"

namespace espp {
/// Class that reepresents an RTSP session, which is uniquely identified by a
/// session id and sends frame data over RTP and RTCP to the client
class RtspSession : public BaseComponent {
public:
  /// Configuration for the RTSP session
  struct Config {
    std::string server_address; ///< The address of the server
    std::string rtsp_path;      ///< The RTSP path of the session
    std::chrono::duration<float> receive_timeout =
        std::chrono::seconds(5); ///< The timeout for receiving data. Should be > 0.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level of the session
  };

  /// @brief Construct a new RtspSession object
  /// @param control_socket The control socket of the session
  /// @param config The configuration of the session
  explicit RtspSession(std::unique_ptr<espp::TcpSocket> control_socket, const Config &config);

  /// @brief Destroy the RtspSession object
  /// Stop the session task
  ~RtspSession();

  /// @brief Get the session id
  /// @return The session id
  uint32_t get_session_id() const;

  /// @brief Check if the session is closed
  /// @return True if the session is closed, false otherwise
  bool is_closed() const;

  /// Get whether the session is connected
  /// @return True if the session is connected, false otherwise
  bool is_connected() const;

  /// Get whether the session is active
  /// @return True if the session is active, false otherwise
  bool is_active() const;

  /// Mark the session as active
  /// This will cause the server to start sending frames to the client
  void play();

  /// Pause the session
  /// This will cause the server to stop sending frames to the client
  /// @note This does not stop the session, it just pauses it
  /// @note This is useful for when the client is buffering
  void pause();

  /// Teardown the session
  /// This will cause the server to stop sending frames to the client
  /// and close the connection
  void teardown();

  /// Send an RTP packet to the client
  /// @param packet The RTP packet to send
  /// @return True if the packet was sent successfully, false otherwise
  bool send_rtp_packet(const espp::RtpPacket &packet);

  /// Send an RTCP packet to the client
  /// @param packet The RTCP packet to send
  /// @return True if the packet was sent successfully, false otherwise
  bool send_rtcp_packet(const espp::RtcpPacket &packet);

protected:
  /// Send a response to a RTSP request
  /// @param code The response code
  /// @param message The response message
  /// @param sequence_number The sequence number of the request
  /// @param headers The response headers (optional)
  /// @param body The response body (optional)
  /// @return True if the response was sent successfully, false otherwise
  bool send_response(int code, std::string_view message, int sequence_number = -1,
                     std::string_view headers = "", std::string_view body = "");

  /// Handle a RTSP options request
  /// @param request The RTSP request
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_options(std::string_view request);

  /// Handle a RTSP describe request
  /// Create a SDP description and send it back to the client
  /// @param request The RTSP request
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_describe(std::string_view request);

  /// Handle a RTSP setup request
  /// Create a session and send the RTP port numbers back to the client
  /// @param request The RTSP request
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_setup(std::string_view request);

  /// Handle a RTSP play request
  /// After responding to the request, the server should start sending RTP
  /// packets to the client
  /// @param request The request to handle
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_play(std::string_view request);

  /// Handle a RTSP pause request
  /// After responding to the request, the server should stop sending RTP
  /// packets to the client
  /// @param request The request to handle
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_pause(std::string_view request);

  /// Handle an RTSP teardown request
  /// @param request The request to handle
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_teardown(std::string_view request);

  /// Handle an invalid RTSP request
  /// @param request The request to handle
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_invalid_request(std::string_view request);

  /// Handle a single RTSP request
  /// @note Requests are of the form "METHOD RTSP_PATH RTSP_VERSION"
  /// @param request The request to handle
  /// @return True if the request was handled successfully, false otherwise
  bool handle_rtsp_request(std::string_view request);

  /// @brief The task function for the control thread
  /// @param m The mutex to lock when waiting on the condition variable
  /// @param cv The condition variable to wait on
  /// @param task_notified A flag to indicate if the task has been notified
  /// @return True if the task should stop, false otherwise
  bool control_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  /// Generate a new RTSP session id for the client
  /// Session IDs are generated randomly when a client sends a SETUP request and are
  /// used to identify the client in subsequent requests when managing the RTP session.
  /// @return The new session id
  uint32_t generate_session_id();

  /// Parse the RTSP command sequence number from a request
  /// @param request The request to parse
  /// @param cseq The command sequence number (output)
  /// @return True if the request was parsed successfully, false otherwise
  bool parse_rtsp_command_sequence(std::string_view request, int &cseq);

  /// Parse a RTSP path from a request
  /// @param request The request to parse
  /// @return The RTSP path
  std::string_view parse_rtsp_path(std::string_view request);

  /// Parse a RTSP setup request
  /// Looks for the client RTP and RTCP port numbers in the request
  /// and returns them
  /// @param request The request to parse
  /// @param rtsp_path The RTSP path from the request (output)
  /// @param client_rtp_port The client RTP port number (output)
  /// @param client_rtcp_port The client RTCP port number (output)
  /// @return True if the request was parsed successfully, false otherwise
  bool parse_rtsp_setup_request(std::string_view request, std::string_view &rtsp_path,
                                int &client_rtp_port, int &client_rtcp_port);

  std::unique_ptr<espp::TcpSocket> control_socket_;
  espp::UdpSocket rtp_socket_;
  espp::UdpSocket rtcp_socket_;

  uint32_t session_id_;
  bool closed_ = false;
  bool session_active_ = false;

  std::string server_address_;
  std::string rtsp_path_;

  std::string client_address_;
  int client_rtp_port_;
  int client_rtcp_port_;

  std::unique_ptr<Task> control_task_;
};
} // namespace espp
