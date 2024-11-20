#pragma once

#include <atomic>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <sys/types.h>

#if defined(ESP_PLATFORM)
#include "esp_random.h"
#else
#include <random>
#endif

#include "base_component.hpp"
#include "file_system.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"

namespace espp {
/// Class representing a client that is connected to the FTP server. This
/// class is used by the FtpServer class to handle the client's requests.
class FtpClientSession : public BaseComponent {
public:
  explicit FtpClientSession(int id, std::string_view local_address,
                            std::unique_ptr<espp::TcpSocket> socket,
                            const std::filesystem::path &root_path)
      : BaseComponent("FtpClientSession " + std::to_string(id))
      , id_(id)
      , local_ip_address_(local_address)
      , current_directory_(root_path)
      , socket_(std::move(socket))
      , passive_socket_({.log_level = Logger::Verbosity::WARN}) {
    logger_.debug("Client session {} created", id_);
    send_welcome_message();
    using namespace std::placeholders;
    task_ = std::make_unique<Task>(Task::Config{
        .callback = std::bind(&FtpClientSession::task_function, this, _1, _2, _3),
        .task_config =
            {
                .name = "FtpClientSession",
                .stack_size_bytes = 1024 * 6,
            },
        .log_level = Logger::Verbosity::WARN,
    });
    task_->start();
  }

  ~FtpClientSession() {
    logger_.debug("Client session {} destroyed", id_);
    task_->stop();
  }

  /// \brief Get the id of the client session.
  /// \return The id of the client session.
  int id() const { return id_; }

  /// \brief Get the current directory of the client session.
  /// \return The current directory of the client session.
  std::filesystem::path current_directory() const { return current_directory_; }

  /// \brief Check if the client session has a valid control connection.
  /// \details This function checks if the client session has a valid control
  ///     connection. A control connection is valid if the control socket is
  ///     valid and connected.
  /// \return True if the control connection is valid, false otherwise.
  bool is_connected() const { return socket_ && socket_->is_connected(); }

  /// \brief Check if the client is using a passive data connection.
  /// \details This function checks if the client is using a passive data
  ///     connection. A client is using a passive data connection if the
  ///     client has sent a PASV command and the session was able to create
  ///     a passive socket.
  /// \return True if the client is using a passive data connection, false
  ///     otherwise.
  bool is_passive_data_connection() const { return is_passive_data_connection_; }

  /// \brief Check if the client session is alive.
  /// \details This function checks if the client session is alive. A client
  ///     session is alive if the task is running.
  /// \return True if the client session is alive, false otherwise.
  bool is_alive() const { return task_ && task_->is_started(); }

protected:
  /// \brief Function which handles requests from the client.
  /// \details This function is called by the task to handle requests from
  ///     the client. This function is called in a loop until it returns
  ///     true, which indicates that the task should stop.
  /// \param m The mutex to use for waiting on the condition variable.
  /// \param cv The condition variable to use for waiting.
  /// \param task_notified A flag to indicate if the task was notified.
  /// \return True if the task should stop, false otherwise.
  bool task_function(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    {
      // delay here
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 1ms, [&task_notified] { return task_notified; });
      task_notified = false;
    }

    if (!socket_) {
      logger_.error("Socket is null, stopping the task");
      // stop the task
      return true;
    }

    if (!socket_->is_connected()) {
      socket_.reset();
      logger_.error("Socket is not connected, stopping the task");
      // stop the task
      return true;
    }

    static constexpr int max_request_size = 1024;
    std::vector<uint8_t> request_data;
    if (!socket_->receive(request_data, max_request_size)) {
      // didn't receive anything, the client may have not sent anything yet
      // or may have disconnected. If it disconnected, the socket will be
      // closed and is_connected() will return false, so we'll handle that
      // case in the next iteration.
      return false;
    }

    if (request_data.size() == 0) {
      logger_.debug("Received empty request, ignoring");
      // don't want to stop the task
      return false;
    }

    logger_.info("Received request of size {}", request_data.size());

    std::string_view request((const char *)request_data.data(), request_data.size());
    if (!handle_request(request)) {
      logger_.error("Failed to handle request");
      return false;
    }

    // don't want to stop the task
    return false;
  }

  /// \brief Send a response to the client.
  /// \details This function sends a response to the client. This function
  ///     uses the control socket and not the data socket.
  /// \param status_code The status code of the response.
  /// \param message The message of the response.
  /// \param multiline Whether or not the response is multiline.
  /// \return True if the response was sent successfully, false otherwise.
  bool send_response(int status_code, std::string_view message, bool multiline = false) {
    std::string response =
        std::to_string(status_code) + (multiline ? "-" : " ") + std::string{message} + "\r\n";
    TcpSocket::TransmitConfig config{}; // default config, no wait for response.
    if (!socket_->transmit(response, config)) {
      logger_.error("Failed to send response");
      return false;
    }
    return true;
  }

  /// \brief Receive data from the client.
  /// \details This function receives data from the client and stores it in
  ///     the given buffer. This function uses the data socket and not the
  ///     control socket, and handles both active and passive mode.
  /// \param buffer The buffer to store the data in.
  /// \param size The size of the buffer.
  /// \return The number of bytes received.
  std::optional<std::vector<uint8_t>> receive_data() {
    if (is_passive_data_connection_) {
      if (!passive_socket_.is_valid()) {
        logger_.error("Passive socket is invalid");
        return {};
      }
      // accept the connection
      data_socket_ = passive_socket_.accept();
      if (!data_socket_) {
        logger_.error("Failed to accept data connection");
        return {};
      }
      if (!data_socket_->is_valid()) {
        logger_.error("Failed to accept data connection");
        return {};
      }
    } else {
      // connect to the client
      if (!data_socket_->connect({.ip_address = data_ip_address_, .port = data_port_})) {
        logger_.error("Failed to connect to client");
        return {};
      }
    }
    // receive the data
    std::vector<uint8_t> data;
    while (true) {
      std::vector<uint8_t> buffer(1024);
      std::size_t received = data_socket_->receive(buffer.data(), buffer.size());
      if (received == 0) {
        break;
      }
      data.insert(data.end(), buffer.begin(), buffer.begin() + received);
    }
    data_socket_->close();
    data_socket_.reset();
    return data;
  }

  /// \brief Send data to the client.
  /// \details This function sends data to the client. This function uses the
  ///     data socket and not the control socket, and handles both active and
  ///     passive mode.
  /// \param data The data to send.
  /// \return True if the data was sent successfully, false otherwise.
  bool send_data(std::string_view data) {
    if (is_passive_data_connection_) {
      if (!passive_socket_.is_valid()) {
        logger_.error("Passive socket is invalid");
        return false;
      }
      // accept the connection
      data_socket_ = passive_socket_.accept();
      if (!data_socket_) {
        logger_.error("Failed to accept data connection");
        return false;
      }
      if (!data_socket_->is_valid()) {
        logger_.error("Failed to accept data connection");
        return false;
      }
    } else {
      // connect to the client
      if (!data_socket_->connect({.ip_address = data_ip_address_, .port = data_port_})) {
        logger_.error("Failed to connect to client");
        return false;
      }
    }
    // send the data
    TcpSocket::TransmitConfig config{};
    bool success = data_socket_->transmit(data, config);
    // close the data socket
    data_socket_->close();
    data_socket_.reset();
    return success;
  }

  /// \brief Receive a file from the client.
  /// \details This function receives a file from the client. This function
  ///     uses the data socket and not the control socket, and handles both
  ///     active and passive mode.
  /// \note This function has to be used instaed of receive_data() because
  ///     receive_data() needs the whole data in memory, which is not possible
  ///     for large files.
  /// \param file_path The path to the file to store the data in.
  /// \return True if the file was received successfully, false otherwise.
  bool receive_file(std::filesystem::path &file_path) {
    if (is_passive_data_connection_) {
      if (!passive_socket_.is_valid()) {
        logger_.error("Passive socket is invalid");
        return false;
      }
      // accept the connection
      data_socket_ = passive_socket_.accept();
      if (!data_socket_) {
        logger_.error("Failed to accept data connection");
        return false;
      }
      if (!data_socket_->is_valid()) {
        logger_.error("Failed to accept data connection");
        return false;
      }
    } else {
      // connect to the client
      if (!data_socket_->connect({.ip_address = data_ip_address_, .port = data_port_})) {
        logger_.error("Failed to connect to client");
        return false;
      }
    }
    // open the file
    std::ofstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      logger_.error("Failed to open file");
      return false;
    }

    auto start = std::chrono::high_resolution_clock::now();
    size_t total_size = 0;
    // receive the data
    while (true) {
      std::vector<uint8_t> buffer(1024);
      std::size_t received = data_socket_->receive(buffer.data(), buffer.size());
      if (received == 0) {
        break;
      }
      total_size += received;
      // write it to the filen
      logger_.debug("Writing {} bytes", received);
      file.write(reinterpret_cast<char *>(buffer.data()), received);
    }
    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(end - start).count();
    logger_.info("Received {} bytes in {:.2f} seconds ({:.2f} bytes/s)", total_size, elapsed,
                 total_size / elapsed);
    file.close();
    file.flush();
    data_socket_->close();
    data_socket_.reset();
    return true;
  }

  /// \brief Send a file to the client.
  /// \details This function sends a file to the client. This function uses
  ///     the data socket and not the control socket, and handles both active
  ///     and passive mode.
  /// \note This function has to be used instaed of send_data() because
  ///     send_data() needs the whole data in memory, which is not possible
  ///     for large files.
  /// \param file_path The path to the file to send.
  /// \return True if the file was sent successfully, false otherwise.
  bool send_file(std::filesystem::path &file_path) {
    if (is_passive_data_connection_) {
      if (!passive_socket_.is_valid()) {
        logger_.error("Passive socket is invalid");
        return false;
      }
      // accept the connection
      data_socket_ = passive_socket_.accept();
      if (!data_socket_) {
        logger_.error("Failed to accept data connection");
        return false;
      }
      if (!data_socket_->is_valid()) {
        logger_.error("Failed to accept data connection");
        return false;
      }
    } else {
      // connect to the client
      if (!data_socket_->connect({.ip_address = data_ip_address_, .port = data_port_})) {
        logger_.error("Failed to connect to client");
        return false;
      }
    }

    TcpSocket::TransmitConfig config{};
    // open the file
    std::ifstream file(file_path, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      logger_.error("Failed to open file");
      return false;
    }
    // get the file size
    file.seekg(0, std::ios::end);
    std::size_t file_size = file.tellg();
    logger_.debug("File size: {}", file_size);
    file.seekg(0, std::ios::beg);
    // send the file
    auto start = std::chrono::high_resolution_clock::now();
    size_t total_size = 0;
    static constexpr std::size_t buffer_size = 1024;
    std::unique_ptr<char[]> buffer(new char[buffer_size]);
    while (true) {
      file.read(buffer.get(), buffer_size);
      std::size_t bytes_read = file.gcount();
      if (bytes_read == 0) {
        break;
      }
      std::string_view data(buffer.get(), bytes_read);
      if (!data_socket_->transmit(data, config)) {
        logger_.error("Failed to send file");
        return false;
      }
      total_size += bytes_read;
    }
    file.close();
    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(end - start).count();
    logger_.info("Sent {} bytes in {:.2f} seconds ({:.2f} bytes/s)", total_size, elapsed,
                 total_size / elapsed);

    // close the data socket
    data_socket_->close();
    data_socket_.reset();
    return true;
  }

  bool parse_ftp_command(std::string_view request, std::string_view &command,
                         std::string_view &arguments) {
    // parses the command from the FTP client's request. The command is the
    // first word in the request. The command is case insensitive.
    // The command is followed by a space and then the arguments.
    // The arguments are separated by spaces.
    // The command and arguments are all ASCII strings.
    if (request.empty()) {
      return false;
    }
    auto command_end = request.find(' ');
    // if there is a space, then there are arguments
    if (command_end != std::string_view::npos) {
      // the command is the first word
      command = request.substr(0, command_end);
      // the arguments are the rest of the request
      arguments = request.substr(command_end + 1);
      return true;
    }
    // if there is no space, then there are no arguments
    // no arguments, strip the \r\n from the end
    auto request_end = request.find("\r\n");
    if (request_end == std::string_view::npos) {
      return false;
    }
    // the command is the first word
    command = request.substr(0, request_end);
    // there are no arguments
    arguments = "";
    return true;
  }

  /// \brief Handle a request.
  /// \param request The request to handle.
  /// \return True if the request was handled, false otherwise.
  bool handle_request(std::string_view request) {
    logger_.info("Handling request: {}", request);
    std::string_view command;
    std::string_view arguments;
    if (!parse_ftp_command(request, command, arguments)) {
      logger_.error("Failed to parse FTP command");
      return false;
    }
    if (command == "USER") {
      return handle_user(arguments);
    }
    if (command == "PASS") {
      return handle_pass(arguments);
    }
    if (command == "SYST") {
      return handle_syst(arguments);
    }
    if (command == "FEAT") {
      return handle_feat(arguments);
    }
    if (command == "PWD") {
      return handle_pwd(arguments);
    }
    if (command == "CWD") {
      return handle_cwd(arguments);
    }
    if (command == "CDUP") {
      return handle_cdup(arguments);
    }
    if (command == "TYPE") {
      return handle_type(arguments);
    }
    if (command == "PASV") {
      return handle_pasv(arguments);
    }
    if (command == "PORT") {
      return handle_port(arguments);
    }
    if (command == "LIST") {
      return handle_list(arguments);
    }
    if (command == "SIZE") {
      return handle_size(arguments);
    }
    if (command == "RETR") {
      return handle_retr(arguments);
    }
    if (command == "STOR") {
      return handle_stor(arguments);
    }
    if (command == "DELE") {
      return handle_dele(arguments);
    }
    if (command == "MKD") {
      return handle_mkd(arguments);
    }
    if (command == "RMD") {
      return handle_rmd(arguments);
    }
    if (command == "RNFR") {
      return handle_rnfr(arguments);
    }
    if (command == "RNTO") {
      return handle_rnto(arguments);
    }
    if (command == "NOOP") {
      return handle_noop(arguments);
    }
    if (command == "QUIT") {
      return handle_quit(arguments);
    }
    return handle_unknown(request);
  }

  /// \brief Send a welcome message to the client.
  /// \details This function sends a welcome message to the client.
  /// \return True if the welcome message was sent successfully, false
  bool send_welcome_message() { return send_response(220, "Welcome to espp FTP server"); }

  /// \brief Handle the USER command.
  /// The USER command is used to specify the user name (USER is a
  /// synonym for the USER NAME command). The argument field is a
  /// Telnet string identifying the user.
  /// \param arguments The arguments to the USER command.
  /// \return True if the command was handled, false otherwise.
  bool handle_user(std::string_view arguments) {
    logger_.info("Handling user: {}", arguments);
    // parse the username
    auto username_end = arguments.find("\r\n");
    if (username_end == std::string_view::npos) {
      logger_.error("Failed to parse username");
      return false;
    }
    username_ = arguments.substr(0, username_end);
    logger_.info("Username: {}", username_);
    return send_response(331, "User name okay, need password.");
  }

  /// \brief Handle the PASS command.
  /// The PASS command is used to specify the user's password. The
  /// argument field is a Telnet string specifying the user's
  /// password. This command must be immediately preceded by the
  /// user name command, and, for some sites, completes the user's
  /// identification for access control.
  /// \param arguments The arguments to the PASS command.
  /// \return True if the command was handled, false otherwise.
  bool handle_pass(std::string_view arguments) {
    logger_.info("Handling pass: {}", arguments);
    // parse the password
    auto password_end = arguments.find("\r\n");
    if (password_end == std::string_view::npos) {
      logger_.error("Failed to parse password");
      return false;
    }
    password_ = arguments.substr(0, password_end);

    /*
    // TODO: check the username and password
    if (username_ == "anonymous" && password_ == "anonymous") {
      return send_response(230, "User logged in, proceed.");
    } else {
      return send_response(530, "Incorrect credentials.");
    }
    */

    return send_response(230, "User logged in, proceed.");
  }

  /// \brief Handle the SYST command.
  /// \details The SYST command prints the system type.
  /// \param arguments The arguments to the SYST command.
  /// \return True if the command was handled, false otherwise.
  bool handle_syst(std::string_view arguments) {
    logger_.info("Handling syst: {}", arguments);
    // TODO: print the system type
    return send_response(215, "UNIX Type: L8");
  }

  /// \brief Handle the FEAT command.
  /// \details The FEAT command lists the features supported by the server.
  /// \param arguments The arguments to the FEAT command.
  /// \return True if the command was handled, false otherwise.
  bool handle_feat(std::string_view arguments) {
    logger_.info("Handling feat: {}", arguments);
    // format the message containing the features
    std::string message = "Features:\r\n";
    message += " USER\r\n";
    message += " PASS\r\n";
    message += " SYST\r\n";
    message += " FEAT\r\n";
    message += " PWD\r\n";
    message += " CWD\r\n";
    message += " CDUP\r\n";
    message += " TYPE\r\n";
    message += " PASV\r\n";
    message += " PORT\r\n";
    message += " LIST\r\n";
    message += " SIZE\r\n";
    message += " RETR\r\n";
    message += " STOR\r\n";
    message += " DELE\r\n";
    message += " MKD\r\n";
    message += " RMD\r\n";
    message += " RNFR\r\n";
    message += " RNTO\r\n";
    message += " NOOP\r\n";
    message += " QUIT\r\n";
    bool success = send_response(211, message, true);
    return success && send_response(211, "End");
  }

  /// \brief Handle the PWD command.
  /// \details The PWD command prints the current directory.
  /// \param arguments The arguments to the PWD command.
  /// \return True if the command was handled, false otherwise.
  bool handle_pwd(std::string_view arguments) {
    logger_.info("Handling pwd: {}", arguments);
    // make the message containing the current directory
    std::string message =
        std::string{"\""} + current_directory_.string() + "\" is the current directory";
    return send_response(257, message);
  }

  /// \brief Handle the CWD command.
  /// \details The CWD command changes the current directory.
  /// \param arguments The arguments to the CWD command.
  /// \return True if the command was handled, false otherwise.
  bool handle_cwd(std::string_view arguments) {
    logger_.info("Handling cwd: {}", arguments);
    // parse the directory
    auto directory_end = arguments.find("\r\n");
    if (directory_end == std::string_view::npos) {
      logger_.error("Failed to parse directory");
      return false;
    }
    std::string_view directory = arguments.substr(0, directory_end);
    // change the current directory
    std::filesystem::path new_directory(directory);
    if (new_directory.is_relative()) {
      logger_.debug("Relative path");
      new_directory = current_directory_ / new_directory;
    }
    if (!std::filesystem::exists(new_directory)) {
      return send_response(550, "Directory does not exist.");
    }
    if (!std::filesystem::is_directory(new_directory)) {
      return send_response(550, "Not a directory.");
    }
    current_directory_ = new_directory;
    logger_.debug("Current directory: {}", current_directory_.string());
    return send_response(250, "Directory successfully changed.");
  }

  /// \brief Handle the CDUP command.
  /// \details The CDUP command changes the current directory to the parent
  ///     directory.
  /// \param arguments The arguments to the CDUP command.
  /// \return True if the command was handled, false otherwise.
  bool handle_cdup(std::string_view arguments) {
    logger_.info("Handling cdup: {}", arguments);
    // change the current directory
    current_directory_ = current_directory_.parent_path();
    logger_.debug("Current directory: {}", current_directory_.string());
    return send_response(250, "Directory successfully changed.");
  }

  /// \brief Handle the TYPE command.
  /// \details The TYPE command specifies the representation type as described
  ///     in the Section on Data Representation and Storage. Several types
  ///     take a second parameter. The first parameter is denoted by a single
  ///     Telnet character, as is the second Format parameter for ASCII and
  ///     EBCDIC; the second parameter for local byte is a decimal integer to
  ///     indicate Bytesize. The parameters are separated by a <SP> (Space,
  ///     ASCII code 32).
  /// \param arguments The arguments to the TYPE command.
  /// \return True if the command was handled, false otherwise.
  bool handle_type(std::string_view arguments) {
    logger_.info("Handling type: {}", arguments);
    // parse the type
    auto type_end = arguments.find("\r\n");
    if (type_end == std::string_view::npos) {
      logger_.error("Failed to parse type");
      return false;
    }
    std::string_view type = arguments.substr(0, type_end);
    // handle the ASCII (A) type
    if (type == "A") {
      return send_response(200, "Switching to ASCII mode.");
    }
    // handle the binary (I) type
    if (type == "I") {
      return send_response(200, "Switching to Binary mode.");
    }
    // handle the EBCDIC (E) type
    if (type == "E") {
      return send_response(200, "Switching to EBCDIC mode.");
    }
    return send_response(504, "Command not implemented for that parameter.");
  }

  /// \brief Handle the PASV command.
  /// \details The PASV command requests the server to "listen" on a data
  ///     port (which is not its default data port) and to wait for a
  ///     connection rather than initiate one upon receipt of a transfer
  ///     command. The response to this command includes the host and port
  ///     address this server is listening on.
  /// \param arguments The arguments to the PASV command.
  /// \return True if the command was handled, false otherwise.
  bool handle_pasv(std::string_view arguments) {
    logger_.info("Handling pasv: {}", arguments);
#if defined(ESP_PLATFORM)
    int port = esp_random() % 10000 + 1024;
#else
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, std::numeric_limits<int>::max());
    int port = dis(gen) % 10000 + 1024;
#endif
    // randomly select a port number
    logger_.debug("Selected port: {}", port);
    // ensure that the socket is closed and ready to be used again
    passive_socket_.reinit();
    // ensure the data connection is closed
    data_socket_.reset();
    // create the data connection socket and listen on it
    if (!passive_socket_.bind(port)) {
      logger_.error("Failed to bind data socket");
      return send_response(425, "Failed to create data connection.");
    }
    int max_pending_connections = 1;
    if (!passive_socket_.listen(max_pending_connections)) {
      logger_.error("Failed to listen on data socket");
      return send_response(425, "Failed to create data connection.");
    }
    // replace the periods with commas
    std::string ip_address = local_ip_address_;
    std::replace(ip_address.begin(), ip_address.end(), '.', ',');
    // format the response message with the address and port
    auto message =
        fmt::format("Entering Passive Mode ({},{},{})", ip_address, port / 256, port % 256);
    logger_.debug("Response message: {}", message);
    // set the data connection mode to passive
    is_passive_data_connection_ = true;
    // send the response
    return send_response(227, message);
  }

  /// \brief Handle the PORT command.
  /// \details The PORT command is used to specify the address and port to
  ///     which the server should connect for the next data transfer.
  /// \param arguments The arguments to the PORT command.
  /// \return True if the command was handled, false otherwise.
  bool handle_port(std::string_view arguments) {
    logger_.info("Handling port: {}", arguments);
    // check if the arguments are valid
    std::array<std::string_view, 6> parts;
    auto part = arguments.begin();
    for (int i = 0; i < 5; ++i) {
      auto comma = std::find(part, arguments.end(), ',');
      if (comma == arguments.end()) {
        logger_.error("Failed to parse port");
        return send_response(501, "Syntax error in parameters or arguments.");
      }
      parts[i] = std::string_view(&*part, std::distance(part, comma));
      part = comma + 1;
    }
    parts[5] = std::string_view(&*part, std::distance(part, arguments.end()));
    // parse the IP address
    data_ip_address_.clear();
    for (int i = 0; i < 4; ++i) {
      data_ip_address_ += parts[i];
      if (i < 3) {
        data_ip_address_ += '.';
      }
    }
    // parse the port
    data_port_ = std::stoi(std::string{parts[4]}) * 256 + std::stoi(std::string{parts[5]});
    logger_.info("IP address: {}", data_ip_address_);
    logger_.info("Port: {}", data_port_);
    // create the data socket and connect to it
    data_socket_.reset();
    data_socket_ =
        std::make_unique<TcpSocket>(TcpSocket::Config{.log_level = Logger::Verbosity::WARN});
    is_passive_data_connection_ = false;
    return send_response(200, "PORT command successful.");
  }

  /// @brief Handle the LIST command
  /// The LIST command lists the contents of the current directory.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_list(std::string_view arguments) {
    logger_.info("Handling list: {}", arguments);
    if (!send_response(150, "File status okay; about to open data connection.")) {
      logger_.error("Failed to send response");
      return false;
    }

    // get the directory listing
    espp::FileSystem::ListConfig config{};
    std::string directory_listing =
        espp::FileSystem::get().list_directory(current_directory_, config);

    logger_.debug("Directory listing:\n{}", directory_listing);
    if (!send_data(directory_listing)) {
      logger_.error("Failed to send directory listing");
      return send_response(426, "Connection closed; transfer aborted.");
    }
    return send_response(226, "Closing data connection. Requested file action successful.");
  }

  /// @brief Handle the SIZE command
  /// The SIZE command returns the size of a file.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_size(std::string_view arguments) {
    logger_.info("Handling size: {}", arguments);
    // get the path of the file to retrieve
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (!std::filesystem::exists(full_path)) {
      return send_response(550, "File does not exist.");
    }
    if (!std::filesystem::is_regular_file(full_path)) {
      return send_response(550, "Not a regular file.");
    }
    // get the size of the file
    std::error_code error;
    auto size = std::filesystem::file_size(full_path.string().data(), error);
    if (error) {
      logger_.error("Failed to get file size: {}", error.message());
      return send_response(550, "Failed to get file size.");
    }
    // send the response
    return send_response(213, std::to_string(size));
  }

  /// @brief Handle the RETR command
  /// The RETR command retrieves a file from the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_retr(std::string_view arguments) {
    logger_.info("Handling retr: {}", arguments);
    // get the path of the file to retrieve
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (!std::filesystem::exists(full_path)) {
      return send_response(550, "File does not exist.");
    }
    if (!std::filesystem::is_regular_file(full_path)) {
      return send_response(550, "Not a regular file.");
    }
    if (!send_response(150, "File status okay; about to open data connection.")) {
      logger_.error("Failed to send response");
      return false;
    }
    // send the file over the data connection
    if (!send_file(full_path)) {
      logger_.error("Failed to send file");
      return send_response(426, "Connection closed; transfer aborted.");
    }
    return send_response(226, "Closing data connection. Requested file action successful.");
  }

  /// @brief Handle the STOR command
  /// The STOR command stores a file on the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_stor(std::string_view arguments) {
    logger_.info("Handling stor: {}", arguments);
    // get the path of the file to store
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    // NOTE: we don't check if the file exists, because we want to overwrite it
    if (!send_response(150, "File status okay; about to open data connection.")) {
      logger_.error("Failed to send response");
      return false;
    }
    // receive the file over the data connection
    if (!receive_file(full_path)) {
      logger_.error("Failed to receive file");
      return send_response(426, "Connection closed; transfer aborted.");
    }
    // and send the response
    return send_response(226, "Closing data connection. Requested file action successful.");
  }

  /// @brief Handle the QUIT command
  /// The QUIT command closes the connection.
  /// @note After sending the response, the control connection is closed and the
  ///       control connection socket is cleaned up. Any further commands will
  ///       not be handled and the client will have to reconnect.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_quit(std::string_view arguments) {
    logger_.info("Handling quit: {}", arguments);
    bool success = send_response(221, "Service closing control connection.");
    // close the control connection
    socket_->close();
    return success;
  }

  /// @brief Handle the DELE command
  /// The DELE command deletes a file on the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_dele(std::string_view arguments) {
    logger_.info("Handling dele: {}", arguments);
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (!std::filesystem::exists(full_path)) {
      return send_response(550, "File does not exist.");
    }
    if (!std::filesystem::is_regular_file(full_path)) {
      return send_response(550, "Not a regular file.");
    }
    std::error_code ec;
    espp::FileSystem::get().remove(full_path, ec);
    if (ec) {
      logger_.error("Failed to delete file: {}", ec.message());
      return send_response(550, fmt::format("Failed to delete file: {}", ec.message()));
    }
    return send_response(250, "Requested file action okay, completed.");
  }

  /// @brief Handle the RMD command
  /// The RMD command removes a directory on the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_rmd(std::string_view arguments) {
    logger_.info("Handling rmd: {}", arguments);
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (!std::filesystem::exists(full_path)) {
      return send_response(550, "File does not exist.");
    }
    if (!std::filesystem::is_directory(full_path)) {
      return send_response(550, "Not a directory.");
    }
    std::error_code ec;
    espp::FileSystem::get().remove(full_path, ec);
    if (ec) {
      logger_.error("Failed to delete directory: {}", ec.message());
      return send_response(550, fmt::format("Failed to delete directory: {}", ec.message()));
    }
    return send_response(250, "Requested file action okay, completed.");
  }

  /// @brief Handle the MKD command
  /// The MKD command creates a directory on the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_mkd(std::string_view arguments) {
    logger_.info("Handling mkd: {}", arguments);
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (std::filesystem::exists(full_path)) {
      return send_response(550, "File already exists.");
    }
    std::filesystem::create_directory(full_path);
    std::string message = full_path.string() + " created.";
    return send_response(257, message);
  }

  /// @brief Handle the RNFR command
  /// The RNFR command renames a file or directory on the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_rnfr(std::string_view arguments) {
    logger_.info("Handling rnfr: {}", arguments);
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (!std::filesystem::exists(full_path)) {
      return send_response(550, "File does not exist.");
    }
    rename_from_ = full_path.string();
    return send_response(350, "File exists, ready for destination name (RNTO).");
  }

  /// @brief Handle the RNTO command
  /// The RNTO command renames a file or directory on the server.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_rnto(std::string_view arguments) {
    logger_.info("Handling rnto: {}", arguments);
    if (rename_from_.empty()) {
      return send_response(503, "Bad sequence of commands.");
    }
    auto path_end = arguments.find("\r\n");
    if (path_end == std::string_view::npos) {
      logger_.error("Failed to parse path");
      return send_response(501, "Syntax error in parameters or arguments.");
    }
    std::string_view path = arguments.substr(0, path_end);
    std::filesystem::path full_path = current_directory_ / std::filesystem::path{path};
    if (std::filesystem::exists(full_path)) {
      return send_response(550, "File already exists.");
    }
    std::error_code ec;
    std::filesystem::rename(rename_from_.string(), full_path.string(), ec);
    if (ec) {
      logger_.error("Failed to rename file: {}", ec.message());
      return send_response(550, "Failed to rename file.");
    }
    rename_from_.clear();
    return send_response(250, "Rename successful.");
  }

  /// @brief Handle the NOOP command
  /// The NOOP command does nothing.
  /// @param arguments The arguments of the command
  /// @return True if the command was handled successfully, false otherwise
  bool handle_noop(std::string_view arguments) {
    logger_.info("Handling noop: {}", arguments);
    return send_response(200, "Command okay.");
  }

  /// @brief Handle unknown commands
  /// @param request The request to handle
  /// @return True if the command was handled successfully, false otherwise
  bool handle_unknown(std::string_view request) {
    logger_.error("Unknown reqeust: '{}'", request);
    return send_response(500, "Syntax error, command unrecognized.");
  }

private:
  int id_;

  std::string local_ip_address_;

  std::string username_;
  std::string password_;

  std::filesystem::path current_directory_;

  std::filesystem::path rename_from_;

  std::unique_ptr<TcpSocket> socket_;

  std::unique_ptr<TcpSocket> data_socket_;
  bool is_passive_data_connection_{false};

  // used when the data connection is passive
  TcpSocket passive_socket_;

  // used when the data connection is active
  std::string data_ip_address_;
  uint16_t data_port_{0};

  std::unique_ptr<Task> task_;
};

} // namespace espp
