#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"

namespace espp {
/// \brief A class which implements a simple FTP client.
class FtpClient : public BaseComponent {
public:
  /// \brief A struct which represents an FTP response.
  struct Response {
    int code;            ///< The response code.
    std::string message; ///< The response message.
  };

  /// \brief A struct which represents the FTP server information.
  struct ServerInfo {
    std::string host;     ///< The hostname or IP address of the FTP server.
    std::string port{21}; ///< The port number of the FTP server.
    std::string path;     ///< The current working directory on the FTP server.
    std::string filename; ///< The filename of the file to download.
  };

  /// \brief A struct which represents a file on the FTP server.
  struct FileInfo {
    std::string name;     ///< The name of the file.
    std::string type;     ///< The type of the file.
    std::string size;     ///< The size of the file.
    std::string modified; ///< The last modified time of the file.
  };

  /// \brief Constructs a new FtpClient object.
  FtpClient()
      : BaseComponent("FtpClient") {}

  /// \brief Connects to the FTP server.
  /// \param host The hostname or IP address of the FTP server.
  /// \param port The port number of the FTP server.
  /// \param user The username to use for authentication.
  /// \param password The password to use for authentication.
  /// \return True if the connection was successful, false otherwise.
  bool connect(const std::string_view host, const std::string_view port,
               const std::string_view user, const std::string_view password);

  /// \brief Disconnects from the FTP server.
  /// \return True if the disconnection was successful, false otherwise.
  bool disconnect();

  /// \brief Changes the current working directory on the FTP server.
  /// \param path The path to the directory to change to.
  /// \return True if the directory was changed successfully, false otherwise.
  bool change_directory(const std::string_view path);

  /// \brief Changes the current working directory on the FTP server to the parent directory.
  /// \return True if the directory was changed successfully, false otherwise.
  bool change_directory_to_parent();

  /// \brief Creates a directory on the FTP server.
  /// \param path The path to the directory to create.
  /// \return True if the directory was created successfully, false otherwise.
  bool create_directory(const std::string_view path);

  /// \brief Deletes a directory on the FTP server.
  /// \param path The path to the directory to delete.
  /// \return True if the directory was deleted successfully, false otherwise.
  bool delete_directory(const std::string_view path);

  /// \brief Deletes a file on the FTP server.
  /// \param path The path to the file to delete.
  /// \return True if the file was deleted successfully, false otherwise.
  bool delete_file(const std::string_view path);

  /// \brief Renames a file on the FTP server.
  /// \param from The path to the file to rename.
  /// \param to The new name of the file.
  /// \return True if the file was renamed successfully, false otherwise.
  bool rename_file(const std::string_view from, const std::string_view to);

  /// \brief Gets the size of a file on the FTP server.
  /// \param path The path to the file to get the size of.
  /// \return The size of the file.
  std::string get_file_size(const std::string_view path);

  /// \brief Gets the last modified time of a file on the FTP server.
  /// \param path The path to the file to get the last modified time of.
  /// \return The last modified time of the file.
  std::string get_file_last_modified_time(const std::string_view path);

  /// \brief Gets the current working directory on the FTP server.
  /// \return The current working directory.
  std::string get_current_directory();

  /// \brief Gets the contents of a directory on the FTP server.
  /// \param path The path to the directory to get the contents of.
  /// \return A vector of FileInfo objects representing the contents of the directory.
  std::vector<FileInfo> get_directory_contents(const std::string_view path);

  /// \brief Gets the contents of a directory on the FTP server.
  /// \param path The path to the directory to get the contents of.
  /// \return A vector of strings representing the contents of the directory.
  std::vector<std::string> get_directory_listing(const std::string_view path);

  /// \brief Downloads a file from the FTP server.
  /// \param from The path to the file to download.
  /// \param to The path to the file to download to.
  /// \return True if the file was downloaded successfully, false otherwise.
  bool download_file(const std::string_view from, const std::string_view to);

  /// \brief Downloads a file from the FTP server.
  /// \param from The path to the file to download.
  /// \param data Vector to store the file data in.
  /// \return True if the file was downloaded successfully, false otherwise.
  bool download_file(const std::string_view from, std::vector<uint8_t> &data);

  /// \brief Uploads a file to the FTP server.
  /// \param from The path to the file to upload.
  /// \param to The path to the file to upload to.
  /// \return True if the file was uploaded successfully, false otherwise.
  bool upload_file(const std::string_view from, const std::string_view to);

  /// \brief Uploads a file to the FTP server.
  /// \param data Vector containing the file data.
  /// \param to The path on the FTP server to upload the file to.
  /// \return True if the file was uploaded successfully, false otherwise.
  bool upload_file(const std::vector<uint8_t> &data, const std::string_view to);

protected:
  /// \brief Sends a command to the FTP server.
  /// \param command The command to send.
  /// \param response The response from the FTP server.
  /// \return True if the command was sent successfully, false otherwise.
  bool send_command(const std::string_view command, Response &response) {
    return send_command(command, "", response);
  }

  /// \brief Sends a command to the FTP server.
  /// \param command The command to send.
  /// \param argument The argument to the command.
  /// \param response The response from the FTP server.
  /// \return True if the command was sent successfully, false otherwise.
  bool send_command(const std::string_view command, const std::string_view argument,
                    Response &response) {
    return send_command(command, std::vector<std::string_view>{argument}, response);
  }

  /// \brief Sends a command to the FTP server.
  /// \param command The command to send.
  /// \param arguments The arguments to the command.
  /// \param response The response from the FTP server.
  /// \return True if the command was sent successfully, false otherwise.
  bool send_command(const std::string_view command, const std::vector<std::string_view> &arguments,
                    Response &response) {
    std::string command_string = command.data();
    for (const auto &argument : arguments) {
      command_string += " " + std::string(argument);
    }
    command_string += "\r\n";
    logger_.debug("Sending command:\n{}", command_string);
    std::string response_string;
    TcpSocket::TransmitConfig config{
        .wait_for_response = true,
        .response_size = 1024, // in bytes
        .on_response_callback =
            [&response_string](const std::string_view data) { response_string += data; },
        .response_timeout = std::chrono::duration<float>(1.0f), // in seconds
    };
    if (!control_socket_.transmit(command_string, config)) {
      logger_.error("Failed to send command");
      return false;
    }
    logger_.debug("Received response:\n{}", response_string);
    // parse the response string
    if (!parse_response(response_string, response)) {
      logger_.error("Failed to parse response");
      return false;
    }
    return true;
  }

  /// \brief Parse a line from the response to get the response code.
  /// \param response_line The line from the response to parse.
  /// \param code The response code.
  /// \return True if the response code was parsed successfully, false otherwise.
  bool get_response_code(std::string_view response_line, int &code) {
    // the response line is in the format:
    // <code> <message>\r\n
    // where <code> is a 3 digit integer and <message> is a string
    // the response line may also contain multiple lines, in which case the code is followed by a
    // hyphen
    if (response_line.size() < 4) {
      logger_.error("Invalid response line: {}", response_line);
      return false;
    }
    // get the code
    std::string code_string = std::string(response_line.substr(0, 3));
    // parse the code string without using stoi since it throws exceptions
    // and we don't want to use exceptions in this library
    code = 0;
    for (const auto &c : code_string) {
      if (c < '0' || c > '9') {
        logger_.error("Invalid response code: {}", code_string);
        return false;
      }
      code *= 10;
      code += c - '0';
    }
    return true;
  }

  /// \brief Parse the response string.
  /// \param response The response string to parse.
  /// \param parsed_response The parsed response.
  /// \return True if the response was parsed successfully, false otherwise.
  bool parse_response(const std::string_view response, Response &parsed_response) {
    // parse the response string
    // the response string is in the format:
    // <code> <message>\r\n
    // where <code> is a 3 digit integer and <message> is a string
    // the response string may also contain multiple lines, in which case the code is followed by a
    // hyphen and the message ends when a line starting with the same code followed by a space is
    // encountered
    int code = 0;
    std::string message;
    std::string line;
    std::istringstream stream(response.data());
    bool multiline = false;
    // get the first (possibly only) line of the response
    if (!std::getline(stream, line)) {
      logger_.error("Invalid response: {}", response);
      return false;
    }
    if (line.size() < 4) {
      logger_.error("Invalid response line: {}", line);
      return false;
    }
    // determine if the message in the response is multiline
    if (line[3] == '-') {
      multiline = true;
    }
    // get the code from the response
    if (!get_response_code(line, code)) {
      logger_.error("Failed to get response code");
      return false;
    }
    // if the message is multiline, keep reading lines until we find the end of the message
    // otherwise, the message is the rest of the line
    if (multiline) {
      while (std::getline(stream, line)) {
        // check if the line contains a code or not
        if (line.size() >= 4 && line[3] == ' ') {
          // get the code from the response
          int final_code = 0;
          if (get_response_code(line, final_code)) {
            // if the code matches the original code, we've reached the end of the message
            if (final_code == code) {
              break;
            }
          }
        }
        // add the line to the message
        message += "\n" + line;
      }
    } else {
      message = line.substr(4);
    }
    // set the parsed response
    parsed_response.code = code;
    parsed_response.message = message;
    return true;
  }

  TcpSocket control_socket_;
  TcpSocket data_socket_;
  std::string host_;
  std::string user_;
  std::string password_;
  std::string port_;
  std::string path_;
  std::string filename_;
  std::string type_;
  std::string mode_;
  std::string structure_;
  std::string command_;
  std::string response_;
  std::string message_;
  std::unique_ptr<Task> task_;
};
} // namespace espp
