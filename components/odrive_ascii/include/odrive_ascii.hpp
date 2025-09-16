#pragma once

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base_component.hpp"

namespace espp {

/**
 * @brief Minimal ODrive-compatible ASCII protocol server.
 *
 * Parses ODrive-style ASCII commands from an input byte stream and generates
 * ASCII responses for the caller to transmit. This component does not perform
 * any I/O itself; applications must feed incoming bytes via process_bytes()
 * and send the returned response bytes over their transport (UART/USB/etc.).
 *
 * Supported commands (subset of ODrive ASCII):
 *  - r <path>
 *  - w <path> <value>
 *  - p <axis> <pos> [vel_ff [torque_ff]]
 *  - v <axis> <vel> [torque_ff]
 *  - c <axis> <torque_nm>
 *  - t <axis> <goal_pos_turns>
 *  - f <axis>
 *  - es <axis> <abs_pos_turns>
 *  - help
 *
 * Applications integrate by registering properties and command callbacks via
 * std::function dependency injection. The component is thread-safe.
 *
 * For more information about this protocol see the odrive documentation:
 * https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html
 *
 * \section odrive_ascii_ex1 Basic Example
 * \snippet odrive_ascii_example.cpp odrive_ascii_basic_example
 * \section odrive_ascii_ex2 Console Example
 * \snippet odrive_ascii_example.cpp odrive_ascii_console_example
 */
class OdriveAscii : public BaseComponent {
public:
  /**
   * @brief Configuration for the OdriveAscii server.
   */
  struct Config {
    size_t max_line_length{256}; /**< Maximum accepted ASCII line length. */
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Create an OdriveAscii protocol server.
   * @param config Configuration parameters.
   */
  explicit OdriveAscii(const Config &config)
      : BaseComponent("ODriveASCII", config.log_level)
      , config_(config) {}

  /**
   * @brief Read-only property accessor. Returns textual value, sets ec on error.
   */
  using read_fn = std::function<std::string(std::error_code &ec)>;

  /**
   * @brief Write property accessor. Receives textual value; return true on success; set ec on
   * error.
   */
  using write_fn = std::function<bool(std::string_view, std::error_code &ec)>;

  /**
   * @brief Register a property accessible via r/w path.
   * @param path Canonical ODrive-style path (e.g., "axis0.encoder.pos_estimate").
   * @param read Optional read function; if omitted, property is write-only.
   * @param write Optional write function; if omitted, property is read-only.
   */
  void register_property(const std::string &path, const read_fn &read = nullptr,
                         const write_fn &write = nullptr) {
    std::scoped_lock lk(prop_mutex_);
    properties_[path] = Property{read, write};
  }

  /**
   * @brief Helper to register a numeric property using typed getter/setter.
   *        Converts to/from string using std::to_string / std::strtod.
   * @param path Property path.
   * @param getter Getter function; if null, property is write-only.
   * @param setter Setter function; if null, property is read-only.
   * @note The float is formatted using "{:0.6g}".
   */
  void
  register_float_property(const std::string &path, const std::function<float()> &getter,
                          const std::function<bool(float, std::error_code &)> &setter = nullptr) {
    read_fn rf = nullptr;
    write_fn wf = nullptr;
    if (getter) {
      rf = [getter](std::error_code &ec) {
        auto v = getter();
        ec.clear();
        return fmt::format("{:0.6g}", static_cast<double>(v));
      };
    }
    if (setter) {
      wf = [setter](std::string_view sv, std::error_code &ec) {
        // tolerate hex/decimal/float per std::strtod
        std::string tmp(sv);
        char *end = nullptr;
        float val = static_cast<float>(strtod(tmp.c_str(), &end));
        if (end == tmp.c_str()) {
          ec = std::make_error_code(std::errc::invalid_argument);
          return false; // no conversion
        }
        return setter(val, ec);
      };
    }
    register_property(path, rf, wf);
  }

  /**
   * @brief Helper to register an integer property using typed getter/setter.
   * @param path Property path.
   * @param getter Getter function; if null, property is write-only.
   * @param setter Setter function; if null, property is read-only.
   * @note The integer is formatted using std::to_string.
   */
  void
  register_int_property(const std::string &path, const std::function<int32_t()> &getter,
                        const std::function<bool(int32_t, std::error_code &)> &setter = nullptr) {
    read_fn rf = nullptr;
    write_fn wf = nullptr;
    if (getter) {
      rf = [getter](std::error_code &ec) {
        ec.clear();
        return std::to_string(getter());
      };
    }
    if (setter) {
      wf = [setter](std::string_view sv, std::error_code &ec) {
        std::string tmp(sv);
        char *end = nullptr;
        long long val = strtoll(tmp.c_str(), &end, 0);
        if (end == tmp.c_str()) {
          ec = std::make_error_code(std::errc::invalid_argument);
          return false; // no conversion
        }
        return setter(static_cast<int32_t>(val), ec);
      };
    }
    register_property(path, rf, wf);
  }

  /**
   * @brief Helper to register a boolean property using typed getter/setter.
   * Accepts "0/1", "true/false" (case-insensitive) when writing.
   * @param path Property path.
   * @param getter Getter function; if null, property is write-only.
   * @param setter Setter function; if null, property is read-only.
   * @note The boolean is formatted as "0" or "1".
   */
  void
  register_bool_property(const std::string &path, const std::function<bool()> &getter,
                         const std::function<bool(bool, std::error_code &)> &setter = nullptr) {
    read_fn rf = nullptr;
    write_fn wf = nullptr;
    if (getter) {
      rf = [getter](std::error_code &ec) {
        ec.clear();
        return getter() ? std::string("1") : std::string("0");
      };
    }
    if (setter) {
      wf = [setter](std::string_view sv, std::error_code &ec) {
        if (sv == "1" || sv == "true" || sv == "TRUE")
          return setter(true, ec);
        if (sv == "0" || sv == "false" || sv == "FALSE")
          return setter(false, ec);
        ec = std::make_error_code(std::errc::invalid_argument);
        return false;
      };
    }
    register_property(path, rf, wf);
  }

  /**
   * @brief Callback for high-rate position command 'p'.
   * Signature: (axis, pos, vel_ff, torque_ff) -> bool
   */
  using position_command_fn =
      std::function<bool(int axis, float pos, const std::optional<float> &vel_ff,
                         const std::optional<float> &torque_ff, std::error_code &ec)>;

  /**
   * @brief Callback for high-rate velocity command 'v'.
   * Signature: (axis, vel, torque_ff) -> bool
   */
  using velocity_command_fn = std::function<bool(
      int axis, float vel, const std::optional<float> &torque_ff, std::error_code &ec)>;

  /**
   * @brief Callback for high-rate torque/current command 'c'.
   * Signature: (axis, torque) -> bool
   */
  using torque_command_fn = std::function<bool(int axis, float torque_nm, std::error_code &ec)>;

  /**
   * @brief Callback for trajectory command 't'.
   * Signature: (axis, goal_pos_turns) -> bool
   */
  using trajectory_command_fn =
      std::function<bool(int axis, float goal_pos_turns, std::error_code &ec)>;

  /**
   * @brief Callback for feedback request 'f'.
   * Signature: (axis, out_pos_turns, out_vel_turns_per_s) -> bool
   */
  using feedback_command_fn =
      std::function<bool(int axis, float &pos_out, float &vel_out, std::error_code &ec)>;

  /**
   * @brief Callback for encoder set absolute position 'es'.
   * Signature: (axis, abs_pos_turns) -> bool
   */
  using encoder_set_abs_position_fn =
      std::function<bool(int axis, float abs_pos_turns, std::error_code &ec)>;

  /**
   * @brief Set the position command callback.
   * @param fn Callback function.
   * @note This is a high-rate command; the callback should be efficient and avoid blocking.
   */
  void on_position_command(const position_command_fn &fn) {
    std::scoped_lock lk(cb_mutex_);
    pos_cb_ = fn;
  }

  /**
   * @brief Set the velocity command callback.
   * @param fn Callback function.
   * @note This is a high-rate command; the callback should be efficient and avoid blocking.
   */
  void on_velocity_command(const velocity_command_fn &fn) {
    std::scoped_lock lk(cb_mutex_);
    vel_cb_ = fn;
  }

  /**
   * @brief Set the torque/current command callback.
   * @param fn Callback function.
   * @note This is a high-rate command; the callback should be efficient and avoid blocking.
   */
  void on_torque_command(const torque_command_fn &fn) {
    std::scoped_lock lk(cb_mutex_);
    torque_cb_ = fn;
  }

  /**
   * @brief Set the trajectory command callback.
   * @param fn Callback function.
   * @note This is a high-rate command; the callback should be efficient and avoid blocking.
   */
  void on_trajectory_command(const trajectory_command_fn &fn) {
    std::scoped_lock lk(cb_mutex_);
    traj_cb_ = fn;
  }

  /**
   * @brief Set the feedback request callback.
   * @param fn Callback function.
   * @note This is a high-rate command; the callback should be efficient and avoid blocking.
   */
  void on_feedback_request(const feedback_command_fn &fn) {
    std::scoped_lock lk(cb_mutex_);
    fb_cb_ = fn;
  }

  /**
   * @brief Set the encoder set absolute position callback.
   * @param fn Callback function.
   * @note This is a high-rate command; the callback should be efficient and avoid blocking.
   */
  void on_encoder_set_absolute(const encoder_set_abs_position_fn &fn) {
    std::scoped_lock lk(cb_mutex_);
    enc_set_abs_position_cb_ = fn;
  }

  /**
   * @brief Process a chunk of input bytes; returns response bytes to transmit (if any).
   * @param data Incoming bytes (may contain partial or multiple lines) using CR/LF line endings.
   * @return Response bytes (possibly empty). May contain multiple response lines.
   */
  std::vector<uint8_t> process_bytes(std::span<const uint8_t> data);

  /**
   * @brief Clear any buffered input state.
   */
  void clear_buffer();

private:
  struct Property {
    read_fn read;
    write_fn write;
  };

  std::optional<std::string> handle_line(std::string_view line);
  std::optional<std::string> handle_read(std::string_view path);
  std::optional<std::string> handle_write(std::string_view path, std::string_view value);
  std::optional<std::string> handle_position_cmd(std::span<std::string_view> tokens);
  std::optional<std::string> handle_velocity_cmd(std::span<std::string_view> tokens);
  std::optional<std::string> handle_torque_cmd(std::span<std::string_view> tokens);          // 'c'
  std::optional<std::string> handle_trajectory_cmd(std::span<std::string_view> tokens);      // 't'
  std::optional<std::string> handle_feedback_cmd(std::span<std::string_view> tokens);        // 'f'
  std::optional<std::string> handle_encoder_set_abs_cmd(std::span<std::string_view> tokens); // 'es'

  static std::string trim(std::string_view sv);
  static std::vector<std::string_view> split_ws(std::string_view sv, size_t max_parts = SIZE_MAX);

  Config config_;

  std::mutex buf_mutex_;
  std::string inbuf_;

  std::mutex prop_mutex_;
  std::unordered_map<std::string, Property> properties_;

  std::mutex cb_mutex_;
  position_command_fn pos_cb_;
  velocity_command_fn vel_cb_;
  torque_command_fn torque_cb_;
  trajectory_command_fn traj_cb_;
  feedback_command_fn fb_cb_;
  encoder_set_abs_position_fn enc_set_abs_position_cb_;
};

} // namespace espp
