#pragma once

#include <atomic>
#include <chrono>
#include <functional>

#include "logger.hpp"
#include "task.hpp"

namespace espp {
/// \brief Class for interacting with the LilyGo T-Keyboard.
/// \details This class is used to interact with the LilyGo T-Keyboard. It is
///          designed as a peripheral component for use with a serial
///          interface such as I2C.
///
/// \section Example
/// \snippet t_keyboard_example.cpp tkeyboard example
class TKeyboard {
public:
  /// The default address of the keyboard.
  static constexpr uint8_t DEFAULT_ADDRESS = 0x55;

  /// \brief The function signature for the write function.
  /// \details This function is used to write data to the keyboard.
  /// \param address The address to write to.
  /// \param data The data to write.
  /// \param size The size of the data to write.
  /// \return True if the write was successful, false otherwise.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> write_fn;

  /// \brief The function signature for the read function.
  /// \details This function is used to read data from the keyboard.
  /// \param address The address to read from.
  /// \param data The data to read into.
  /// \param size The size of the data to read.
  /// \return True if the read was successful, false otherwise.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> read_fn;

  /// \brief The function signature for the key callback function.
  /// \details This function is called when a key is pressed.
  /// \param key The key that was pressed.
  typedef std::function<void(uint8_t)> key_cb_fn;

  /// The configuration structure for the keyboard.
  struct Config {
    /// The write function to use.
    write_fn write;

    /// The read function to use.
    read_fn read;

    /// The key callback function to use.
    key_cb_fn key_cb;

    /// The address of the keyboard.
    uint8_t address = DEFAULT_ADDRESS;

    /// The polling interval for the keyboard
    std::chrono::milliseconds polling_interval = std::chrono::milliseconds(10);

    /// Whether or not to automatically start the keyboard task.
    bool auto_start = true;

    /// The log level to use.
    Logger::Verbosity log_level = Logger::Verbosity::WARN;
  };

  /// \brief Constructor for the TKeyboard class.
  /// \param config The configuration to use.
  explicit TKeyboard(const Config &config)
      : write_(config.write), read_(config.read), key_cb_(config.key_cb), address_(config.address),
        polling_interval_(config.polling_interval),
        logger_({.tag = "TKeyboard", .level = config.log_level}) {
    logger_.info("TKeyboard created");
    task_ = std::make_shared<espp::Task>(espp::Task::Config{
        .name = "tkeyboard_task",
        .callback =
            std::bind(&TKeyboard::key_task, this, std::placeholders::_1, std::placeholders::_2),
        .stack_size_bytes = 4 * 1024,
    });
    if (config.auto_start) {
      start();
    }
  }

  /// \brief Get the currently pressed key.
  /// \details This function returns the currently pressed key.
  /// \return The currently pressed key.
  /// \note This function will return 0 if no key has been pressed.
  uint8_t get_key() { return pressed_key_; }

  /// \brief Start the keyboard task.
  /// \details This function starts the keyboard task. It should be called
  ///         after the keyboard has been initialized.
  /// \return True if the task was started, false otherwise.
  bool start() {
    logger_.info("Starting keyboard task");
    return task_->start();
  }

  /// \brief Stop the keyboard task.
  /// \details This function stops the keyboard task.
  /// \return True if the task was stopped, false otherwise.
  bool stop() { return task_->stop(); }

protected:
  bool key_task(std::mutex &m, std::condition_variable &cv) {
    std::error_code ec;
    auto start = std::chrono::steady_clock::now();
    auto key = read_char(ec);
    if (!ec) {
      pressed_key_ = key;
      if (key != 0 && key_cb_) {
        key_cb_(key);
      }
    } else {
      logger_.error("Failed to get key: {}", ec.message());
      pressed_key_ = 0;
    }
    {
      std::unique_lock<std::mutex> lock(m);
      cv.wait_until(lock, start + polling_interval_);
    }
    return false;
  }

  uint8_t read_char(std::error_code &ec) {
    uint8_t data = 0;
    read(&data, 1, ec);
    if (ec) {
      logger_.error("Failed to read char: {}", ec.message());
      return 0;
    }
    return data;
  }

  void write(uint8_t *data, size_t size, std::error_code &ec) {
    if (!write_(address_, data, size)) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  void read(uint8_t *data, size_t size, std::error_code &ec) {
    if (!read_(address_, data, size)) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  write_fn write_;
  read_fn read_;
  key_cb_fn key_cb_;
  uint8_t address_;
  std::atomic<uint8_t> pressed_key_{0};
  std::chrono::milliseconds polling_interval_;
  std::shared_ptr<espp::Task> task_;
  espp::Logger logger_;
};
} // namespace espp
