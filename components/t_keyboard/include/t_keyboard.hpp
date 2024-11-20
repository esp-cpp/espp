#pragma once

#include <atomic>
#include <chrono>
#include <functional>

#include "base_peripheral.hpp"
#include "task.hpp"

namespace espp {
/// \brief Class for interacting with the LilyGo T-Keyboard.
/// \details This class is used to interact with the LilyGo T-Keyboard. It is
///          designed as a peripheral component for use with a serial
///          interface such as I2C. On The T-Keyboard, you can press Alt+B to
///          toggle the keyboard backlight.
///
/// \section Example
/// \snippet t_keyboard_example.cpp tkeyboard example
class TKeyboard : public BasePeripheral<> {
public:
  /// The default address of the keyboard.
  static constexpr uint8_t DEFAULT_ADDRESS = 0x55;

  /// \brief The function signature for the key callback function.
  /// \details This function is called when a key is pressed.
  /// \param key The key that was pressed.
  typedef std::function<void(uint8_t)> key_cb_fn;

  /// The configuration structure for the keyboard.
  struct Config {
    /// The write function to use.
    BasePeripheral::write_fn write;

    /// The read function to use.
    BasePeripheral::read_fn read;

    /// The key callback function to use. This function will be called when a
    /// key is pressed if it is not null and the keyboard task is running.
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
      : BasePeripheral({.address = config.address, .write = config.write, .read = config.read},
                       "TKeyboard", config.log_level)
      , key_cb_(config.key_cb)
      , polling_interval_(config.polling_interval) {
    logger_.info("TKeyboard created");
    using namespace std::placeholders;
    task_ = std::make_shared<espp::Task>(espp::Task::Config{
        .name = "tkeyboard_task",
        .callback = std::bind(&TKeyboard::key_task, this, _1, _2, _3),
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
  uint8_t get_key() const { return pressed_key_; }

  /// \brief Read a key from the keyboard.
  /// \details This function reads a key from the keyboard. It will return 0 and
  ///          set the error code if an error occurs. If the keyboard task is
  ///          running, it will return 0 and set the error code.
  ///          If the keyboard task is not running, it will return the key that
  ///          was read, and update the currently pressed key.
  /// \param ec The error code to set if an error occurs.
  /// \return The key that was read.
  /// \see get_key()
  /// \note This function will return 0 if no key was read.
  /// \note This function will return 0 if an error occurs.
  /// \note This function will set the error code if the keyboard task is
  ///       running.
  uint8_t read_key(std::error_code &ec) {
    if (task_->is_running()) {
      logger_.error("Keyboard task is running, use get_key() instead");
      ec = std::make_error_code(std::errc::operation_in_progress);
      return 0;
    }
    auto key = read_u8(ec);
    if (ec) {
      logger_.error("Failed to get key: {}", ec.message());
      return 0;
    }
    pressed_key_ = key;
    return key;
  }

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
  bool key_task(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    std::error_code ec;
    auto start_time = std::chrono::steady_clock::now();
    auto key = read_u8(ec);
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
      cv.wait_until(lock, start_time + polling_interval_,
                    [&task_notified] { return task_notified; });
      task_notified = false;
    }
    return false;
  }

  key_cb_fn key_cb_;
  std::atomic<uint8_t> pressed_key_{0};
  std::chrono::milliseconds polling_interval_;
  std::shared_ptr<espp::Task> task_;
};
} // namespace espp
