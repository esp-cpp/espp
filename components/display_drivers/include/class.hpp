
/**
 * @brief Base class for display drivers.
 */
class DisplayDriver {
public:
  /// @brief Construct a DisplayDriver
  /// @param config Configuration for the display driver
  explicit DisplayDriver(const Config &config)
      : config_(config) {}

  /// @brief Initialize the display driver
  /// @param write Write function for sending commands to the display
  /// @param send_lines Function for sending pixel data to the display
  /// @param width Width of the display in pixels
  /// @param height Height of the display in pixels
  /// @return True if initialization was successful, false otherwise
  virtual bool initialize(write_command_fn write, send_lines_fn send_lines, size_t width,
                          size_t height) {
    write_command_ = write;
    send_lines_ = send_lines;
    width_ = width;
    height_ = height;
    return true;
  }

  /// @brief Reset the display
  virtual void reset() {
    std::scoped_lock lk(config_mutex_);
    if (config_.reset_pin != GPIO_NUM_NC) {
      gpio_set_level(config_.reset_pin, config_.reset_value);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      gpio_set_level(config_.reset_pin, !config_.reset_value);
      std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
  }

  /// @brief Get the width of the display
  /// @return Width in pixels
  size_t width() const { return width_; }

  /// @brief Get the height of the display
  /// @return Height in pixels
  size_t height() const { return height_; }

protected:
  std::mutex config_mutex_;
  Config config_;
  size_t width_{0};
  size_t height_{0};
};
