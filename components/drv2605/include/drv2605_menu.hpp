#pragma once

#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "drv2605.hpp"
#include "format.hpp"

namespace espp {
/// @brief A menu for controlling the DRV2605 haptics driver.
/// This menu provides commands to calibrate the LRA, set modes, play waveforms,
/// set waveforms, and control the haptics driver in various ways.
/// It is designed to be used with the ESP-CPP / ESPP C++ CLI framework.
/// It supports multiple drivers, allowing you to control
/// multiple DRV2605 devices simultaneously.
///
/// @section drv2605_menu_ex1 Example
/// @snippet drv2605_example.cpp drv2605 menu example
class Drv2605Menu {
public:
  /// @brief Type alias for the DRV2605 driver.
  using Driver = espp::Drv2605;

  /// @brief Construct a new Drv2605Menu object.
  /// @param i2c A vector of shared pointers to the Drv2605s to interact with.
  explicit Drv2605Menu(const std::vector<std::shared_ptr<Driver>> &drivers)
      : drivers_(drivers) {}

  /// @brief Get the Drv2605 menu.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the Drv2605 menu that you can use to add to a
  ///         CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "drv2605",
                                 std::string_view description = "Drv2605 menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // calibrate the LRA
    menu->Insert(
        "calibrate", {"LRA frequency (Hz)"},
        [this](std::ostream &out, float lra_freq) -> void {
          if (lra_freq <= 0) {
            out << "Invalid frequency. Must be greater than 0.\n";
            return;
          }
          calibrate(out, lra_freq);
        },
        "Calibrate the LRA with the specified frequency in Hz.");

    // set the mode
    menu->Insert(
        "set_mode",
        {"mode (0=INTTRIG, 1=EXTTRIGEDGE, 2=EXTTRIGLVL, 3=PWMANALOG, 4=AUDIOVIBE, 5=REALTIMEPWM)"},
        [this](std::ostream &out, int mode) -> void {
          if (mode < 0 || mode > static_cast<int>(Driver::Mode::REALTIME)) {
            out << "Invalid mode. Must be one of:\n"
                << "0: INTTRIG\n"
                << "1: EXTTRIGEDGE\n"
                << "2: EXTTRIGLVL\n"
                << "3: PWMANALOG\n"
                << "4: AUDIOVIBE\n"
                << "5: REALTIMEPWM\n";
            return;
          }
          set_mode(out, static_cast<Driver::Mode>(mode));
        },
        "Set the mode of the DRV2605(s). Valid modes are:\n"
        "\t0: INTTRIG - used for playing built-in haptics using the 'play' command.\n"
        "\t1: EXTTRIGEDGE - used for playing built-in haptics using in IN pin edge as the "
        "trigger.\n"
        "\t2: EXTTRIGLVL - used for playing built-in haptics using in IN pin level as the "
        "trigger.\n"
        "\t3: PWMANALOG - direct PWM control of the LRA using the IN pin as a PWM input.\n"
        "\t4: AUDIOVIBE - use the IN pin as analog audio input to drive the LRA.\n"
        "\t5: REALTIMEPWM - use I2C to provide PWM / amplitude control in real-time.\n");

    // play a waveform by ID
    menu->Insert(
        "play", {"waveform ID (1-123)"},
        [this](std::ostream &out, uint8_t waveform_id) -> void {
          if (waveform_id < 1 || waveform_id > 123) {
            out << "Invalid waveform ID. Must be between 1 and 123.\n";
            return;
          }
          play_waveform(out, waveform_id);
        },
        "Play a waveform by its ID (1-123). Requires mode = 0 (INTTRIG).");

    // set realtime PWM
    menu->Insert(
        "pwm", {"amplitude (0-255)"},
        [this](std::ostream &out, uint8_t amplitude) -> void { set_realtime_pwm(out, amplitude); },
        "Set the realtime PWM amplitude. Valid range is 0-255. To stop the PWM, set amplitude to "
        "0. Requires mode = 5 (REALTIMEPWM).");

    // set waveforms to play
    menu->Insert(
        "set_waveforms", {"waveform ID (1-123)", "..."},
        [this](std::ostream &out, const std::vector<std::string> &waveforms) -> void {
          if (waveforms.empty()) {
            out << "No waveforms provided.\n";
            return;
          }
          // ensure the size is <= 7
          if (waveforms.size() > 7) {
            out << "Too many waveforms provided. Maximum number of waveforms is 7.\n";
            return;
          }
          // convert the string IDs to integers
          std::vector<uint8_t> waveform_ids;
          for (const auto &id_str : waveforms) {
            try {
              int id = std::stoi(id_str);
              if (id < 1 || id > 123) {
                throw std::out_of_range("ID out of range");
              }
              waveform_ids.push_back(id);
            } catch (const std::invalid_argument &) {
              out << "Invalid waveform ID: " << id_str
                  << ". Must be an integer between 1 and 123.\n";
              return;
            } catch (const std::out_of_range &) {
              out << "Waveform ID out of range: " << id_str << ". Must be between 1 and 123.\n";
              return;
            }
          }
          set_waveforms(out, waveform_ids);
        },
        "Set the waveforms to play. Provide a list of waveform IDs (1-123). To play/stop the "
        "waveforms, use the 'start' or 'stop' commands. ");

    // start the driver
    menu->Insert(
        "start", [this](std::ostream &out) -> void { start(out); },
        "start the driver(s). This will start playing the currently set waveforms. Requires "
        "mode = 0 (INTTRIG).");
    // stop the driver
    menu->Insert(
        "stop", [this](std::ostream &out) -> void { stop(out); },
        "Stop the driver(s). This will stop any currently playing waveforms. Requires mode = "
        "0 (INTTRIG).");

    return menu;
  }

  /// @brief Calibrate the LRA with the specified frequency.
  /// @param out The output stream to write messages to.
  /// @param lra_freq The frequency to calibrate the LRA at, in Hz.
  /// @note This will set the rated voltage to 255 and the overdrive clamp to 255.
  void calibrate(std::ostream &out, float lra_freq) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      Driver::LraCalibrationSettings lra_calibration_settings{};
      lra_calibration_settings.rated_voltage = 255;
      lra_calibration_settings.overdrive_clamp = 255;
      lra_calibration_settings.drive_time = Driver::lra_freq_to_drive_time(lra_freq);
      Driver::CalibratedData calibrated_data;
      if (!driver->calibrate(lra_calibration_settings, calibrated_data, ec)) {
        out << "Error calibrating LRA: " << ec.message() << "\n";
      } else {
        out << "LRA calibrated successfully.\n";
      }
    }
  }

  /// @brief Play a waveform by its ID.
  /// @param out The output stream to write messages to.
  /// @param waveform_id The ID of the waveform to play (1-123).
  /// @note This will stop any currently playing waveforms before starting the new one.
  /// @note Requires mode = 0 (INTTRIG).
  void play_waveform(std::ostream &out, uint8_t waveform_id) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      // stop any playing waveforms
      driver->stop(ec);
      // set the waveform to play
      driver->set_waveform(0, static_cast<Driver::Waveform>(waveform_id), ec);
      driver->set_waveform(1, Driver::Waveform::END, ec);
      // play the waveform on each driver
      driver->start(ec);
      if (ec) {
        out << "Error playing waveform " << (int)waveform_id << ": " << ec.message() << "\n";
      } else {
        out << "Started waveform " << (int)waveform_id << " successfully.\n";
      }
    }
  }

  /// @brief Set the waveforms to play.
  /// @param out The output stream to write messages to.
  /// @param waveforms A vector of waveform IDs (1-123) to set.
  /// @note This will stop any currently playing waveforms before setting the new ones.
  /// @note The maximum number of waveforms that can be set is 7.
  /// @note Requires mode = 0 (INTTRIG).
  void set_waveforms(std::ostream &out, const std::vector<uint8_t> &waveforms) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      // stop any playing waveforms
      driver->stop(ec);
      // set the waveforms to play
      for (size_t i = 0; i < waveforms.size(); ++i) {
        driver->set_waveform(i, static_cast<Driver::Waveform>(waveforms[i]), ec);
      }
      // end the waveform list
      driver->set_waveform(waveforms.size(), Driver::Waveform::END, ec);
      if (ec) {
        out << "Error setting waveforms: " << ec.message() << "\n";
      } else {
        out << "Set waveforms successfully: ";
        for (const auto &id : waveforms) {
          out << (int)id << " ";
        }
        out << "\n";
      }
    }
  }

  /// @brief Start the haptics driver.
  /// @param out The output stream to write messages to.
  /// @note This will start playing the currently set waveform(s).
  /// @note Requires mode = 0 (INTTRIG).
  void start(std::ostream &out) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      driver->start(ec);
      if (ec) {
        out << "Error starting driver: " << ec.message() << "\n";
      } else {
        out << "Started driver successfully.\n";
      }
    }
  }

  /// @brief Stop the haptics driver.
  /// @param out The output stream to write messages to.
  /// @note This will stop any currently playing waveforms.
  /// @note Requires mode = 0 (INTTRIG).
  void stop(std::ostream &out) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      driver->stop(ec);
      if (ec) {
        out << "Error stopping driver: " << ec.message() << "\n";
      } else {
        out << "Stopped driver successfully.\n";
      }
    }
  }

  /// @brief Set the mode of the haptics driver.
  /// @param out The output stream to write messages to.
  /// @param mode The mode to set the driver to.
  /// @note Valid modes are:
  /// 0: INTTRIG - used for playing built-in haptics using the 'play' command.
  /// 1: EXTTRIGEDGE - used for playing built-in haptics using in IN pin edge as the trigger.
  /// 2: EXTTRIGLVL - used for playing built-in haptics using in IN pin level as the trigger.
  /// 3: PWMANALOG - direct PWM control of the LRA using the IN pin as a PWM input.
  /// 4: AUDIOVIBE - use the IN pin as analog audio input to drive the LRA.
  /// 5: REALTIMEPWM - use I2C to provide PWM / amplitude control in real-time.
  /// @note This will set the mode for all drivers in the menu.
  void set_mode(std::ostream &out, Driver::Mode mode) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      driver->set_mode(mode, ec);
      if (ec) {
        out << "Error setting mode: " << ec.message() << "\n";
      } else {
        out << "Set mode successfully.\n";
      }
    }
  }

  /// @brief Set the RTP data format to unsigned.
  /// @param out The output stream to write messages to.
  /// @param use_unsigned Whether to use unsigned format for RTP data.
  /// @note This will set the RTP data format for all drivers in the menu.
  /// @note This is only applicable when the mode is set to REALTIMEPWM.
  void set_rtp_pwm_unsigned(std::ostream &out, bool use_unsigned) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      driver->set_rtp_data_format_unsigned(use_unsigned, ec);
      if (ec) {
        out << "Error setting RTP data format unsigned: " << ec.message() << "\n";
      } else {
        out << "Set RTP data format to unsigned successfully.\n";
      }
    }
  }

  /// @brief Set the realtime PWM amplitude.
  /// @param out The output stream to write messages to.
  /// @param amplitude The amplitude to set (0-255).
  /// @note This will set the realtime PWM amplitude for all drivers in the menu.
  /// @note This is only applicable when the mode is set to REALTIMEPWM.
  void set_realtime_pwm(std::ostream &out, uint8_t amplitude) {
    for (auto &driver : drivers_) {
      std::error_code ec;
      driver->set_rtp_pwm_unsigned(amplitude, ec);
      if (ec) {
        out << "Error setting realtime PWM: " << ec.message() << "\n";
      } else {
        out << "Set realtime PWM successfully.\n";
      }
    }
  }

protected:
  std::vector<std::shared_ptr<Driver>> drivers_;
}; // class Drv2605Menu
} // namespace espp
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
