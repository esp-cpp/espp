#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "base_peripheral.hpp"

namespace espp {
/// \brief VL53L5CX multizone Time-of-Flight ranging sensor (4x4 / 8x8 zones)
/// \details
/// The VL53L5CX is a Time-of-Flight (ToF) sensor with a 63 degree diagonal
/// field of view that ranges up to 4 m in every zone of a 4x4 or 8x8 grid, at
/// up to 60 Hz (4x4) or 15 Hz (8x8). Every zone reports a distance, a signal
/// / ambient rate, a sigma, a reflectance estimate and a validity status; the
/// sensor can also run a motion indicator over the zones.
///
/// The sensor has no ROM firmware: this driver uploads the ~84 KB VL53L5CX
/// firmware (STMicroelectronics, bundled, see the BSD-3-Clause notice in the
/// sources) over I2C at every initialize(), then talks to it through the
/// "DCI" command interface. Initialization takes about two seconds at 400 kHz
/// (the sensor supports I2C Fast Mode Plus, 1 MHz). The logic is a port of
/// ST's Ultra Lite Driver (ULD) 1.1.2.
///
/// Results are reported per zone in row-major order (`zone = row * width +
/// col`, width 4 or 8); for the physical orientation of zone 0 see UM2884
/// ("Ranging data arrangement").
///
/// \note The communication uses 16-bit register addresses, like the VL53L4
/// driver (espp::Vl53l) in this component, but the two devices share nothing
/// else: use espp::Vl53l for the VL53L4CD / VL53L4CX single-zone sensors.
///
/// \note Only one target per zone is supported (the ULD default); the
/// crosstalk calibration and detection threshold plugins of the ULD are not
/// ported (the default crosstalk shape is loaded).
///
/// \see https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html
/// \see UM2884 "A guide to using the VL53L5CX multizone Time-of-Flight ranging
/// sensor with wide field of view Ultra Lite Driver (ULD)"
///
/// \section vl53l5cx_ex1 Example
/// \snippet vl53l5cx_example.cpp vl53l5cx example
class Vl53l5cx : public BasePeripheral<uint16_t, true> {
  using Base = BasePeripheral<uint16_t, true>;
  using Base::logger_;
  using Base::read_many_from_register;
  using Base::set_address;
  using Base::set_read;
  using Base::set_write;
  using Base::write_many_to_register;

public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x29; ///< 7-bit address (ULD's 8-bit 0x52)
  static constexpr size_t MAX_ZONES = 64;          ///< zones in 8x8 mode
  /// Longest I2C transaction the driver issues (an 8x8 result frame with every
  /// output enabled, plus the register address): size the bus / device
  /// timeout for it, see i2c_timeout_ms().
  static constexpr size_t MAX_TRANSFER_BYTES = 1440 + 2;
  /// \brief A transaction timeout that covers MAX_TRANSFER_BYTES at `clock_hz`
  ///        with a 2x margin (plus 10 ms), for the I2C bus and device configs
  /// \param clock_hz I2C clock
  /// \return Timeout in milliseconds
  static constexpr int i2c_timeout_ms(uint32_t clock_hz) {
    // 9 bits per byte on the wire
    const uint64_t transfer_ms =
        (uint64_t{MAX_TRANSFER_BYTES} * 9 * 1000 + clock_hz - 1) / clock_hz;
    return static_cast<int>(transfer_ms * 2 + 10);
  }

  /// \brief Number of ranging zones
  enum class Resolution : uint8_t {
    ZONES_4X4 = 16, ///< 4x4 zones, up to 60 Hz
    ZONES_8X8 = 64, ///< 8x8 zones, up to 15 Hz
  };

  /// \brief Ranging mode
  enum class RangingMode : uint8_t {
    CONTINUOUS = 1, ///< ranges back to back at the ranging frequency (the integration time is
                    ///< whatever the frequency allows; highest power)
    AUTONOMOUS = 3, ///< ranges once per period, for the integration time, then idles (lower
                    ///< power; the default)
  };

  /// \brief Which target a zone reports first
  enum class TargetOrder : uint8_t {
    CLOSEST = 1,   ///< the closest target
    STRONGEST = 2, ///< the target with the strongest signal (the default)
  };

  /// \brief Power mode
  enum class PowerMode : uint8_t {
    SLEEP = 0,  ///< low power, no ranging possible (settings are kept)
    WAKEUP = 1, ///< ready to range (the default)
  };

  /// \brief Validity of a zone's measurement (UM2884, "target status")
  enum class TargetStatus : uint8_t {
    NOT_UPDATED = 0,                     ///< ranging data not updated
    SIGNAL_RATE_TOO_LOW_SPAD_ARRAY = 1,  ///< signal rate too low on the SPAD array
    TARGET_PHASE = 2,                    ///< target phase
    SIGMA_TOO_HIGH = 3,                  ///< sigma estimator too high
    TARGET_CONSISTENCY_FAILED = 4,       ///< target consistency failed
    RANGE_VALID = 5,                     ///< range valid
    WRAP_AROUND_NOT_PERFORMED = 6,       ///< wrap-around not performed (typically the first range)
    RATE_CONSISTENCY_FAILED = 7,         ///< rate consistency failed
    SIGNAL_RATE_TOO_LOW_TARGET = 8,      ///< signal rate too low for the current target
    RANGE_VALID_LARGE_PULSE = 9,         ///< range valid with a large pulse (maybe a merged target)
    RANGE_VALID_NO_PREVIOUS_TARGET = 10, ///< range valid, no target detected at the previous range
    MEASUREMENT_CONSISTENCY_FAILED = 11, ///< measurement consistency failed
    TARGET_BLURRED = 12,                 ///< target blurred by another one (sharpener)
    TARGET_INCONSISTENT = 13,            ///< target detected but the data is inconsistent
    NO_TARGET = 255,                     ///< no target detected in the zone
  };

  /// \brief Motion indicator results (only meaningful after
  ///        enable_motion_indicator())
  struct MotionIndicator {
    uint32_t global_indicator_1{0};
    uint32_t global_indicator_2{0};
    uint8_t status{0};
    uint8_t nb_of_detected_aggregates{0};
    uint8_t nb_of_aggregates{0};
    uint8_t spare{0};
    std::array<uint32_t, 32> motion{}; ///< motion intensity per aggregate
  };
  static_assert(sizeof(MotionIndicator) == 140, "MotionIndicator must match the sensor layout");

  /// \brief One frame of ranging results. Only the first num_zones() entries
  ///        of every array are meaningful.
  struct RangingData {
    Resolution resolution{Resolution::ZONES_4X4};            ///< resolution the frame was ranged at
    std::array<uint32_t, MAX_ZONES> ambient_per_spad_kcps{}; ///< ambient noise, kcps / SPAD
    std::array<uint8_t, MAX_ZONES> nb_target_detected{};     ///< targets detected in the zone
    std::array<uint32_t, MAX_ZONES> nb_spads_enabled{};      ///< SPADs enabled for the zone
    std::array<uint32_t, MAX_ZONES> signal_per_spad_kcps{};  ///< return signal, kcps / SPAD
    std::array<uint16_t, MAX_ZONES> range_sigma_mm{};        ///< sigma of the distance, mm
    std::array<int16_t, MAX_ZONES> distance_mm{};            ///< distance, mm (0 if negative)
    std::array<uint8_t, MAX_ZONES> reflectance_percent{};    ///< estimated reflectance, %
    std::array<uint8_t, MAX_ZONES> target_status{};          ///< \see TargetStatus
    MotionIndicator motion_indicator{};                      ///< \see enable_motion_indicator

    /// \brief Number of valid zones in this frame (16 or 64)
    size_t num_zones() const { return static_cast<size_t>(resolution); }
    /// \brief Zones per row (4 or 8)
    size_t width() const { return resolution == Resolution::ZONES_8X8 ? 8 : 4; }
    /// \brief Index of the zone at (row, col), row-major
    size_t zone_index(size_t row, size_t col) const { return row * width() + col; }
    /// \brief Validity of a zone
    TargetStatus status(size_t zone) const {
      return static_cast<TargetStatus>(target_status[zone]);
    }
    /// \brief True when the zone's distance is valid (status 5 or 9)
    bool is_valid(size_t zone) const {
      const auto s = status(zone);
      return s == TargetStatus::RANGE_VALID || s == TargetStatus::RANGE_VALID_LARGE_PULSE;
    }
  };

  /// \brief Configuration
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the sensor
    Base::write_fn write = nullptr;           ///< Write function (16-bit register, MSB first)
    Base::read_fn read = nullptr;             ///< Read function
    bool auto_init = true; ///< Upload the firmware and initialize in the constructor (about
                           ///< two seconds; see initialize())
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief Constructor
  /// \param config Configuration
  /// \note With Config::auto_init the firmware is uploaded here; check
  ///       is_initialized() afterwards (or call initialize() yourself).
  explicit Vl53l5cx(const Config &config);

  /// \brief True once initialize() succeeded
  bool is_initialized() const { return initialized_; }

  /// \brief Reboot the sensor, upload the firmware, load the NVM offset data
  ///        and the default configuration (4x4, 1 Hz, autonomous, strongest
  ///        target first, 5 ms integration time).
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \note Takes about two seconds at 400 kHz I2C. Ranging must be restarted
  ///       (and settings re-applied) after it.
  bool initialize(std::error_code &ec);

  /// \brief Check the sensor answers with the VL53L5CX device / revision ids
  /// \param ec Error code if the I2C transaction failed
  /// \return True if a VL53L5CX answered
  bool is_alive(std::error_code &ec);

  /// \brief Change the sensor's I2C address (volatile: back to 0x29 at reset)
  /// \param address New 7-bit address
  /// \param ec Error code if unsuccessful
  /// \return True if successful; the driver then uses the new address
  bool set_i2c_address(uint8_t address, std::error_code &ec);

  /// \brief Read the power mode
  /// \param ec Error code if unsuccessful
  /// \return The power mode (WAKEUP on error)
  PowerMode get_power_mode(std::error_code &ec);
  /// \brief Set the power mode (not while ranging)
  /// \param mode SLEEP or WAKEUP
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_power_mode(PowerMode mode, std::error_code &ec);

  /// \brief Read the resolution
  /// \param ec Error code if unsuccessful
  /// \return ZONES_4X4 or ZONES_8X8 (4x4 on error)
  Resolution get_resolution(std::error_code &ec);
  /// \brief Set the resolution (not while ranging)
  /// \param resolution ZONES_4X4 or ZONES_8X8
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \note Re-applies the motion indicator configuration if it is enabled.
  ///       The ranging frequency limit depends on the resolution: set the
  ///       resolution first.
  bool set_resolution(Resolution resolution, std::error_code &ec);

  /// \brief Read the ranging frequency
  /// \param ec Error code if unsuccessful
  /// \return Frequency in Hz (0 on error)
  uint8_t get_ranging_frequency_hz(std::error_code &ec);
  /// \brief Set the ranging frequency (not while ranging)
  /// \param frequency_hz 1..60 Hz in 4x4, 1..15 Hz in 8x8
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_ranging_frequency_hz(uint8_t frequency_hz, std::error_code &ec);

  /// \brief Read the integration time (AUTONOMOUS ranging mode)
  /// \param ec Error code if unsuccessful
  /// \return Integration time in ms (0 on error)
  uint32_t get_integration_time_ms(std::error_code &ec);
  /// \brief Set the integration time (AUTONOMOUS mode; not while ranging)
  /// \param time_ms 2..1000 ms; must fit in the ranging period (in 8x8 four
  ///        integrations are done per frame)
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_integration_time_ms(uint32_t time_ms, std::error_code &ec);

  /// \brief Read the sharpener (how much a zone is blurred by its neighbours)
  /// \param ec Error code if unsuccessful
  /// \return Sharpener in percent (0 on error)
  uint8_t get_sharpener_percent(std::error_code &ec);
  /// \brief Set the sharpener (not while ranging)
  /// \param percent 0..99 (default 5)
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_sharpener_percent(uint8_t percent, std::error_code &ec);

  /// \brief Read the target order
  /// \param ec Error code if unsuccessful
  /// \return The target order (STRONGEST on error)
  TargetOrder get_target_order(std::error_code &ec);
  /// \brief Set the target order (not while ranging)
  /// \param order CLOSEST or STRONGEST
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_target_order(TargetOrder order, std::error_code &ec);

  /// \brief Read the ranging mode
  /// \param ec Error code if unsuccessful
  /// \return The ranging mode (AUTONOMOUS on error)
  RangingMode get_ranging_mode(std::error_code &ec);
  /// \brief Set the ranging mode (not while ranging)
  /// \param mode CONTINUOUS or AUTONOMOUS
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_ranging_mode(RangingMode mode, std::error_code &ec);

  /// \brief Enable the motion indicator (not while ranging)
  /// \details The sensor then reports, per aggregate of zones, how much the
  ///          scene moved between frames (RangingData::motion_indicator).
  /// \param distance_min_mm Start of the motion detection window, >= 400 mm
  /// \param distance_max_mm End of the window, <= 4000 mm and at most 1500 mm
  ///        past distance_min_mm
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool enable_motion_indicator(uint16_t distance_min_mm, uint16_t distance_max_mm,
                               std::error_code &ec);

  /// \brief Start ranging with the current settings
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool start_ranging(std::error_code &ec);
  /// \brief Stop ranging
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool stop_ranging(std::error_code &ec);
  /// \brief True between a successful start_ranging() and stop_ranging()
  bool is_ranging() const { return ranging_; }

  /// \brief Check whether a new frame is available (poll at a few ms)
  /// \param ec Error code if the I2C transaction failed
  /// \return True once per new frame
  bool is_data_ready(std::error_code &ec);
  /// \brief Read the latest frame (after is_data_ready() returned true)
  /// \param data Filled with the frame
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool get_ranging_data(RangingData &data, std::error_code &ec);

  /// \brief Name of a target status
  static const char *target_status_name(TargetStatus status);

protected:
  static constexpr uint8_t DEVICE_ID = 0xF0;
  static constexpr uint8_t REVISION_ID = 0x02;
  static constexpr uint8_t TARGETS_PER_ZONE = 1;
  static constexpr size_t NVM_DATA_SIZE = 492;
  static constexpr size_t OFFSET_BUFFER_SIZE = 488;
  static constexpr size_t XTALK_BUFFER_SIZE = 776;
  /// metadata + common data + every output block for 8x8, one target per zone
  static constexpr size_t MAX_RESULTS_SIZE =
      40 + 260 + 260 + 68 + (256 + 4) + (128 + 4) + (128 + 4) + (64 + 4) + (64 + 4) + 144 + 8;
  static constexpr size_t TEMP_BUFFER_SIZE = MAX_RESULTS_SIZE > 1024 ? MAX_RESULTS_SIZE : 1024;
  /// Bytes per I2C write transaction (the register address auto-increments);
  /// the firmware is uploaded in chunks of this size.
  static constexpr size_t MAX_WRITE_CHUNK = 512;
  static_assert(MAX_TRANSFER_BYTES >= TEMP_BUFFER_SIZE + 2 &&
                    MAX_TRANSFER_BYTES >= MAX_WRITE_CHUNK + 2,
                "MAX_TRANSFER_BYTES must cover the longest read and write");
  static constexpr int POLL_INTERVAL_MS = 10;
  static constexpr int POLL_ATTEMPTS = 200; // 2 s

  // DCI (firmware) parameter indices
  static constexpr uint16_t DCI_ZONE_CONFIG = 0x5450;
  static constexpr uint16_t DCI_FREQ_HZ = 0x5458;
  static constexpr uint16_t DCI_INT_TIME = 0x545C;
  static constexpr uint16_t DCI_FW_NB_TARGET = 0x5478;
  static constexpr uint16_t DCI_RANGING_MODE = 0xAD30;
  static constexpr uint16_t DCI_DSS_CONFIG = 0xAD38;
  static constexpr uint16_t DCI_TARGET_ORDER = 0xAE64;
  static constexpr uint16_t DCI_SHARPENER = 0xAED8;
  static constexpr uint16_t DCI_MOTION_DETECTOR_CFG = 0xBFAC;
  static constexpr uint16_t DCI_SINGLE_RANGE = 0xCD5C;
  static constexpr uint16_t DCI_OUTPUT_CONFIG = 0xCD60;
  static constexpr uint16_t DCI_OUTPUT_ENABLES = 0xCD68;
  static constexpr uint16_t DCI_OUTPUT_LIST = 0xCD78;
  static constexpr uint16_t DCI_PIPE_CONTROL = 0xCF78;
  static constexpr uint16_t UI_CMD_STATUS = 0x2C00;
  static constexpr uint16_t UI_CMD_START = 0x2C04;
  static constexpr uint16_t UI_CMD_END = 0x2FFF;

  // result block headers (one target per zone) and their indices
  static constexpr uint32_t START_BH = 0x0000000D;
  static constexpr uint32_t METADATA_BH = 0x54B400C0;
  static constexpr uint32_t COMMONDATA_BH = 0x54C00040;
  static constexpr uint32_t AMBIENT_RATE_BH = 0x54D00104;
  static constexpr uint32_t SPAD_COUNT_BH = 0x55D00404;
  static constexpr uint32_t NB_TARGET_DETECTED_BH = 0xCF7C0401;
  static constexpr uint32_t SIGNAL_RATE_BH = 0xCFBC0404;
  static constexpr uint32_t RANGE_SIGMA_MM_BH = 0xD2BC0402;
  static constexpr uint32_t DISTANCE_BH = 0xD33C0402;
  static constexpr uint32_t REFLECTANCE_BH = 0xD43C0401;
  static constexpr uint32_t TARGET_STATUS_BH = 0xD47C0401;
  static constexpr uint32_t MOTION_DETECT_BH = 0xCC5008C0;
  static constexpr uint16_t AMBIENT_RATE_IDX = 0x54D0;
  static constexpr uint16_t SPAD_COUNT_IDX = 0x55D0;
  static constexpr uint16_t NB_TARGET_DETECTED_IDX = 0xCF7C;
  static constexpr uint16_t SIGNAL_RATE_IDX = 0xCFBC;
  static constexpr uint16_t RANGE_SIGMA_MM_IDX = 0xD2BC;
  static constexpr uint16_t DISTANCE_IDX = 0xD33C;
  static constexpr uint16_t REFLECTANCE_EST_PC_IDX = 0xD43C;
  static constexpr uint16_t TARGET_STATUS_IDX = 0xD47C;
  static constexpr uint16_t MOTION_DETEC_IDX = 0xCC50;

  /// The motion indicator configuration written to DCI_MOTION_DETECTOR_CFG
  /// (layout of the ULD's VL53L5CX_Motion_Configuration)
  struct MotionConfiguration {
    int32_t ref_bin_offset;
    uint32_t detection_threshold;
    uint32_t extra_noise_sigma;
    uint32_t null_den_clip_value;
    uint8_t mem_update_mode;
    uint8_t mem_update_choice;
    uint8_t sum_span;
    uint8_t feature_length;
    uint8_t nb_of_aggregates;
    uint8_t nb_of_temporal_accumulations;
    uint8_t min_nb_for_global_detection;
    uint8_t global_indicator_format_1;
    uint8_t global_indicator_format_2;
    uint8_t spare_1;
    uint8_t spare_2;
    uint8_t spare_3;
    int8_t map_id[64];
    uint8_t indicator_format_1[32];
    uint8_t indicator_format_2[32];
  };
  static_assert(sizeof(MotionConfiguration) == 156, "MotionConfiguration must match the sensor");

  // raw register access (16-bit addresses)
  bool write_byte(uint16_t reg, uint8_t value, std::error_code &ec);
  bool read_byte(uint16_t reg, uint8_t &value, std::error_code &ec);
  bool write_multi(uint16_t reg, const uint8_t *data, size_t length, std::error_code &ec);
  bool read_multi(uint16_t reg, uint8_t *data, size_t length, std::error_code &ec);
  /// Poll `size` bytes at `reg` until (byte[pos] & mask) == expected (2 s)
  bool poll_for_answer(size_t size, size_t pos, uint16_t reg, uint8_t mask, uint8_t expected,
                       std::error_code &ec);
  /// Byte-swap every 32-bit word (the firmware is big-endian)
  static void swap_buffer(uint8_t *buffer, size_t size);

  // DCI parameter access
  bool dci_read_data(uint8_t *data, uint16_t index, uint16_t size, std::error_code &ec);
  bool dci_write_data(const uint8_t *data, uint16_t index, uint16_t size, std::error_code &ec);
  bool dci_replace_data(uint16_t index, uint16_t size, const uint8_t *new_data, uint16_t new_size,
                        uint16_t new_pos, std::error_code &ec);

  bool send_offset_data(Resolution resolution, std::error_code &ec);
  bool send_xtalk_data(Resolution resolution, std::error_code &ec);
  bool refuse_while_ranging(const char *what, std::error_code &ec);
  bool read_resolution_locked(Resolution &resolution, std::error_code &ec);
  bool write_motion_configuration(std::error_code &ec);
  void motion_set_resolution(Resolution resolution);

  std::recursive_mutex mutex_;
  std::atomic<bool> initialized_{false}; ///< read lock-free by is_initialized()
  std::atomic<bool> ranging_{false};     ///< read lock-free by is_ranging()
  Resolution resolution_{Resolution::ZONES_4X4};
  uint8_t stream_count_{255};
  uint32_t data_read_size_{0};
  bool motion_enabled_{false};
  MotionConfiguration motion_config_{};
  std::array<uint8_t, OFFSET_BUFFER_SIZE> offset_data_{};
  std::array<uint8_t, XTALK_BUFFER_SIZE> xtalk_data_{};
  std::array<uint8_t, TEMP_BUFFER_SIZE> temp_buffer_{};
};
} // namespace espp
