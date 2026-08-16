#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "base_peripheral.hpp"

namespace espp {
/// Driver for the Semtech SX126x series of sub-GHz LoRa radio transceivers
/// (SX1261, SX1262) and compatible chips (LLCC68).
///
/// The SX126x is a command-based SPI peripheral: each operation is an opcode
/// followed by parameters, and read operations return data in the same SPI
/// transaction (with the chip select held asserted for the entire command).
/// Because of this, the driver requires the <tt>write_then_read</tt> function provided
/// in the configuration to perform the write phase and the read phase within a
/// single chip-select assertion - e.g. by performing one full-duplex transfer
/// of the concatenated write + read lengths. See the example for how to do
/// this with the espp::Spi component.
///
/// The chip signals command processing via its BUSY pin, which must be
/// provided via the <tt>is_busy</tt> function in the configuration. The <tt>reset</tt>
/// function (driving the active-low NRESET pin) is optional but recommended.
///
/// Interrupt-driven operation is supported by connecting the radio's DIO1 pin
/// to a GPIO interrupt (e.g. with the espp::Interrupt component) and calling
/// handle_dio1_interrupt() from the interrupt handler / task. Fully-polled
/// operation (no DIO1 wiring) is also supported via the blocking transmit()
/// and by periodically calling service_interrupts().
///
/// This driver only implements the LoRa modem (not FSK), and always uses
/// explicit (variable-length) headers.
///
/// \section sx126x_example Example
/// \snippet sx126x_example.cpp sx126x example
class Sx126x : public BasePeripheral<uint8_t, false> {
public:
  /// The chip variant controlled by this driver. The variants differ in their
  /// power amplifier configuration (SX1261: low power, up to +15 dBm; SX1262 /
  /// LLCC68: high power, up to +22 dBm) and supported spreading factors
  /// (LLCC68 supports only SF5-SF11).
  enum class Variant : uint8_t {
    SX1261, ///< Semtech SX1261 (up to +15 dBm)
    SX1262, ///< Semtech SX1262 (up to +22 dBm)
    LLCC68, ///< Semtech LLCC68 (up to +22 dBm, SF5-SF11 only)
  };

  /// LoRa spreading factor (SF). Higher spreading factors increase range and
  /// time-on-air, and decrease data rate.
  enum class SpreadingFactor : uint8_t {
    SF5 = 5,   ///< 5 chips / symbol
    SF6 = 6,   ///< 6 chips / symbol
    SF7 = 7,   ///< 7 chips / symbol
    SF8 = 8,   ///< 8 chips / symbol
    SF9 = 9,   ///< 9 chips / symbol
    SF10 = 10, ///< 10 chips / symbol
    SF11 = 11, ///< 11 chips / symbol
    SF12 = 12, ///< 12 chips / symbol
  };

  /// LoRa signal bandwidth.
  enum class Bandwidth : uint8_t {
    BW_7_8_KHZ = 0x00,   ///< 7.81 kHz
    BW_10_4_KHZ = 0x08,  ///< 10.42 kHz
    BW_15_6_KHZ = 0x01,  ///< 15.63 kHz
    BW_20_8_KHZ = 0x09,  ///< 20.83 kHz
    BW_31_25_KHZ = 0x02, ///< 31.25 kHz
    BW_41_7_KHZ = 0x0A,  ///< 41.67 kHz
    BW_62_5_KHZ = 0x03,  ///< 62.50 kHz
    BW_125_KHZ = 0x04,   ///< 125 kHz
    BW_250_KHZ = 0x05,   ///< 250 kHz
    BW_500_KHZ = 0x06,   ///< 500 kHz
  };

  /// LoRa forward error correction coding rate.
  enum class CodingRate : uint8_t {
    CR_4_5 = 0x01, ///< 4/5
    CR_4_6 = 0x02, ///< 4/6
    CR_4_7 = 0x03, ///< 4/7
    CR_4_8 = 0x04, ///< 4/8
  };

  /// Power amplifier ramp time.
  enum class RampTime : uint8_t {
    RAMP_10_US = 0x00,   ///< 10 us
    RAMP_20_US = 0x01,   ///< 20 us
    RAMP_40_US = 0x02,   ///< 40 us
    RAMP_80_US = 0x03,   ///< 80 us
    RAMP_200_US = 0x04,  ///< 200 us
    RAMP_800_US = 0x05,  ///< 800 us
    RAMP_1700_US = 0x06, ///< 1700 us
    RAMP_3400_US = 0x07, ///< 3400 us
  };

  /// Standby oscillator configuration.
  enum class StandbyMode : uint8_t {
    RC = 0x00,   ///< Standby with 13 MHz RC oscillator (lowest power)
    XOSC = 0x01, ///< Standby with crystal oscillator running (faster TX/RX entry)
  };

  /// Interrupt flags reported by the radio (bitfield). These match the chip's
  /// IRQ register bit positions.
  enum Irq : uint16_t {
    IRQ_NONE = 0x0000,              ///< No interrupts
    IRQ_TX_DONE = 0x0001,           ///< Packet transmission completed
    IRQ_RX_DONE = 0x0002,           ///< Packet reception completed
    IRQ_PREAMBLE_DETECTED = 0x0004, ///< Preamble detected
    IRQ_SYNC_WORD_VALID = 0x0008,   ///< Sync word valid (FSK only)
    IRQ_HEADER_VALID = 0x0010,      ///< LoRa header received & valid
    IRQ_HEADER_ERR = 0x0020,        ///< LoRa header CRC error
    IRQ_CRC_ERR = 0x0040,           ///< Payload CRC error
    IRQ_CAD_DONE = 0x0080,          ///< Channel activity detection finished
    IRQ_CAD_DETECTED = 0x0100,      ///< Channel activity detected
    IRQ_TIMEOUT = 0x0200,           ///< RX or TX timeout
    IRQ_ALL = 0x03FF,               ///< All interrupts
  };

  /// Status of a received packet.
  struct PacketStatus {
    float rssi{0};        ///< Average RSSI over the packet, in dBm
    float snr{0};         ///< Estimated SNR of the packet, in dB
    float signal_rssi{0}; ///< RSSI of the despread LoRa signal, in dBm
  };

  /// A received packet, as provided to the receive callback.
  struct RxPacket {
    std::vector<uint8_t> data{}; ///< The packet payload
    PacketStatus status{};       ///< RSSI / SNR of the packet
  };

  /// Callback invoked (from the caller of handle_dio1_interrupt /
  /// service_interrupts) when a packet has been received.
  typedef std::function<void(const RxPacket &packet)> receive_callback_fn;

  /// Callback invoked when a packet transmission has completed.
  typedef std::function<void(void)> transmit_done_callback_fn;

  /// Callback invoked when channel activity detection completes. The
  /// parameter is true if activity was detected.
  typedef std::function<void(bool detected)> cad_callback_fn;

  /// Function which returns the state of the radio's BUSY pin (true = busy).
  typedef std::function<bool(void)> is_busy_fn;

  /// Function which drives the radio's active-low NRESET pin. The parameter
  /// is the logic level to drive (false = held in reset).
  typedef std::function<void(bool level)> reset_fn;

  /// LoRa modem configuration. The defaults match the Meshtastic "LongFast"
  /// preset modulation (US frequency shown; set the frequency for your
  /// region / channel).
  struct RadioConfig {
    uint32_t frequency_hz = 906875000;                        ///< RF center frequency, in Hz
    int8_t tx_power_dbm = 22;                                 ///< TX power in dBm. Clamped to the
                                                              ///< variant's supported range.
    RampTime ramp_time = RampTime::RAMP_200_US;               ///< PA ramp time
    SpreadingFactor spreading_factor = SpreadingFactor::SF11; ///< Spreading factor
    Bandwidth bandwidth = Bandwidth::BW_250_KHZ;              ///< Signal bandwidth
    CodingRate coding_rate = CodingRate::CR_4_5;              ///< Coding rate
    uint16_t preamble_length = 16;                            ///< Preamble length in symbols
    bool crc_enabled = true;     ///< Whether to append/check payload CRC
    bool invert_iq = false;      ///< Whether to invert the IQ signals
    uint8_t sync_word = 0x2B;    ///< LoRa sync word. 0x12 = private networks, 0x34
                                 ///< = public (LoRaWAN), 0x2B = Meshtastic.
    bool rx_boosted_gain = true; ///< Use the boosted-gain RX mode (~2 dB better
                                 ///< sensitivity for ~0.7 mA more current)
  };

  /// Configuration for the Sx126x driver.
  struct Config {
    Variant variant = Variant::SX1262; ///< Which chip this is
    BasePeripheral::write_fn write;    ///< Function to write bytes to the radio (one full
                                       ///< chip-select assertion per call)
    BasePeripheral::write_then_read_fn write_then_read; ///< Function to write then read bytes
                                                        ///< from the radio, holding chip-select
                                                        ///< asserted across both phases
    is_busy_fn is_busy;                                 ///< Function returning the BUSY pin state
    reset_fn reset = nullptr;  ///< Optional function driving the NRESET pin
    float tcxo_voltage = 0.0f; ///< If > 0, DIO3 is used to power a TCXO at this voltage
                               ///< (1.6-3.3V). 0 disables TCXO control (crystal used).
    std::chrono::microseconds tcxo_delay =
        std::chrono::microseconds(5000);      ///< TCXO stabilization delay
    bool use_dio2_as_rf_switch = true;        ///< Whether DIO2 controls the RF switch (true on most
                                              ///< modules, including the T-Deck and M5Stack LoRa
                                              ///< modules)
    bool use_dcdc_regulator = true;           ///< Use the DC-DC regulator (true on most modules)
                                              ///< instead of only the LDO
    RadioConfig radio_config{};               ///< Initial radio (modem) configuration
    receive_callback_fn on_receive = nullptr; ///< Called when a packet is received
    transmit_done_callback_fn on_transmit_done = nullptr; ///< Called when transmission completes
    cad_callback_fn on_cad_done = nullptr;                ///< Called when CAD completes
    bool auto_init = true; ///< Whether to initialize the radio on construction
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the driver
  };

  /// Constructor
  /// \param config The configuration for the driver
  explicit Sx126x(const Config &config);

  /// Initialize the radio: reset (if a reset function was provided), verify
  /// SPI communications, configure the regulator / TCXO / RF switch,
  /// calibrate, and apply the radio configuration.
  /// \param ec The error code to set if there is an error
  /// \return True if initialization succeeded
  bool initialize(std::error_code &ec);

  /// Whether the radio has been successfully initialized
  /// \return True if the radio has been initialized
  bool is_initialized() const { return initialized_; }

  /// Apply a new radio (modem) configuration. The radio is placed in standby.
  /// \param config The radio configuration to apply
  /// \param ec The error code to set if there is an error
  /// \return True if the configuration was applied
  bool set_radio_config(const RadioConfig &config, std::error_code &ec);

  /// Get the current radio (modem) configuration
  /// \return The current radio configuration
  const RadioConfig &radio_config() const { return config_.radio_config; }

  /// Set the RF center frequency. Also performs image calibration for the
  /// new frequency band.
  /// \param frequency_hz The frequency in Hz
  /// \param ec The error code to set if there is an error
  void set_frequency(uint32_t frequency_hz, std::error_code &ec);

  /// Set the TX output power.
  /// \param power_dbm The power in dBm; clamped to the variant's range
  ///        (SX1261: -17 to +15, SX1262/LLCC68: -9 to +22)
  /// \param ec The error code to set if there is an error
  void set_tx_power(int8_t power_dbm, std::error_code &ec);

  /// Set the LoRa sync word.
  /// \param sync_word The sync word (0x12 private, 0x34 public, 0x2B Meshtastic)
  /// \param ec The error code to set if there is an error
  void set_sync_word(uint8_t sync_word, std::error_code &ec);

  /// Set the callback invoked when a packet is received
  /// \param callback The callback to invoke
  void set_receive_callback(const receive_callback_fn &callback);

  /// Set the callback invoked when a transmission completes
  /// \param callback The callback to invoke
  void set_transmit_done_callback(const transmit_done_callback_fn &callback);

  /// Set the callback invoked when channel activity detection completes
  /// \param callback The callback to invoke
  void set_cad_callback(const cad_callback_fn &callback);

  /// Transmit a packet, blocking until transmission completes or the timeout
  /// elapses. Polls the radio's IRQ status, so this works without DIO1 wired.
  /// The radio is returned to standby (or receive, if it was receiving)
  /// afterwards.
  /// \param data The payload to transmit (1-255 bytes)
  /// \param timeout The maximum time to wait for transmission to complete
  /// \param ec The error code to set if there is an error
  /// \return True if the packet was transmitted
  bool transmit(std::span<const uint8_t> data, std::chrono::milliseconds timeout,
                std::error_code &ec);

  /// Start transmitting a packet without blocking. Completion is signaled via
  /// the transmit-done callback when handle_dio1_interrupt() /
  /// service_interrupts() is called.
  /// \param data The payload to transmit (1-255 bytes)
  /// \param ec The error code to set if there is an error
  /// \return True if the transmission was started
  bool start_transmit(std::span<const uint8_t> data, std::error_code &ec);

  /// Enter continuous receive mode. Received packets are delivered via the
  /// receive callback when handle_dio1_interrupt() / service_interrupts() is
  /// called.
  /// \param ec The error code to set if there is an error
  /// \return True if receive mode was entered
  bool start_receive(std::error_code &ec);

  /// Start a channel activity detection. The result is delivered via the CAD
  /// callback when handle_dio1_interrupt() / service_interrupts() is called.
  /// \param ec The error code to set if there is an error
  /// \return True if CAD was started
  bool start_cad(std::error_code &ec);

  /// Put the radio into standby mode.
  /// \param mode The standby oscillator mode
  /// \param ec The error code to set if there is an error
  void standby(std::error_code &ec, StandbyMode mode = StandbyMode::RC);

  /// Put the radio into its lowest-power sleep mode (cold start, configuration
  /// retained). Waking requires any SPI transaction (e.g. standby()).
  /// \param ec The error code to set if there is an error
  void sleep(std::error_code &ec);

  /// Whether the radio is currently in receive mode
  /// \return True if the radio was last commanded into receive mode
  bool is_receiving() const { return receiving_; }

  /// Whether a preamble/header has been detected and reception is likely in
  /// progress. Useful for channel-activity checks before transmitting.
  /// \param ec The error code to set if there is an error
  /// \return True if the radio has detected preamble/valid header since the
  ///         last packet completed
  bool is_reception_in_progress(std::error_code &ec);

  /// Get the instantaneous RSSI measured by the radio (only meaningful in
  /// receive mode).
  /// \param ec The error code to set if there is an error
  /// \return The instantaneous RSSI in dBm
  float get_rssi_inst(std::error_code &ec);

  /// Handle a DIO1 interrupt: read & clear the radio's IRQ flags and invoke
  /// the appropriate callbacks (receive / transmit-done / CAD). Call this
  /// from a task context (e.g. an espp::Interrupt callback) - it performs SPI
  /// transactions and must not be called from an ISR.
  /// \param ec The error code to set if there is an error
  /// \return The IRQ flags which were set (bitfield of Irq values)
  uint16_t handle_dio1_interrupt(std::error_code &ec);

  /// Poll the radio's IRQ status and dispatch callbacks - identical to
  /// handle_dio1_interrupt(), provided for readability in polled operation.
  /// \param ec The error code to set if there is an error
  /// \return The IRQ flags which were set (bitfield of Irq values)
  uint16_t service_interrupts(std::error_code &ec) { return handle_dio1_interrupt(ec); }

  /// Get the time-on-air for a payload with the current radio configuration.
  /// \param payload_length The payload length in bytes
  /// \return The approximate time-on-air
  std::chrono::milliseconds time_on_air(size_t payload_length) const;

  /// Read a radio register (for advanced use).
  /// \param address The register address
  /// \param ec The error code to set if there is an error
  /// \return The register value
  uint8_t read_radio_register(uint16_t address, std::error_code &ec);

  /// Write a radio register (for advanced use).
  /// \param address The register address
  /// \param value The value to write
  /// \param ec The error code to set if there is an error
  void write_radio_register(uint16_t address, uint8_t value, std::error_code &ec);

protected:
  static constexpr uint32_t XTAL_FREQ = 32000000;
  static constexpr uint32_t FREQ_DIV = 33554432; // 2^25
  static constexpr uint8_t MAX_PAYLOAD_LENGTH = 255;

  // Command opcodes
  static constexpr uint8_t CMD_SET_SLEEP = 0x84;
  static constexpr uint8_t CMD_SET_STANDBY = 0x80;
  static constexpr uint8_t CMD_SET_FS = 0xC1;
  static constexpr uint8_t CMD_SET_TX = 0x83;
  static constexpr uint8_t CMD_SET_RX = 0x82;
  static constexpr uint8_t CMD_STOP_TIMER_ON_PREAMBLE = 0x9F;
  static constexpr uint8_t CMD_SET_CAD = 0xC5;
  static constexpr uint8_t CMD_SET_REGULATOR_MODE = 0x96;
  static constexpr uint8_t CMD_CALIBRATE = 0x89;
  static constexpr uint8_t CMD_CALIBRATE_IMAGE = 0x98;
  static constexpr uint8_t CMD_SET_PA_CONFIG = 0x95;
  static constexpr uint8_t CMD_SET_RX_TX_FALLBACK_MODE = 0x93;
  static constexpr uint8_t CMD_WRITE_REGISTER = 0x0D;
  static constexpr uint8_t CMD_READ_REGISTER = 0x1D;
  static constexpr uint8_t CMD_WRITE_BUFFER = 0x0E;
  static constexpr uint8_t CMD_READ_BUFFER = 0x1E;
  static constexpr uint8_t CMD_SET_DIO_IRQ_PARAMS = 0x08;
  static constexpr uint8_t CMD_GET_IRQ_STATUS = 0x12;
  static constexpr uint8_t CMD_CLEAR_IRQ_STATUS = 0x02;
  static constexpr uint8_t CMD_SET_DIO2_AS_RF_SWITCH = 0x9D;
  static constexpr uint8_t CMD_SET_DIO3_AS_TCXO_CTRL = 0x97;
  static constexpr uint8_t CMD_SET_RF_FREQUENCY = 0x86;
  static constexpr uint8_t CMD_SET_PACKET_TYPE = 0x8A;
  static constexpr uint8_t CMD_GET_PACKET_TYPE = 0x11;
  static constexpr uint8_t CMD_SET_TX_PARAMS = 0x8E;
  static constexpr uint8_t CMD_SET_MODULATION_PARAMS = 0x8B;
  static constexpr uint8_t CMD_SET_PACKET_PARAMS = 0x8C;
  static constexpr uint8_t CMD_SET_CAD_PARAMS = 0x88;
  static constexpr uint8_t CMD_SET_BUFFER_BASE_ADDRESS = 0x8F;
  static constexpr uint8_t CMD_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0;
  static constexpr uint8_t CMD_GET_STATUS = 0xC0;
  static constexpr uint8_t CMD_GET_RSSI_INST = 0x15;
  static constexpr uint8_t CMD_GET_RX_BUFFER_STATUS = 0x13;
  static constexpr uint8_t CMD_GET_PACKET_STATUS = 0x14;
  static constexpr uint8_t CMD_GET_DEVICE_ERRORS = 0x17;
  static constexpr uint8_t CMD_CLEAR_DEVICE_ERRORS = 0x07;

  // Register addresses
  static constexpr uint16_t REG_SYNC_WORD_MSB = 0x0740;
  static constexpr uint16_t REG_SYNC_WORD_LSB = 0x0741;
  static constexpr uint16_t REG_RX_GAIN = 0x08AC;
  static constexpr uint16_t REG_TX_CLAMP_CONFIG = 0x08D8;
  static constexpr uint16_t REG_TX_MODULATION = 0x0889;
  static constexpr uint16_t REG_IQ_POLARITY = 0x0736;
  static constexpr uint16_t REG_RTC_CONTROL = 0x0902;
  static constexpr uint16_t REG_EVENT_MASK = 0x0944;

  bool wait_on_busy(std::error_code &ec,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(10));
  void cmd_write(uint8_t opcode, const uint8_t *params, size_t length, std::error_code &ec);
  void cmd_read(uint8_t opcode, const uint8_t *params, size_t params_length, uint8_t *response,
                size_t response_length, std::error_code &ec);
  void write_registers(uint16_t address, const uint8_t *data, size_t length, std::error_code &ec);
  void read_registers(uint16_t address, uint8_t *data, size_t length, std::error_code &ec);
  void write_tx_buffer(std::span<const uint8_t> data, std::error_code &ec);

  void set_packet_params(uint8_t payload_length, std::error_code &ec);
  void set_modulation_params(std::error_code &ec);
  void set_pa_config(std::error_code &ec);
  void set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, std::error_code &ec);
  uint16_t get_irq_status(std::error_code &ec);
  void clear_irq_status(uint16_t mask, std::error_code &ec);
  uint16_t get_device_errors(std::error_code &ec);
  void clear_device_errors(std::error_code &ec);
  void calibrate_all(std::error_code &ec);
  void calibrate_image(uint32_t frequency_hz, std::error_code &ec);
  void apply_workarounds(std::error_code &ec);
  bool read_packet(RxPacket &packet, std::error_code &ec);
  PacketStatus get_packet_status(std::error_code &ec);
  bool low_data_rate_optimize() const;
  float bandwidth_hz() const;

  Config config_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> receiving_{false};
  std::atomic<bool> transmitting_{false};
  // TX completion, recorded by handle_dio1_interrupt() (the single reader of
  // the IRQ status) so that blocking transmit() does not race the DIO1
  // interrupt to read / clear the TX_DONE flag
  std::atomic<bool> tx_done_{false};
  std::atomic<bool> tx_timeout_{false};
};
} // namespace espp
