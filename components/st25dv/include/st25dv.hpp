#pragma once

#include <atomic>
#include <cmath>
#include <functional>

#include "logger.hpp"
#include "ndef.hpp"
#include "task.hpp"

namespace espp {
/**
 * @brief Class for wireless communications using a ST25DV Dynamic NFC/RFID
 *        tag. The datasheet for the ST25DV can be found here:
 *        https://www.st.com/resource/en/datasheet/st25dv04k.pdf
 *
 * @note See
 * https://stackoverflow.com/questions/61622309/base-address-requirement-for-ndef-messages-on-type-5-tags
 * for some discussion about the Capability Container (CC) header that must be
 * the first data in the EEPROM.
 *
 * \section st25dv_ex1 St25dv Example
 * \snippet st25dv_example.cpp st25dv example
 */
class St25dv {
public:
  // NOTE: when the datasheet mentions E2 device select, they are talking
  // about Bit 4 of the address which selects between the data (user memory,
  // dynamic registers, FTM mailbox), and system memory
  static constexpr uint8_t DATA_ADDRESS = (0xA6 >> 1); ///< I2C Address for writing / reading data
  static constexpr uint8_t SYST_ADDRESS =
      (0xAE >> 1); ///< I2C Address for writing / reading system config

  /**
   * @brief Encapsulates the different flags / bit fields that the IT_STS
   * dynamic register holds for the different states it represents. Reading
   * this register clears it to 0x00.

   * @note RF events are reported in the IT_STS register even if GPO output is
   * disabled.
   */
  class IT_STS {
  public:
    static constexpr int RF_USER = 0b00000001;       ///< Manage GPO (set / reset GPO)
    static constexpr int RF_ACTIVITY = 0b00000010;   ///< Indicates RF Access
    static constexpr int RF_INTTERUPT = 0b00000100;  ///< GPO Interrupt request
    static constexpr int FIELD_FALLING = 0b00001000; ///< RF field is falling
    static constexpr int FIELD_RISING = 0b00010000;  ///< RF field is rising
    static constexpr int RF_PUT_MSG = 0b00100000;    ///< Message put by RF into FTM mailbox
    static constexpr int RF_GET_MSG =
        0b01000000; ///< Message read by RF from FTM mailbox, and end of message has been reached.
    static constexpr int RF_WRITE = 0b10000000; ///< Write in eeprom
  };

  /**
   * @brief Function to write bytes to St25dv.
   * @param addr I2C address to write to
   * @param data Data to be written.
   * @param length Number of bytes to write.
   */
  typedef std::function<void(uint8_t addr, uint8_t *data, uint8_t length)> write_fn;

  /**
   * @brief Function to read a sequence of bytes from St25dv.
   * @param addr I2C address to read from
   * @param reg_addr Start register address to read from.
   * @param data Pointer to memory which will be filled with data read
   *        from St25dv.
   * @param length Number of bytes to read
   */
  typedef std::function<void(uint8_t addr, uint16_t reg_addr, uint8_t *data, uint8_t length)>
      read_fn;

  /**
   * @brief Configuration information for the St25dv.
   */
  struct Config {
    write_fn write;                                       ///< Function to write to the device.
    read_fn read;                                         ///< Function to read from the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the component.  */
  };

  /**
   * @brief Construct the St25dv and start the update task.
   */
  St25dv(const Config &config)
      : write_(config.write), read_(config.read),
        logger_({.tag = "St25dv", .level = config.log_level}) {
    init();
  }

  /**
   * @brief Get the interrupt status register (dynamic IT_STS).
   * @note Reading the interrupt status register clears it.
   * @note The available states / flags in the register are available in \c
   *       St25dv::IT_STS.
   * @return The raw interrupt status register value read from the chip.
   */
  uint8_t get_interrupt_status() {
    uint8_t it_sts = 0;
    read_(DATA_ADDRESS, (uint16_t)Registers::IT_STS, &it_sts, 1);
    return it_sts;
  }

  /**
   * @brief Writes the provided record (along with CC header) to the EEPROM.
   * @note Right now this only supports 4 B CC headers (for memory less than
   *       16 Kbit).
   * @param record The new NDEF record to serialize to the NFC EEPROM.
   */
  void set_record(Ndef &record) {
    // add in the T5T tag compatibility container (CC) header
    std::vector<uint8_t> full_record;
    auto payload = record.serialize();
    full_record.resize(payload.size() + 6);
    /**
     * @note CC indicates how the tag can be accessed. There are two different
     *       types of CC used (depending on the size of the tag):
     *         1. 4 B CC (for memory size < 16 Kbit)
     *            | Byte0        | Byte1                       | Byte2       | Byte3          |
     *            |:------------:|:---------------------------:|:-----------:|:--------------:|
     *            | Magic Number | Version & Access Conditions | Memory Size | NFC Type 5 Tag |
     *         2. 8 B CC (for memory size > 16 Kbit)
     *            | Byte0        | Byte1                       | Byte2 | Byte3 |Byte4|Byte5| Byte6
     * | Byte7     |
     *            |:------------:|:---------------------------:|:-----:|:--------------:|:---:|:---:|:---------:|:---------:|
     *            | Magic Number | Version & Access Conditions | 0x00h | NFC Type 5 Tag | RFU | RFU
     * |Memory Size|Memory Size|
     */
    // capability container (T5T tag)
    full_record[0] = 0xE1; // magic number, should be 0xE1 or 0xE2 (for extended API)
    full_record[1] = 0x40; // CC version (1.0) and access condition (always, always) (version is
                           // b7b6.b5b4, read access is b3b2, write access is b1b0)
    full_record[2] = 0x40; // MLEN NDEF data size 512 bytes (0x40), expressed in blocks (set to 0 if
                           // tag is greater than 16 Kbit)
    full_record[3] = 5; // additional feature information (support multiple block read) (b0: support
                        // read multiple block, b1 & b2 : RFU, b3: supports lock block, b4: requires
                        // special frame format)
    // The message is preceded by a type5 tag header:
    Tlv tlv(Type5TagType::NDEF_MSG, payload.size());
    int tlv_size = tlv.size();
    memcpy(&full_record[4], tlv.raw, tlv_size);
    memcpy(&full_record[4 + tlv_size], payload.data(), payload.size());
    write(std::string_view{(const char *)full_record.data(), full_record.size()});
  }

  /**
   * @brief Write a raw sequence of bytes to the EEPROM.
   * @param payload Sequence of bytes to write.
   */
  void write(std::string_view payload) {
    uint8_t data[2 + payload.size()];
    data[0] = (uint8_t)(AREA_1_START_ADDR >> 8);
    data[1] = (uint8_t)(AREA_1_START_ADDR & 0xFF);
    memcpy(&data[2], payload.data(), payload.size());
    write_(DATA_ADDRESS, data, sizeof(data));
  }

  /**
   * @brief Read a sequence of bytes from the EEPROM.
   * @note This may contain raw NDEF bytes as well as the CC header.
   * @param *data Pointer to memory to be filled with bytes read.
   * @param length Number of bytes to read.
   * @param offset Optional offset to start reading from.
   */
  void read(uint8_t *data, uint8_t length, uint16_t offset = 0) {
    read_(DATA_ADDRESS, AREA_1_START_ADDR + offset, data, length);
  }

  /**
   * @brief Enable fast transfer mode (using up to 255 bytes at a time)
   *        between RF and I2C. After calling this, you can call transfer(),
   *        receive(), and get_ftm_length() for fast bi-directional
   *        communications between RF and I2C.
   */
  void start_fast_transfer_mode() {
    uint16_t reg_addr = (uint16_t)Registers::MB_CTRL;
    uint8_t data[3] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        // data
        MB_CTRL::EN,
    };
    write_(DATA_ADDRESS, data, sizeof(data));
  }

  /**
   * @brief Disable fast transfer mode (using up to 255 bytes at a time)
   *        between RF and I2C. After calling this, you cannot call transfer()
   *        or receive() without again calling start_fast_transfer_mode()
   *        first.
   */
  void stop_fast_transfer_mode() {
    uint16_t reg_addr = (uint16_t)Registers::MB_CTRL;
    uint8_t data[3] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        // data
        0,
    };
    write_(DATA_ADDRESS, data, sizeof(data));
  }

  /**
   * @brief Returns the available message length in the FTM message box.
   * @details Will return non-zero if the RF received data into the FTM.
   * @return Number of bytes (up to 255) available in the FTM message box.
   */
  uint8_t get_ftm_length() {
    uint8_t len = 0;
    read_(DATA_ADDRESS, (uint16_t)Registers::MB_LEN, &len, 1);
    return len;
  }

  /**
   * @brief Write data to the FTM message box to send.
   * @note Must call start_fast_transfer_mode() prior to use.
   * @param data Data to be written.
   * @param length Number of bytes to write.
   */
  void transfer(uint8_t *data, uint8_t length) { write_ftm(data, length); }

  /**
   * @brief Read data from the FTM message box.
   * @note Must call start_fast_transfer_mode() prior to use.
   * @note The length available to be read can be found by calling
   *       get_ftm_length().
   * @param data Pointer to memory to be filled with data.
   * @param length Number of bytes to read.
   */
  void receive(uint8_t *data, uint8_t length) { read_ftm(data, length); }

protected:
  void init() {
    logger_.info("Initializing");
    read_uuid();
    read_password();
    auto block_size_bytes = read_block_size_bytes();
    auto memory_size_blocks = read_memory_size_blocks();
    memory_size_bytes_ = block_size_bytes * memory_size_blocks;
    logger_.info("Memory size (B): {}", memory_size_bytes_);
  }

  void write_ftm(uint8_t *data, uint8_t length) {
    // must start from FTM_START_ADDR
    uint8_t all_data[2 + length];
    all_data[0] = (uint8_t)(FTM_START_ADDR >> 8);
    all_data[1] = (uint8_t)(FTM_START_ADDR & 0xFF);
    memcpy(&all_data[2], data, length);
    write_(DATA_ADDRESS, all_data, length + 2);
  }

  void read_ftm(uint8_t *data, uint8_t length, uint8_t offset = 0) {
    // read can start from any byte offset within the FTM mailbox.
    read_(DATA_ADDRESS, FTM_START_ADDR + offset, data, length);
  }

  void read_uuid() {
    uint8_t uuid[8];
    read_(SYST_ADDRESS, (uint16_t)Registers::UID, uuid, sizeof(uuid));
    memcpy(&uuid_, uuid, sizeof(uuid));
    logger_.debug("Got uuid: 0x{:016X}", uuid_);
  }

  uint8_t read_block_size_bytes() {
    uint8_t block_size_bytes = 0;
    read_(SYST_ADDRESS, (uint16_t)Registers::MEM_SIZE, &block_size_bytes, 1);
    logger_.debug("Block size (B): {}", block_size_bytes);
    return block_size_bytes;
  }

  uint16_t read_memory_size_blocks() {
    uint16_t memory_size_blocks = 0;
    read_(SYST_ADDRESS, (uint16_t)Registers::BLK_SIZE, (uint8_t *)&memory_size_blocks, 2);
    logger_.debug("Memory size (blocks): {}", memory_size_blocks);
    return memory_size_blocks;
  }

  void read_password() {
    uint8_t pswd[8];
    read_(SYST_ADDRESS, (uint16_t)Registers::I2C_PWD, pswd, sizeof(pswd));
    memcpy(&password_, pswd, sizeof(pswd));
    logger_.debug("Got pswd: 0x{:016X}", password_);
  }

  void present_password() {
    // length of messsage is 17 bytes, plus 2 for address
    uint8_t data[2 + 17] = {0};
    data[0] = (uint16_t)Registers::I2C_PWD >> 8;
    data[1] = (uint16_t)Registers::I2C_PWD & 0xFF;
    uint8_t *pswd_data = (uint8_t *)&password_;
    // validation code in the middle
    data[8 + 2] = 0x09;
    for (int i = 0; i < 4; i++) {
      data[2 + i] = pswd_data[i];
      data[2 + i + 4] = pswd_data[i + 4];
      data[2 + i + 9] = data[2 + i];
      data[2 + i + 13] = data[2 + i + 4];
    }
    logger_.debug("Presenting password: {}\n", data);
    write_(SYST_ADDRESS, data, sizeof(data));
  }

  /**
   * @brief Register map for the ST25DV.
   */
  enum class Registers : uint16_t {
    // System configuration registers, must be accessed with device select
    // E2=1, and a security session must be opened first by presenting a valid
    // I2C password.
    GPO_CONF = 0x0000, /**< Enable / Disable interrupts on GPO. */
    MEM_SIZE = 0x0014, /**< Memory size value in blocks, 2 bytes. */
    BLK_SIZE = 0x0016, /**< Block size value in bytes. */
    UID = 0x0018,      /**< Unique identifier, 8 bytes. */
    I2C_PWD = 0x0900,  /**< I2C Security session password, 8 bytes. */
    // Dynamic registers:
    GPO_CTRL = 0x2000, /**< GPO Control register. */
    EH_CTRL = 0x2002,  /**< Energy Harvesting management and usage status register. */
    RF_MNGT = 0x2003,  /**< RF Interface usage management. */
    I2C_SSO = 0x2004,  /**< I2C Security session status register. */
    IT_STS = 0x2005,   /**< Interruption status register. */
    MB_CTRL = 0x2006,  /**< Fast transfer mode control & status register. */
    MB_LEN = 0x2007,   /**< Length of fast transfer mode message. */
  };

  /**
   * @brief Type5 Tag Type-Length-Value (TLV) Type values
   */
  enum class Type5TagType : uint8_t {
    NDEF_MSG = 0x03,   ///< Type5 Tag NDEF Message (TLV-Type)
    TERMINATOR = 0xFE, ///< Type5 Tag Terminator
  };

  /**
   * @brief Type5 Tag Type-Length-Value (TLV) structure (defined by NFC
   *        forum).
   */
  struct Tlv {
    /**
     * @brief Construct the TLV and set the length/length16 members accordingly.
     * @param t The Type5TagType type of the message.
     * @param len The length of the message, in bytes.
     */
    Tlv(Type5TagType t, int len) {
      type = t;
      if (len < 255) {
        length = len;
      } else {
        length = 0xFF;
        length16 = len;
      }
    }

    union {
      struct {
        Type5TagType type; ///< Message type
        uint8_t length;    ///< Length if < 255 bytes
        // NOTE: only written if Length >= 255 bytes
        uint16_t length16; ///< Length if >= 255 bytes
      };
      uint8_t raw[4];
    };

    /**
     * @brief Get the number of bytes that the TLV will occupy, based on the
     *        length.
     * @return Number of bytes of raw that should be written to represent the
     *         TLV.
     */
    int size() { return length < 255 ? 2 : 4; }
  };

  class GPO {
  public:
    static constexpr int RF_USER_EN = 0b00000001;
    static constexpr int RF_ACTIVITY_EN = 0b00000010;
    static constexpr int RF_INTTERUPT_EN = 0b00000100;
    static constexpr int FIELD_CHANGE_EN = 0b00001000;
    static constexpr int RF_PUT_MSG_EN = 0b00010000;
    static constexpr int RF_GET_MSG_EN = 0b00100000;
    static constexpr int RF_WRITE_EN = 0b01000000;
    static constexpr int GPO_EN = 0b10000000;
  };

  class EH_CTRL {
  public:
    static constexpr int EH_EN = 0b00000001;
    static constexpr int EH_ON = 0b00000010;
    static constexpr int FIELD_ON = 0b00000100;
    static constexpr int VCC_ON = 0b00001000;
  };

  class MB_CTRL {
  public:
    static constexpr int EN = 0b00000001;
    static constexpr int HOST_PUT_MSG = 0b00000010;
    static constexpr int RF_PUT_MSG = 0b00000100;
    static constexpr int HOST_MISS_MSG = 0b00010000;
    static constexpr int RF_MISS_MSG = 0b00100000;
    static constexpr int HOST_CURRENT_MSG = 0b01000000;
    static constexpr int RF_CURRENT_MSG = 0b10000000;
  };

  static constexpr uint16_t AREA_1_START_ADDR =
      0x0000; /**< Start address of the first user memory area. */

  static constexpr uint16_t FTM_START_ADDR =
      0x2008;                           /**< Start address of the Fast Transfer Mode Mailbox. */
  static constexpr int FTM_SIZE = 0xFF; /**< Number of bytes in the Fast Transfer Mode Mailbox. */

  write_fn write_;
  read_fn read_;
  uint32_t memory_size_bytes_;
  uint64_t uuid_;
  uint64_t password_;
  Logger logger_;
};
} // namespace espp
