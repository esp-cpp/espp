#pragma once

#include <atomic>
#include <cmath>
#include <functional>
#include <span>
#include <vector>

#include "base_peripheral.hpp"
#include "ndef.hpp"

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
class St25dv : public BasePeripheral {
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
   * @brief Configuration information for the St25dv.
   */
  struct Config {
    BasePeripheral::write_fn write;                       ///< Function to write to the device.
    BasePeripheral::read_fn read;                         ///< Function to read from the device.
    bool auto_init{true};                                 ///< Automatically initialize the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the component.  */
  };

  /**
   * @brief Construct the St25dv with the provided configuration.
   */
  explicit St25dv(const Config &config)
      : BasePeripheral({.address = DATA_ADDRESS, .write = config.write, .read = config.read},
                       "St25dv", config.log_level) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
    }
  }

  /**
   * @brief Initialize the St25dv.
   * @param &ec Error code to be filled with any errors that occur during
   *            initialization.
   */
  void initialize(std::error_code &ec) { init(ec); }

  /**
   * @brief Get the interrupt status register (dynamic IT_STS).
   * @param &ec Error code to be filled with any errors that occur during
   *            reading.
   * @note Reading the interrupt status register clears it.
   * @note The available states / flags in the register are available in \c
   *       St25dv::IT_STS.
   * @return The raw interrupt status register value read from the chip.
   */
  uint8_t get_interrupt_status(std::error_code &ec) {
    uint8_t it_sts = 0;
    // NOTE: we have to use the underlying peripheral's functions since we
    // have to use different device addresses for the different types of
    // registers
    read_syst_register(Registers::IT_STS, &it_sts, 1, ec);
    if (ec) {
      return 0;
    }
    return it_sts;
  }

  /**
   * @brief Writes the provided record (along with CC header) to the EEPROM.
   * @note Right now this only supports 4 B CC headers (for memory less than
   *       16 Kbit).
   * @param record The new NDEF record to serialize to the NFC EEPROM.
   * @param &ec Error code to be filled with any errors that occur during
   *            writing.
   */
  void set_record(Ndef &record, std::error_code &ec) {
    auto record_data = record.serialize();
    set_record(record_data, ec);
  }

  /**
   * @brief Writes the provided record (along with CC header) to the EEPROM.
   * @param &record_data The serialized NDEF record to write to the NFC EEPROM.
   * @param &ec Error code to be filled with any errors that occur during
   *            writing.
   */
  void set_record(const std::vector<uint8_t> &record_data, std::error_code &ec) {
    // clang-format off
    /**
     * @note CC indicates how the tag can be accessed. There are two different
     *       types of CC used (depending on the size of the tag):
     *         1. 4 B CC (for memory size < 16 Kbit)
     *            | Byte0        | Byte1                       | Byte2       | Byte3          |
     *            |:------------:|:---------------------------:|:-----------:|:--------------:|
     *            | Magic Number | Version & Access Conditions | Memory Size | NFC Type 5 Tag |
     *         2. 8 B CC (for memory size > 16 Kbit)
     *            | Byte0        | Byte1                       | Byte2 | Byte3          |Byte4|Byte5| Byte6     | Byte7     |
     *            |:------------:|:---------------------------:|:-----:|:--------------:|:---:|:---:|:---------:|:---------:|
     *            | Magic Number | Version & Access Conditions | 0x00h | NFC Type 5 Tag | RFU | RFU |Memory Size|Memory Size|
     */
    // clang-format on
    size_t cc_size = 4;
    size_t ndef_size = record_data.size();
    Tlv tlv(Type5TagType::NDEF_MSG, ndef_size);
    size_t tlv_size = tlv.size();
    size_t total_size = cc_size + tlv_size + ndef_size + 1; // +1 for terminator TLV
    std::vector<uint8_t> full_record(total_size, 0);
    int offset = 0;
    // NDEF is preceded by a CC header
    full_record[offset++] = 0xE1; // magic number, should be 0xE1 or 0xE2 (for extended API)
    full_record[offset++] = 0x40; // CC version (1.0) and access condition (always, always) (version
                                  // is b7b6.b5b4, read access is b3b2, write access is b1b0)
    full_record[offset++] = 0x40; // MLEN NDEF data size 512 bytes (0x40), expressed in blocks (set
                                  // to 0 if tag is greater than 16 Kbit)
    full_record[offset++] = 5; // additional feature information (support multiple block read) (b0:
                               // support read multiple block, b1 & b2 : RFU, b3: supports lock
                               // block, b4: requires special frame format)
    // The message is preceded by a type5 tag header:
    tlv.serialize(full_record, offset);
    offset += tlv_size;
    // debug log the record up to the NDEF data
    logger_.debug("Writing {} bytes of record header: {::#04x}", offset,
                  std::span(full_record.data(), offset));

    // copy the NDEF record data
    logger_.debug("Writing {} bytes of record data: {::#04x}", ndef_size, record_data);
    memcpy(&full_record[offset], record_data.data(), record_data.size());
    offset += ndef_size;
    // add the TLV terminator (0xFE)
    full_record[offset] = (uint8_t)Type5TagType::TERMINATOR;
    write(std::string_view{(const char *)full_record.data(), full_record.size()}, ec);
  }

  /**
   * @brief Writes the provided records (along with CC header) to the EEPROM.
   * @param &records Vector of NDEF records to serialize to the NFC EEPROM.
   * @param &ec Error code to be filled with any errors that occur during
   *            writing.
   */
  void set_records(std::vector<Ndef> &records, std::error_code &ec) {
    std::vector<uint8_t> record_data;
    size_t total_size = 0;
    for (auto &record : records) {
      total_size += record.get_size();
    }
    record_data.reserve(total_size);
    for (int i = 0; i < records.size(); i++) {
      // set the first record to have MB = 1
      bool message_begin = (i == 0);
      // set the last record to have ME = 1
      bool message_end = (i == (records.size() - 1));
      auto &record = records[i];
      auto serialized_record = record.serialize(message_begin, message_end);
      record_data.insert(record_data.end(), serialized_record.begin(), serialized_record.end());
      serialized_record.resize(0);
    }
    set_record(record_data, ec);
  }

  /**
   * @brief Write a raw sequence of bytes to the EEPROM.
   * @param payload Sequence of bytes to write.
   * @param &ec Error code to be filled with any errors that occur during
   *            writing.
   */
  void write(std::string_view payload, std::error_code &ec) {
    size_t payload_size = payload.size();
    uint8_t data[2 + payload_size];
    data[0] = (uint8_t)(AREA_1_START_ADDR >> 8);
    data[1] = (uint8_t)(AREA_1_START_ADDR & 0xFF);
    memcpy(&data[2], payload.data(), payload_size);
    bool success = base_config_.write(DATA_ADDRESS, data, sizeof(data));
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    } else {
      ec.clear();
    }
  }

  /**
   * @brief Read a sequence of bytes from the EEPROM starting at offset 0.
   * @note This may contain raw NDEF bytes as well as the CC header.
   * @param *data Pointer to memory to be filled with bytes read.
   * @param length Number of bytes to read.
   * @param &ec Error code to be filled with any errors that occur during
   *            reading.
   */
  void read(uint8_t *data, uint8_t length, std::error_code &ec) { read(data, length, 0, ec); }

  /**
   * @brief Read a sequence of bytes from the EEPROM starting at the provided
   *        offset.
   * @note This may contain raw NDEF bytes as well as the CC header.
   * @param *data Pointer to memory to be filled with bytes read.
   * @param length Number of bytes to read.
   * @param offset Offset to start reading from.
   * @param &ec Error code to be filled with any errors that occur during
   *            reading.
   */
  void read(uint8_t *data, uint8_t length, uint16_t offset, std::error_code &ec) {
    uint16_t reg = AREA_1_START_ADDR + offset;
    read_data_register((Registers)reg, data, length, ec);
  }

  /**
   * @brief Enable fast transfer mode (using up to 255 bytes at a time)
   *        between RF and I2C. After calling this, you can call transfer(),
   *        receive(), and get_ftm_length() for fast bi-directional
   *        communications between RF and I2C.
   * @note You must call stop_fast_transfer_mode() before calling any other
   *       functions on this class.
   * @param &ec Error code to be filled with any errors that occur during
   *            writing.
   */
  void start_fast_transfer_mode(std::error_code &ec) {
    uint16_t reg_addr = (uint16_t)Registers::MB_CTRL;
    uint8_t data[3] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        // data
        MB_CTRL::EN,
    };
    bool success = base_config_.write(DATA_ADDRESS, data, sizeof(data));
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    } else {
      ec.clear();
    }
  }

  /**
   * @brief Disable fast transfer mode (using up to 255 bytes at a time)
   *        between RF and I2C. After calling this, you cannot call transfer()
   *        or receive() without again calling start_fast_transfer_mode()
   *        first.
   * @param &ec Error code to be filled with any errors that occur during
   *            writing.
   */
  void stop_fast_transfer_mode(std::error_code &ec) {
    uint16_t reg_addr = (uint16_t)Registers::MB_CTRL;
    uint8_t data[3] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        // data
        0,
    };
    bool success = base_config_.write(DATA_ADDRESS, data, sizeof(data));
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    } else {
      ec.clear();
    }
  }

  /**
   * @brief Returns the available message length in the FTM message box.
   * @details Will return non-zero if the RF received data into the FTM.
   * @param &ec Error code to be filled with any errors that occur during
   *            reading.
   * @return Number of bytes (up to 255) available in the FTM message box.
   */
  uint8_t get_ftm_length(std::error_code &ec) {
    uint8_t len = 0;
    read_data_register(Registers::MB_LEN, &len, 1, ec);
    if (ec) {
      return 0;
    }
    return len;
  }

  /**
   * @brief Write data to the FTM message box to send.
   * @note Must call start_fast_transfer_mode() prior to use.
   * @param data Data to be written.
   * @param length Number of bytes to write.
   * @param &ec Error code to be filled with any errors that occur during
   */
  void transfer(const uint8_t *data, uint8_t length, std::error_code &ec) {
    write_ftm(data, length, ec);
  }

  /**
   * @brief Read data from the FTM message box.
   * @note Must call start_fast_transfer_mode() prior to use.
   * @note The length available to be read can be found by calling
   *       get_ftm_length().
   * @param data Pointer to memory to be filled with data.
   * @param length Number of bytes to read.
   * @param &ec Error code to be filled with any errors that occur during
   */
  void receive(uint8_t *data, uint8_t length, std::error_code &ec) { read_ftm(data, length, ec); }

protected:
  void init(std::error_code &ec) {
    logger_.info("Initializing");
    [[maybe_unused]] auto uuid = read_uuid(ec);
    if (ec) {
      logger_.error("Failed to read uuid");
      return;
    }
    [[maybe_unused]] auto password = read_password(ec);
    if (ec) {
      logger_.error("Failed to read password");
      return;
    }
    auto block_size_bytes = read_block_size_bytes(ec);
    if (ec) {
      logger_.error("Failed to read block size");
      return;
    }
    auto memory_size_blocks = read_memory_size_blocks(ec);
    if (ec) {
      logger_.error("Failed to read memory size");
      return;
    }
    memory_size_bytes_ = block_size_bytes * memory_size_blocks;
    logger_.info("Memory size (B): {}", memory_size_bytes_);
  }

  void write_ftm(const uint8_t *data, uint8_t length, std::error_code &ec) {
    // must start from FTM_START_ADDR
    uint8_t all_data[2 + length];
    all_data[0] = (uint8_t)(FTM_START_ADDR >> 8);
    all_data[1] = (uint8_t)(FTM_START_ADDR & 0xFF);
    memcpy(&all_data[2], data, length);
    bool success = base_config_.write(DATA_ADDRESS, all_data, length + 2);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  void read_ftm(uint8_t *data, uint8_t length, std::error_code &ec) {
    read_ftm(data, length, 0, ec);
  }

  void read_ftm(uint8_t *data, uint8_t length, uint8_t offset, std::error_code &ec) {
    // read can start from any byte offset within the FTM mailbox.
    uint16_t reg = FTM_START_ADDR + offset;
    read_data_register((Registers)reg, data, length, ec);
  }

  uint64_t read_uuid(std::error_code &ec) {
    uint8_t uuid[8];
    read_syst_register(Registers::UID, uuid, sizeof(uuid), ec);
    if (ec) {
      return 0;
    }
    memcpy(&uuid_, uuid, sizeof(uuid));
    logger_.debug("Got uuid: 0x{:016X}", uuid_);
    return uuid_;
  }

  uint8_t read_block_size_bytes(std::error_code &ec) {
    uint8_t block_size_bytes = 0;
    read_syst_register(Registers::MEM_SIZE, &block_size_bytes, 1, ec);
    if (ec) {
      return 0;
    }
    logger_.debug("Block size (B): {}", block_size_bytes);
    return block_size_bytes;
  }

  uint16_t read_memory_size_blocks(std::error_code &ec) {
    uint8_t buffer[2];
    read_syst_register(Registers::BLK_SIZE, buffer, 2, ec);
    if (ec) {
      return 0;
    }
    uint16_t memory_size_blocks = 0;
    memory_size_blocks = (buffer[0] << 8) | buffer[1];
    logger_.debug("Memory size (blocks): {}", memory_size_blocks);
    return memory_size_blocks;
  }

  uint64_t read_password(std::error_code &ec) {
    uint8_t pswds[8];
    read_syst_register(Registers::I2C_PWD, pswds, sizeof(pswds), ec);
    if (ec) {
      return 0;
    }
    memcpy(&password_, pswds, sizeof(pswds));
    logger_.debug("Got pswd: 0x{:016X}", password_);
    return password_;
  }

  void present_password(std::error_code &ec) {
    // length of messsage is 17 bytes, plus 2 for address
    uint8_t data[2 + 17] = {0};
    data[0] = (uint16_t)Registers::I2C_PWD >> 8;
    data[1] = (uint16_t)Registers::I2C_PWD & 0xFF;
    const uint8_t *pswd_data = (uint8_t *)&password_;
    // validation code in the middle
    data[8 + 2] = 0x09;
    for (int i = 0; i < 4; i++) {
      data[2 + i] = pswd_data[i];
      data[2 + i + 4] = pswd_data[i + 4];
      data[2 + i + 9] = data[2 + i];
      data[2 + i + 13] = data[2 + i + 4];
    }
    logger_.debug("Presenting password: {}\n", data);
    bool success = base_config_.write(SYST_ADDRESS, data, sizeof(data));
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  /**
   * @brief Register map for the ST25DV.
   */
  enum class Registers : uint16_t {
    // System configuration registers, must be accessed with device select
    // E2=1, and a security session must be opened first by presenting a valid
    // I2C password.
    GPO_CONF = 0x0000,           /**< Enable / Disable interrupts on GPO. */
    INT_PULSE_DURATION = 0x0001, /**< Duration of the interrupt pulse. */
    ENDA1 = 0x0005,              /**< End address of Area 1. */
    ENDA2 = 0x0007,              /**< End address of Area 2. */
    ENDA3 = 0x0009,              /**< End address of Area 3. */
    MEM_SIZE = 0x0014,           /**< Memory size value in blocks, 2 bytes. */
    BLK_SIZE = 0x0016,           /**< Block size value in bytes. */
    UID = 0x0018,                /**< Unique identifier, 8 bytes. */
    IC_REV = 0x0020,             /**< IC revision, 1 byte. */
    I2C_PWD = 0x0900,            /**< I2C Security session password, 8 bytes. */
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
      length = len;
    }

    /**
     * @brief Get the number of bytes that the TLV will occupy, based on the
     *        length.
     * @return Number of bytes of raw that should be written to represent the
     *         TLV.
     */
    int size() const { return length < 255 ? 2 : 4; }

    /**
     * @brief Append the TLV into a vector of bytes.
     * @param data The vector to append the TLV to.
     */
    void append(std::vector<uint8_t> &data) {
      data.push_back((uint8_t)type);
      if (length < 255) {
        data.push_back((uint8_t)length);
      } else {
        data.push_back(0xFF);
        data.push_back(length >> 8);
        data.push_back(length & 0xFF);
      }
    }

    /**
     * @brief Serialize the TLV into a vector of bytes.
     * @param data The vector to serialize the TLV into.
     * @param offset The offset into the vector to start writing at.
     */
    void serialize(std::vector<uint8_t> &data, int offset) const {
      data[offset] = (uint8_t)type;
      if (length < 255) {
        data[offset + 1] = length;
      } else {
        data[offset + 1] = 0xFF;
        data[offset + 2] = length >> 8;
        data[offset + 3] = length & 0xFF;
      }
    }

    Type5TagType type; ///< Message type
    size_t length;     ///< Message length
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

  void read_syst_register(Registers reg, uint8_t *data, uint8_t length, std::error_code &ec) {
    uint8_t reg_addr[2];
    reg_addr[0] = (uint16_t)reg >> 8;
    reg_addr[1] = (uint16_t)reg & 0xFF;
    bool success = base_config_.write(SYST_ADDRESS, reg_addr, 2);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
      return;
    }
    success = base_config_.read(SYST_ADDRESS, data, length);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    } else {
      ec.clear();
    }
  }

  void read_data_register(Registers reg, uint8_t *data, uint8_t length, std::error_code &ec) {
    uint8_t reg_addr[2];
    reg_addr[0] = (uint16_t)reg >> 8;
    reg_addr[1] = (uint16_t)reg & 0xFF;
    bool success = base_config_.write(DATA_ADDRESS, reg_addr, 2);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
      return;
    }
    success = base_config_.read(DATA_ADDRESS, data, length);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    } else {
      ec.clear();
    }
  }

  uint32_t memory_size_bytes_;
  uint64_t uuid_;
  uint64_t password_;
};
} // namespace espp
