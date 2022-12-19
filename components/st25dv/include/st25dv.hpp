#pragma once

#include <atomic>
#include <functional>
#include <cmath>

#include "logger.hpp"
#include "task.hpp"

namespace espp {
  /**
   * @brief Class for wireless communications using a ST25DV Dynamic NFC/RFID
   *        tag. The datasheet for the ST25DV can be found here:
   *        https://www.st.com/resource/en/datasheet/st25dv04k.pdf
   *
   * \section st25dv_ex1 St25dv Example
   * \snippet st25dv_example.cpp st25dv example
   */
  class St25dv {
  public:
    // NOTE: when the datasheet mentions E2 device select, they are talking
    // about Bit 4 of the address which selects between the data (user memory,
    // dynamic registers, FTM mailbox), and system memory
    static constexpr uint8_t DATA_ADDRESS = (0b10100110 >> 1); ///< I2C Address for writing / reading data
    static constexpr uint8_t SYST_ADDRESS = (0x10101110 >> 1); ///< I2C Address for writing / reading system config

    /**
     * @brief Function to write bytes to St25dv.
     * @param addr I2C address to write to
     * @param data Data to be written.
     * @param length Number of bytes to write.
     */
    typedef std::function<void(uint8_t addr, uint8_t* data, uint8_t length)> write_fn;

    /**
     * @brief Function to read a sequence of bytes from St25dv.
     * @param addr I2C address to read from
     * @param reg_addr Start register address to read from.
     * @param data Pointer to memory which will be filled with data read
     *        from St25dv.
     * @param length Number of bytes to read
     */
    typedef std::function<void(uint8_t addr, uint16_t reg_addr, uint8_t *data, uint8_t length)> read_fn;

    /**
     * @brief Configuration information for the St25dv.
     */
    struct Config {
      write_fn write; ///< Function to write to the device.
      read_fn read; ///< Function to read from the device.
      Logger::Verbosity log_level{Logger::Verbosity::WARN};
    };

    /**
     * @brief Construct the St25dv and start the update task.
     */
    St25dv(const Config& config)
      : write_(config.write),
        read_(config.read),
        logger_({.tag = "St25dv", .level = config.log_level}) {
      init();
    }

    uint8_t get_interrupt_status() {
      uint8_t it_sts = 0;
      read_(SYST_ADDRESS, (uint16_t)Registers::IT_STS, &it_sts, 1);
      return it_sts;
    }

    void write_uri(std::string_view protocol, std::string_view uri, std::string_view info) {
      //
    }

    std::string read_uri() {
      std::string uri;
      return uri;
    }

    void start_fast_transfer_mode() {
      uint16_t reg_addr = (uint16_t)Registers::MB_CTRL;
      uint8_t data[3] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        // data
        MB_CTRL::EN,
      };
      write_(SYST_ADDRESS, data, sizeof(data));
    }

    void stop_fast_transfer_mode() {
      uint16_t reg_addr = (uint16_t)Registers::MB_CTRL;
      uint8_t data[3] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        // data
        0,
      };
      write_(SYST_ADDRESS, data, sizeof(data));
    }

    uint8_t get_ftm_length() {
      uint8_t len = 0;
      read_(SYST_ADDRESS, (uint16_t)Registers::MB_LEN, &len, 1);
      return len;
    }

    /**
     * @brief Write data to the FTM message box to send.
     * @param data Data to be written.
     * @param length Number of bytes to write.
     */
    void transfer(uint8_t* data, uint8_t length) {
      write_ftm(data, length);
    }

    /**
     * @brief Read data from the FTM message box.
     * @param data Pointer to memory to be filled with data.
     * @param length Number of bytes to read.
     */
    void receive(uint8_t* data, uint8_t length) {
      read_ftm(data, length);
    }

  protected:
    void init() {
      logger_.info("Initializing");
      read_uuid();
      read_password();
    }

    void write_ftm(uint8_t* data, uint8_t length) {
      // must start from FTM_START_ADDR
      uint8_t all_data[2+length];
      all_data[0] = (uint8_t)(FTM_START_ADDR >> 8);
      all_data[1] = (uint8_t)(FTM_START_ADDR & 0xFF);
      memcpy(&all_data[2], data, length);
      write_(DATA_ADDRESS, data, length);
    }

    void read_ftm(uint8_t* data, uint8_t length, uint8_t offset=0) {
      // read can start from any byte offset within the FTM mailbox.
      read_(DATA_ADDRESS, FTM_START_ADDR + offset, data, length);
    }

    void read_uuid() {
      uint8_t uuid[8];
      read_(SYST_ADDRESS, (uint16_t)Registers::UID, uuid, sizeof(uuid));
      memcpy(&uuid_, uuid, sizeof(uuid));
      logger_.debug("Got uuid: 0x{:016X}", uuid_);
    }

    void read_password() {
      uint8_t pswd[8];
      read_(SYST_ADDRESS, (uint16_t)Registers::I2C_PWD, pswd, sizeof(pswd));
      memcpy(&password_, pswd, sizeof(pswd));
      logger_.debug("Got pswd: 0x{:016X}", password_);
    }

    void present_password() {
      // length of messsage is 17 bytes, plus 2 for address
      uint8_t data[2+17] = {0};
      data[0] = (uint16_t)Registers::I2C_PWD >> 8;
      data[1] = (uint16_t)Registers::I2C_PWD & 0xFF;
      uint8_t *pswd_data = (uint8_t*)&password_;
      // validation code in the middle
      data[8+2] = 0x09;
      for (int i=0; i<4; i++) {
        data[2+i] = pswd_data[i];
        data[2+i + 4] = pswd_data[i+4];
        data[2+i + 9] = data[2+i];
        data[2+i + 13] = data[2+i+4];
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
      GPO_CONF = 0x0000,  /**< Enable / Disable interrupts on GPO. */
      UID      = 0x0018,  /**< Unique identifier, 8 bytes. */
      I2C_PWD  = 0x0900,  /**< I2C Security session password, 8 bytes. */
      // Dynamic registers:
      GPO_CTRL = 0x2000,  /**< GPO Control register. */
      EH_CTRL  = 0x2002,  /**< Energy Harvesting management and usage status register. */
      RF_MNGT  = 0x2003,  /**< RF Interface usage management. */
      I2C_SSO  = 0x2004,  /**< I2C Security session status register. */
      IT_STS   = 0x2005,  /**< Interruption status register. */
      MB_CTRL  = 0x2006,  /**< Fast transfer mode control & status register. */
      MB_LEN   = 0x2007,  /**< Length of fast transfer mode message. */
    };

    class GPO {
    public:
      static constexpr int RF_USER_EN       = 0b00000001;
      static constexpr int RF_ACTIVITY_EN   = 0b00000010;
      static constexpr int RF_INTTERUPT_EN  = 0b00000100;
      static constexpr int FIELD_CHANGE_EN  = 0b00001000;
      static constexpr int RF_PUT_MSG_EN    = 0b00010000;
      static constexpr int RF_GET_MSG_EN    = 0b00100000;
      static constexpr int RF_WRITE_EN      = 0b01000000;
      static constexpr int GPO_EN           = 0b10000000;
    };

    class EH_CTRL {
    public:
      static constexpr int EH_EN       = 0b00000001;
      static constexpr int EH_ON       = 0b00000010;
      static constexpr int FIELD_ON    = 0b00000100;
      static constexpr int VCC_ON      = 0b00001000;
    };

    class IT_STS {
    public:
      static constexpr int RF_USER       = 0b00000001;
      static constexpr int RF_ACTIVITY   = 0b00000010;
      static constexpr int RF_INTTERUPT  = 0b00000100;
      static constexpr int FIELD_FALLING = 0b00001000;
      static constexpr int FIELD_RISING  = 0b00010000;
      static constexpr int RF_PUT_MSG    = 0b00100000;
      static constexpr int RF_GET_MSG    = 0b01000000;
      static constexpr int RF_WRITE      = 0b10000000;
    };

    class MB_CTRL {
    public:
      static constexpr int EN               = 0b00000001;
      static constexpr int HOST_PUT_MSG     = 0b00000010;
      static constexpr int RF_PUT_MSG       = 0b00000100;
      static constexpr int HOST_MISS_MSG    = 0b00010000;
      static constexpr int RF_MISS_MSG      = 0b00100000;
      static constexpr int HOST_CURRENT_MSG = 0b01000000;
      static constexpr int RF_CURRENT_MSG   = 0b10000000;
    };

    static constexpr uint16_t FTM_START_ADDR = 0x2008; /**< Start address of the Fast Transfer Mode Mailbox. */
    static constexpr int FTM_SIZE = 0xFF; /**< Number of bytes in the Fast Transfer Mode Mailbox. */

    write_fn write_;
    read_fn read_;
    uint64_t uuid_;
    uint64_t password_;
    Logger logger_;
  };
}
