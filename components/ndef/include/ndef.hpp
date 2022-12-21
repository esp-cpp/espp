#pragma once

#include <cstring>
#include <vector>
#include <string>
#include <string_view>

namespace espp {
  /**
   * @brief implements serialization & deserialization logic for NFC Data
   *        Exchange Format (NDEF) records which can be stored on and
   *        transmitted from NFC devices.
   *
   * @details NDEF records can be composed the following way:
   *   @code{.unparsed}
   *   Bit 7     6       5       4       3       2       1       0
   *   ------  ------  ------  ------  ------  ------  ------  ------
   *   [ MB ]  [ ME ]  [ CF ]  [ SR ]  [ IL ]  [        TNF         ]
   *   [                         TYPE LENGTH  (may be 0)            ]
   *   [                       PAYLOAD LENGTH (1B or 4B, see SR)    ]
   *   [                          ID LENGTH   (if IL)               ]
   *   [                         RECORD TYPE  (if TYPE LENGTH > 0)  ]
   *   [                              ID      (if IL)               ]
   *   [                           PAYLOAD    (payload length bytes)]
   *  @endcode
   *
   *  The first byte (Flags) has these bits:
   *  * Bits 0-3: TNF - Type Name Format - describes record type (see TNF class)
   *  * Bit 3: IL - ID Length - indicates if the ID Length Field is present or not
   *  * Bit 4: SR - Short Record - set to 1 if the payload length field is 1 byte (8
   *      bits / 0-255) or less, otherwise the payload length is 4 bytes
   *  * Bit 5: CF - Chunk Flag - indicates if this is the first record chunk or a
   *      middle record chunk, set to 0 for the first record of the message and
   *      for subsequent records set to 1.
   *  * Bit 6: ME - Message End - 1 indicates if this is the last record in the
   *      message
   *  * Bit 7: MB - Message Begin - 1 indicates if this is the first record in the
   *      message
   *
   * @note Some information about NDEF can be found:
   *       * https://www.maskaravivek.com/post/understanding-the-format-of-ndef-messages/
   *       * https://ndeflib.readthedocs.io/en/stable/records/bluetooth.html
   *       * https://developer.android.com/reference/android/nfc/NdefMessage
   *       * https://www.oreilly.com/library/view/beginning-nfc/9781449324094/ch04.html
   *
   */
  class Ndef {
  public:
    /**
     * @brief Type Name Format (TNF) field is a 3-bit value that describes the
     *        record type.
     *
     * Some Common TNF::WELL_KNOWN record type strings:
     *   * Text (T)
     *   * URI  (U)
     *   * Smart Poster (Sp)
     *   * Alternative Carrier (ac)
     *   * Handover Carrier (Hc)
     *   * Handover Request (Hr)
     *   * Handover Select (Hs)
     */
    enum class TNF : uint8_t {
      EMPTY          = 0x00, ///< Record is empty
      WELL_KNOWN     = 0x01, ///< Type field contains a well-known RTD type name
      MIME_MEDIA     = 0x02, ///< Type field contains a media type (RFC 2046)
      ABSOLUTE_URI   = 0x03, ///< Type field contains an absolute URI (RFC 3986)
      EXTERNAL_TYPE  = 0x04, ///< Type field Contains an external type name
      UNKNOWN        = 0x05, ///< Payload type is unknown, type length must be 0.
      UNCHANGED      = 0x06, ///< Indicates the payload is an intermediate or final chunk of a chunked NDEF record, type length must be 0.
      RESERVED       = 0x07, ///< Reserved by the NFC forum for future use
    };

    /**
     * URI Identifier Codes (UIC), See Table A-3 at
     * https://www.oreilly.com/library/view/beginning-nfc/9781449324094/apa.html
     * and https://learn.adafruit.com/adafruit-pn532-rfid-nfc/ndef
     */
    enum class Uic {
      NONE        = 0x00, ///< Exactly as written
      HTTP_WWW    = 0x01, ///< http://www.
      HTTPS_WWW   = 0x02, ///< https://www.
      HTTP        = 0x03, ///< http://
      HTTPS       = 0x04, ///< https://
      TEL         = 0x05, ///< tel:
      MAILTO      = 0x06, ///< mailto:
      FTP_ANON    = 0x07, ///< ftp://anonymous:anonymous@
      FTP_FTP     = 0x08, ///< ftp://ftp.
      FTPS        = 0x09, ///< ftps://
      SFTP        = 0x0A, ///< sftp://
      SMB         = 0x0B, ///< smb://
      NFS         = 0x0C, ///< nfs://
      FTP         = 0x0D, ///< ftp://
      DAV         = 0x0E, ///< dav://
      NEWS        = 0x0F, ///< news:
      TELNET      = 0x10, ///< telnet://
      IMAP        = 0x11, ///< imap:
      RSTP        = 0x12, ///< rtsp://
      URN         = 0x13, ///< urn:
      POP         = 0x14, ///< pop:
      SIP         = 0x15, ///< sip:
      SIPS        = 0x16, ///< sips:
      TFTP        = 0x17, ///< tftp:
      BTSPP       = 0x18, ///< btspp://
      BTL2CAP     = 0x19, ///< btl2cap://
      BTGOEP      = 0x1A, ///< btgoep://
      TCPOBEX     = 0x1B, ///< tcpobex://
      IRDAOBEX    = 0x1C, ///< irdaobex://
      FILE        = 0x1D, ///< file://
      URN_EPC_ID  = 0x1E, ///< urn:epc:id:
      URN_EPC_TAG = 0x1F, ///< urn:epc:tag:
      URN_EPC_PAT = 0x20, ///< urn:epc:pat:
      URN_EPC_RAW = 0x21, ///< urn:epc:raw:
      URN_EPC     = 0x22, ///< urn:epc:
      URN_NFC     = 0x23, ///< urn:nfc:
    };

    /**
     * @brief Handover codes for data types in BT and BLE out of band (OOB)
     *        pairing NDEF records.
     */
    enum class BtHandover {
      UUIDS_16_BIT_PARTIAL   = 0x02, ///< Incomplete list of 16 bit service class UUIDs
      UUIDS_16_BIT_COMPLETE  = 0x03, ///< Complete list of 16 bit service class UUIDs
      UUIDS_32_BIT_PARTIAL   = 0x04, ///< Incomplete list of 32 bit service class UUIDs
      UUIDS_32_BIT_COMPLETE  = 0x05, ///< Complete list of 32 bit service class UUIDs
      UUIDS_128_BIT_PARTIAL  = 0x06, ///< Incomplete list of 128 bit service class UUIDs
      UUIDS_128_BIT_COMPLETE = 0x07, ///< Complete list of 128 bit service class UUIDs
      SHORT_LOCAL_NAME       = 0x08, ///< Shortened Bluetooth Local Name
      LONG_LOCAL_NAME        = 0x09, ///< Complete Bluetooth Local Name
      CLASS_OF_DEVICE        = 0x0D, ///< Class of Device
      SP_HASH_C192           = 0x0E, ///< Simple Pairing Hash C-192
      SP_RANDOM_R192         = 0x0F, ///< Simple Pairing Randomizer R-192
      SECURITY_MANAGER_TK    = 0x10, ///< Security Manager TK Value (LE Legacy Pairing)
      APPEARANCE             = 0x19, ///< Appearance
      MAC                    = 0x1B, ///< Bluetooth Device Address
      LE_ROLE                = 0x1C, ///< LE Role
      SP_HASH_C256           = 0x1D, ///< Simple Pairing Hash C-256
      SP_HASH_R256           = 0x1E, ///< Simple Pairing Randomizer R-256
      LE_SC_CONFIRMATION     = 0x22, ///< LE Secure Connections Confirmation Value
      LE_SC_RANDOM           = 0x23, ///< LE Secure Connections Random Value
    };

    /**
     * @brief Possible roles for BLE records to indicate support for.
     */
    enum class BleRole : uint8_t {
      LE_ROLE_PERIPHERAL_ONLY      = 0x00, ///< Radio can only act as a peripheral
      LE_ROLE_CENTRAL_ONLY         = 0x01, ///< Radio can only act as a central
      LE_ROLE_PERIPHERAL_CENTRAL   = 0x02, ///< Radio can act as both a peripheral and a central
    };

    /**
     * @brief Makes an NDEF record with header and payload.
     * @param tnf The TNF for this packet.
     * @param type String view for the type of this packet
     * @param payload The payload data for the packet
     */
    Ndef(TNF tnf, std::string_view type, std::string_view payload)
      : tnf_(tnf),
        type_(type),
        payload_(payload) {
    }

    /**
     * @brief Static function to make an NDEF record for transmitting english
     *        text.
     * @param text The text that the NDEF record will hold.
     * @return NDEF record object.
     */
    static Ndef make_text(std::string_view text) {
      // TODO: only supports english (en) text right now...
      // need to preprend start of text / language
      static const std::string start{0x02, 'e', 'n'};
      std::string full = start + std::string(text);
      return Ndef(TNF::WELL_KNOWN, "T", full);
    }

    /**
     * @brief Static function to make an NDEF record for loading a URI.
     * @param uri URI for the record to point to.
     * @param uic UIC for the uri - helps shorten the uri text / NDEF record.
     * @return NDEF record object.
     */
    static Ndef make_uri(std::string_view uri, Uic uic = Uic::NONE) {
      // prepend URI with identifier code
      std::vector<uint8_t> full;
      full.resize(1 + uri.size());
      full[0] = (uint8_t)uic;
      memcpy(&full[1], uri.data(), uri.size());
      return Ndef(TNF::WELL_KNOWN, "U", std::string_view{(const char*)full.data(), full.size()});
    }

    /**
     * @brief Static function to make an NDEF record for launching an Android App.
     * @param uri URI for the android package / app to launch.
     * @return NDEF record object.
     */
    static Ndef make_android_launcher(std::string_view uri) {
      return Ndef(TNF::EXTERNAL_TYPE, "android.com:pkg", uri);
    }

    /**
     * @brief Static function to make an NDEF record for BT classic OOB Pairing (Android).
     * @param mac_addr 48 bit MAC Address of the BT radio
     * @param device_class The bluetooth device class for this radio.
     * @param name Name of the BT device.
     * @return NDEF record object.
     */
    static Ndef make_oob_pairing(uint64_t mac_addr, uint32_t device_class, std::string_view name) {
      std::vector<uint8_t> data;
      // NOTE: for the extended inquiry response (EIR) data types see the
      // BT_HANDOVER_TYPE_ codes here:
      // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

      // fill the data
      // 2 bytes of data length (e.g. 0x20, 0x00) including length field
      data.resize(2); // 2 bytes that we'll fill in later with the length
      // 6 bytes of BT device MAC address
      data.push_back(mac_addr & 0x0000FF0000000000 >> 40);
      data.push_back(mac_addr & 0x000000FF00000000 >> 32);
      data.push_back(mac_addr & 0x00000000FF000000 >> 24);
      data.push_back(mac_addr & 0x0000000000FF0000 >> 16);
      data.push_back(mac_addr & 0x000000000000FF00 >> 8);
      data.push_back(mac_addr & 0x00000000000000FF >> 0);

      // TODO: add optional data
      // OOB optional data as extended inquiry response (EIR) - no specific
      //   order. Example below:
      //   * local name field:
      //    * 1 byte length
      //    * 1 byte EIR data type (local name = 0x09)
      //    * local name (length - 1 bytes)
      //   * class of device field:
      //    * 1 byte length
      //    * 1 byte EIR data type (class of device = 0x0D, 24 bit class of device value)
      //    * class of device (length - 1 bytes) (service class, major class, minor class)
      //   * uuid field:
      //    * 1 byte length
      //    * 1 byte EIR data type (UUID = 0x03, 16bit service class uuids)
      //    * groups of 2 byte fields (each is two bytes, total bytes is length - 1)

      // now make sure the length is updated
      int length = data.size();
      data[0] = length & 0xFF00 >> 8;
      data[1] = length & 0x00FF >> 0;
      auto sv_data = std::string_view{(const char*)data.data(), data.size()};
      return Ndef(TNF::MIME_MEDIA, "application/vnd.bluetooth.ep.oob", sv_data);
    }

    /**
     * @brief Static function to make an NDEF record for BLE OOB Pairing (Android).
     * @param mac_addr 48 bit MAC Address of the BLE radio
     * @param role The BLE role of the device (central / peripheral / dual)
     * @param name Name of the BLE device.
     * @return NDEF record object.
     */
    static Ndef make_le_oob_pairing(uint64_t mac_addr, BleRole role, std::string_view name) {
      std::vector<uint8_t> data;
      // NOTE: for the extended inquiry response (EIR) data types see the
      // BT_HANDOVER_TYPE_ codes here:
      // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

      // fill the data - NOTE: all BLE data is passed as EIR data with no additional data

      // (mandatory 0x1B) LE device address
      // NOTE: we use an additional byte here because there's an extra byte
      // required after the MAC as part of this EIR payload...
      uint8_t mac_addr_bytes[9];
      memset(mac_addr_bytes, 0, sizeof(mac_addr_bytes));
      // get all 8 bytes of the 64bit mac addr passed in
      memcpy(mac_addr_bytes, &mac_addr, 8);
      make_bt_eir(data, BtHandover::MAC, std::string_view{(const char*)&mac_addr_bytes[2], 7});

      // (mandatory 0x1C) LE role
      make_bt_eir(data, BtHandover::LE_ROLE, std::string_view{(const char*)&role, 1});

      // TODO: provide optional parameters
      // (optional  0x10) Security Manager TK value (LE legacy pairing)
      // (optional  0x19) 2 byte Appearance, e.g. 'Remote Control', 'Computer', or 'Heart Rate Sensor'
      // (optional  0x01) Flags
      // (optional  0x08 or 0x09) Shortened or complete bluetooth local name, e.g. 'My BLE Device'
      // (optional  0x22) LE secure connections confirmation value
      // (optional  0x23) LE secure connections random value
      auto sv_data = std::string_view{(const char*)data.data(), data.size()};
      return Ndef(TNF::MIME_MEDIA, "application/vnd.bluetooth.le.oob", sv_data);
    }

    /**
     * @brief Serialize the NDEF record into a sequence of bytes.
     * @return The vector<uint8_t> of bytes representing the NDEF record.
     */
    std::vector<uint8_t> serialize() {
      std::vector<uint8_t> data;
      int size = get_size();
      set_flags();
      data.resize(size);
      data[0] = flags_.raw;
      data[1] = type_.size();
      if (flags_.SR) {
        data[2] = (uint8_t)payload_.size();
        memcpy(&data[3], type_.data(), type_.size());
        memcpy(&data[3 + type_.size()], payload_.data(), payload_.size());
      } else {
        uint32_t _num_bytes = payload_.size();
        memcpy(&data[2], &_num_bytes, 4);
        memcpy(&data[6], type_.data(), type_.size());
        memcpy(&data[6 + type_.size()], payload_.data(), payload_.size());
      }
      return data;
    }

    /**
     * @brief Get the number of bytes needed for the NDEF record.
     * @return Size of the NDEF record (bytes), for serialization.
     */
    int get_size() const {
      int num_payload_bytes = payload_.size();
      uint8_t num_payload_length_bytes = 1;
      if (num_payload_bytes > 255) {
        num_payload_length_bytes = 4;
      }
      uint8_t type_length = type_.size();
      int total_length =
        sizeof(flags_.raw) +
        sizeof(type_length) +
        num_payload_length_bytes +
        type_length +
        num_payload_bytes;
      return total_length;
    }

  protected:
    struct Flags {
      union {
        struct {
          uint8_t TNF : 3;
          uint8_t IL : 1;
          uint8_t SR : 1;
          uint8_t CF : 1;
          uint8_t ME : 1;
          uint8_t MB : 1;
        };
        uint8_t raw;
      };
    };

    void set_flags() {
      flags_.TNF = (uint8_t)tnf_;
      flags_.IL = 0; // no id length / field
      // short record?
      flags_.SR = payload_.size() < 256 ? 1 : 0;
      flags_.CF = 0; // first record of a chunk
      flags_.ME = 1; // message end
      flags_.MB = 1; // message begin
    }

    static int make_bt_eir(std::vector<uint8_t>& data, BtHandover eir_type, std::string_view payload) {
      uint8_t num_eir_bytes = 0;
      switch (eir_type) {
      case BtHandover::UUIDS_16_BIT_PARTIAL:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::UUIDS_16_BIT_COMPLETE:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::UUIDS_32_BIT_PARTIAL:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::UUIDS_32_BIT_COMPLETE:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::UUIDS_128_BIT_PARTIAL:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::UUIDS_128_BIT_COMPLETE:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::SHORT_LOCAL_NAME:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::LONG_LOCAL_NAME:
        num_eir_bytes = payload.size();
        break;
      case BtHandover::CLASS_OF_DEVICE:
        num_eir_bytes = 3; // 24 bit class of device structure
        break;
      case BtHandover::SECURITY_MANAGER_TK:
        break;
      case BtHandover::APPEARANCE:
        num_eir_bytes = 2; // 2 byte appearance type
        break;
      case BtHandover::MAC:
        num_eir_bytes = 7; // 6 for mac addr + 1 for mac addr type
        break;
      case BtHandover::LE_ROLE:
        num_eir_bytes = 1;
        break;
      case BtHandover::LE_SC_CONFIRMATION:
        break;
      case BtHandover::LE_SC_RANDOM:
        break;
      default:
        break;
      }
      data.push_back(num_eir_bytes);
      data.push_back((uint8_t)eir_type);
      for (int i=0; i<num_eir_bytes; i++) {
        data.push_back(payload[i]);
      }
      return 2 + num_eir_bytes;
    }

    TNF tnf_;
    Flags flags_;
    std::string type_{""};
    std::string payload_{""};
  };
}
