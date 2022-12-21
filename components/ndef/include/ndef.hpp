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
   * @note Some information about NDEF can be found
   *       https://www.maskaravivek.com/post/understanding-the-format-of-ndef-messages/
   *
   * @note Some other information about NDEF for BT OOB pairing can be found
   *       https://ndeflib.readthedocs.io/en/stable/records/bluetooth.html
   *
   * @note Some additional information can be found
   *       https://developer.android.com/reference/android/nfc/NdefMessage
   *
   * @note Some further information can be found
   *       https://www.oreilly.com/library/view/beginning-nfc/9781449324094/ch04.html#:~:text=NDEF%20is%20a%20binary%20format,the%20content%20of%20the%20message.
   *
   * @note TNF01 (Well Known) could have a record type of "T" for text message,
   *       "U" for URI message, "Sp" if the payload is a smart poster.
   *
   * @note There is also a well known type (WKT) for bluetooth handover which is
   *       a MIME (multipurpose internet mail extensions) type.
   *
   * @note Will see TNF04 frequently since Android uses an External type called
   *       an Android Application Record to trigger apps to open.
   *
   * @details NDEF records can be composed the following way:
   *   Bit 7     6       5       4       3       2       1       0
   *   ------  ------  ------  ------  ------  ------  ------  ------
   *   [ MB ]  [ ME ]  [ CF ]  [ SR ]  [ IL ]  [        TNF         ]
   *   [                         TYPE LENGTH  (may be 0)            ]
   *   [                       PAYLOAD LENGTH (1B or 4B, see SR)    ]
   *   [                          ID LENGTH   (if IL)               ]
   *   [                         RECORD TYPE  (if TYPE LENGTH > 0)  ]
   *   [                              ID      (if IL)               ]
   *   [                           PAYLOAD                          ]
   *
   * 0-2. TNF - Type Name Format - describes record type (see TNF class)
   *   3. IL - ID Length - indicates if the ID Length Field is present or not
   *   4. SR - Short Record - set to 1 if the payload length field is 1 byte (8
   *      bits / 0-255) or less, otherwise the payload length is 4 bytes
   *   5. CF - Chunk Flag - indicates if this is the first record chunk or a
   *      middle record chunk, set to 0 for the first record of the message and
   *      for subsequent records set to 1.
   *   6. ME - Message End - 1 indicates if this is the last record in the
   *      message
   *   7. MB - Message Begin - 1 indicates if this is the first record in the
   *      message
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
     */
    enum class Uic {
      NONE        = 0x00, ///< Exactly as written
      HTTP_WWW    = 0x01, ///< http://www.
      HTTPS_WWW   = 0x02, ///< https://www.
      HTTP        = 0x03, ///< http://
      HTTPS       = 0x04, ///< https://
      TEL         = 0x05, ///< tel:
      MAILTO      = 0x06, ///< mailto:
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
      // NOTE: for the EIR data types see the BT_HANDOVER_TYPE_ codes here:
      // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

      // TODO: fill the data
      // 2 bytes of data length (e.g. 0x20, 0x00) including length field
      // 6 bytes of BT device MAC address
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
      //
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
    static Ndef make_le_oob_pairing(uint64_t mac_addr, uint32_t role, std::string_view name) {
      std::vector<uint8_t> data;
      // NOTE: for the EIR data types see the BT_HANDOVER_TYPE_ codes here:
      // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

      // TODO: fill the data - NOTE: all BLE data is passed as EIR data with no additional data
      // (mandatory 0x1B) LE device address
      // (mandatory 0x1C) LE role
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

    TNF tnf_;
    Flags flags_;
    std::string type_{""};
    std::string payload_{""};
  };
}
