#pragma once

#include <cstring>
#include <vector>
#include <string_view>

namespace espp {
  /**
   * @brief implements serialization & deserialization logic for NRF Data
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
     * Common record types:
     * * Text (T)
     * * URI  (U)
     * * Smart Poster (Sp)
     * * Alternative Carrier (ac)
     * * Handover Carrier (Hc)
     * * Handover Request (Hr)
     * * Handover Select (Hs)
     */

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

    static std::vector<uint8_t> make_text(std::string_view text) {
      // TODO: only supports english (en) text right now...
      // need to preprend start of text / language
      static const std::string start{0x02, 'e', 'n'};
      std::string full = start + std::string(text);
      return make(TNF::WELL_KNOWN, "T", full);
    }

    static std::vector<uint8_t> make_uri(std::string_view uri, Uic uic = Uic::NONE) {
      // prepend URI with identifier code
      std::vector<uint8_t> full;
      full.resize(1 + uri.size());
      full[0] = (uint8_t)uic;
      memcpy(&full[1], uri.data(), uri.size());
      return make(TNF::WELL_KNOWN, "U", std::string_view{(const char*)full.data(), full.size()});
    }

    static std::vector<uint8_t> make_android_launcher(std::string_view uri) {
      return make(TNF::EXTERNAL_TYPE, "android.com:pkg", uri);
    }

    static std::vector<uint8_t> make_oob_pairing(uint64_t mac_addr, uint32_t device_class, std::string_view name) {
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
      return make(TNF::MIME_MEDIA, "application/vnd.bluetooth.ep.oob", sv_data);
    }

    static std::vector<uint8_t> make_le_oob_pairing(uint64_t mac_addr, uint32_t device_class, std::string_view name) {
      std::vector<uint8_t> data;
      // NOTE: for the EIR data types see the BT_HANDOVER_TYPE_ codes here:
      // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

      // TODO: fill the data - NOTE: all BLE data is passed as EIR data with no additional data
      // (mandatory 0x1B) LE device address
      // (mandatory 0x1C) LE role
      // (optional  0x10) Security Manager TK value (LE legacy pairing)
      // (optional  0x19) 2 byte Appearance, e.g. 'Remote Control', 'Computer', or 'Heart Rate Sensor'
      // (optional  0x01) Flags
      // (optional  0x08 or 0x09) Shortend or complete bluetooth local name, e.g. 'My BLE Device'
      // (optional  0x22) LE secure connections confirmation value
      // (optional  0x23) LE secure connections random value
      auto sv_data = std::string_view{(const char*)data.data(), data.size()};
      return make(TNF::MIME_MEDIA, "application/vnd.bluetooth.le.oob", sv_data);
    }

    /**
     * @brief Makes an NDEF packet with header and payload.
     * @note This does not include ID functionality for the packet.
     * @param tnf The TNF for this packet.
     * @param type String view for the type of this packet
     * @param payload The payload data for the packet
     * @return Vector of bytes which contains the header and payload.
     */
    static std::vector<uint8_t> make(TNF tnf, std::string_view type, std::string_view payload) {
      std::vector<uint8_t> data;
      int num_payload_bytes = payload.size();
      int num_payload_length_bytes = 1;
      if (num_payload_bytes > 255) {
        num_payload_length_bytes = 4;
      }
      uint8_t type_length = type.size();
      Flags flags;
      flags.TNF = (uint8_t)tnf;
      flags.IL = 0; // no id length / field
      flags.SR = num_payload_length_bytes == 1 ? 1 : 0;
      flags.CF = 0; // first record of a chunk
      flags.ME = 1; // message end
      flags.MB = 1; // message begin
      int total_length = sizeof(flags.raw) + sizeof(type_length) + num_payload_length_bytes + type_length + num_payload_bytes;
      data.resize(total_length);
      data[0] = flags.raw;
      data[1] = type_length;
      if (flags.SR) {
        data[2] = (uint8_t)num_payload_bytes;
        memcpy(&data[3], type.data(), type_length);
        memcpy(&data[3 + type_length], payload.data(), payload.size());
      } else {
        uint32_t _num_bytes = num_payload_bytes;
        memcpy(&data[2], &_num_bytes, 4);
        memcpy(&data[6], type.data(), type_length);
        memcpy(&data[6 + type_length], payload.data(), payload.size());
      }
      return data;
    }
  };
}
