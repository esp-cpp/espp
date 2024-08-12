#pragma once

#include <algorithm>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#if defined(ESP_PLATFORM)
#include <esp_random.h>
#else
#include <random>
#endif

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
 *       * https://learn.adafruit.com/adafruit-pn532-rfid-nfc/ndef
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
    EMPTY = 0x00,         ///< Record is empty
    WELL_KNOWN = 0x01,    ///< Type field contains a well-known RTD type name
    MIME_MEDIA = 0x02,    ///< Type field contains a media type (RFC 2046)
    ABSOLUTE_URI = 0x03,  ///< Type field contains an absolute URI (RFC 3986)
    EXTERNAL_TYPE = 0x04, ///< Type field Contains an external type name
    UNKNOWN = 0x05,       ///< Payload type is unknown, type length must be 0.
    UNCHANGED = 0x06, ///< Indicates the payload is an intermediate or final chunk of a chunked NDEF
                      ///< record, type length must be 0.
    RESERVED = 0x07,  ///< Reserved by the NFC forum for future use
  };

  /**
   * URI Identifier Codes (UIC), See Table A-3 at
   * https://www.oreilly.com/library/view/beginning-nfc/9781449324094/apa.html
   * and https://learn.adafruit.com/adafruit-pn532-rfid-nfc/ndef
   */
  enum class Uic {
    NONE = 0x00,        ///< Exactly as written
    HTTP_WWW = 0x01,    ///< http://www.
    HTTPS_WWW = 0x02,   ///< https://www.
    HTTP = 0x03,        ///< http://
    HTTPS = 0x04,       ///< https://
    TEL = 0x05,         ///< tel:
    MAILTO = 0x06,      ///< mailto:
    FTP_ANON = 0x07,    ///< ftp://anonymous:anonymous@
    FTP_FTP = 0x08,     ///< ftp://ftp.
    FTPS = 0x09,        ///< ftps://
    SFTP = 0x0A,        ///< sftp://
    SMB = 0x0B,         ///< smb://
    NFS = 0x0C,         ///< nfs://
    FTP = 0x0D,         ///< ftp://
    DAV = 0x0E,         ///< dav://
    NEWS = 0x0F,        ///< news:
    TELNET = 0x10,      ///< telnet://
    IMAP = 0x11,        ///< imap:
    RSTP = 0x12,        ///< rtsp://
    URN = 0x13,         ///< urn:
    POP = 0x14,         ///< pop:
    SIP = 0x15,         ///< sip:
    SIPS = 0x16,        ///< sips:
    TFTP = 0x17,        ///< tftp:
    BTSPP = 0x18,       ///< btspp://
    BTL2CAP = 0x19,     ///< btl2cap://
    BTGOEP = 0x1A,      ///< btgoep://
    TCPOBEX = 0x1B,     ///< tcpobex://
    IRDAOBEX = 0x1C,    ///< irdaobex://
    FILE = 0x1D,        ///< file://
    URN_EPC_ID = 0x1E,  ///< urn:epc:id:
    URN_EPC_TAG = 0x1F, ///< urn:epc:tag:
    URN_EPC_PAT = 0x20, ///< urn:epc:pat:
    URN_EPC_RAW = 0x21, ///< urn:epc:raw:
    URN_EPC = 0x22,     ///< urn:epc:
    URN_NFC = 0x23,     ///< urn:nfc:
  };

  /**
   * @brief Type of Bluetooth radios.
   */
  enum class BtType {
    BREDR = 0x00, ///< BT Classic
    BLE = 0x01,   ///< BT Low Energy
  };

  /**
   * @brief Some appearance codes for BLE radios.
   */
  enum class BtAppearance : uint16_t {
    UNKNOWN = 0x0000, ///< Generic Unknown
    // Generic Phone (b15-b6 = 0x001 << 6 = 0x0040)
    PHONE = 0x0040, ///< Generic Phone
    // Generic Computer (b15-b6 = 0x002 << 6 = 0x0080)
    COMPUTER = 0x0080, ///< Generic Computer
    // Generic Watch (b15-b6 = 0x003 << 6 = 0x00C0)
    WATCH = 0x00C0, ///< Generic Watch
    // Generic Clock (b15-b6 = 0x004 << 6 = 0x0100)
    CLOCK = 0x0100, ///< Generic Clock
    // Generic Computer (b15-b6 = 0x005 << 6 = 0x0140)
    DISPLAY = 0x0140, ///< Generic Display
    // Generic Computer (b15-b6 = 0x006 << 6 = 0x0180)
    REMOTE_CONTROL = 0x0180, ///< Generic Remote Control
    // Generic HID (b15-b6 = 0x00F << 6 = 0x03C0)
    GENERIC_HID = 0x03C0, ///< Generic HID
    KEYBOARD = 0x03C1,    ///< HID Keyboard
    MOUSE = 0x03C2,       ///< HID Mouse
    JOYSTICK = 0x03C3,    ///< HID Joystick
    GAMEPAD = 0x03C4,     ///< HID Gamepad
    TOUCHPAD = 0x03C9,    ///< HID Touchpad
    // Generic Gaming (b15-b6 = 0x02A << 6 = 0x0A80)
    GAMING = 0x0A80, ///< Generic Gaming group
  };

  /**
   * @brief Power state of a BLE radio.
   * @details Representation of the carrier power state in a Handover Select
   *          message.
   */
  enum class CarrierPowerState {
    INACTIVE = 0x00,   ///< Carrier power is off
    ACTIVE = 0x01,     ///< Carrier power is on
    ACTIVATING = 0x02, ///< Carrier power is turning on
    UNKNOWN = 0x03,    ///< Carrier power state is unknown
  };

  /**
   * @brief Extended Inquiry Response (EIR) codes for data types in BT and BLE
   *        out of band (OOB) pairing NDEF records.
   */
  enum class BtEir {
    FLAGS = 0x01, ///< BT flags: b0: LE limited discoverable mode, b1: LE general discoverable mode,
                  ///< b2: BR/EDR not supported, b3: Simultaneous LE & BR/EDR controller, b4:
                  ///< simultaneous LE & BR/EDR Host
    UUIDS_16_BIT_PARTIAL = 0x02,   ///< Incomplete list of 16 bit service class UUIDs
    UUIDS_16_BIT_COMPLETE = 0x03,  ///< Complete list of 16 bit service class UUIDs
    UUIDS_32_BIT_PARTIAL = 0x04,   ///< Incomplete list of 32 bit service class UUIDs
    UUIDS_32_BIT_COMPLETE = 0x05,  ///< Complete list of 32 bit service class UUIDs
    UUIDS_128_BIT_PARTIAL = 0x06,  ///< Incomplete list of 128 bit service class UUIDs
    UUIDS_128_BIT_COMPLETE = 0x07, ///< Complete list of 128 bit service class UUIDs
    SHORT_LOCAL_NAME = 0x08,       ///< Shortened Bluetooth Local Name
    LONG_LOCAL_NAME = 0x09,        ///< Complete Bluetooth Local Name
    TX_POWER_LEVEL = 0x0A,         ///< TX Power level (1 byte), -127 dBm to +127 dBm
    CLASS_OF_DEVICE = 0x0D,        ///< Class of Device
    SP_HASH_C192 = 0x0E,           ///< Simple Pairing Hash C-192
    SP_RANDOM_R192 = 0x0F,         ///< Simple Pairing Randomizer R-192
    SECURITY_MANAGER_TK = 0x10,    ///< Security Manager TK Value (LE Legacy Pairing)
    SECURITY_MANAGER_FLAGS =
        0x11, ///< Flags (1 B), b0: OOB flags field (1 = 00B data present, 0 not), b1: LE Supported
              ///< (host), b2: Simultaneous LE & BR/EDR to same device capable (host), b3: address
              ///< type (0 = public, 1 = random)
    APPEARANCE = 0x19,         ///< Appearance
    MAC = 0x1B,                ///< Bluetooth Device Address
    LE_ROLE = 0x1C,            ///< LE Role
    SP_HASH_C256 = 0x1D,       ///< Simple Pairing Hash C-256
    SP_HASH_R256 = 0x1E,       ///< Simple Pairing Randomizer R-256
    LE_SC_CONFIRMATION = 0x22, ///< LE Secure Connections Confirmation Value
    LE_SC_RANDOM = 0x23,       ///< LE Secure Connections Random Value
  };

  /**
   * @brief Possible roles for BLE records to indicate support for.
   */
  enum class BleRole : uint8_t {
    PERIPHERAL_ONLY = 0x00, ///< Radio can only act as a peripheral
    CENTRAL_ONLY = 0x01,    ///< Radio can only act as a central
    PERIPHERAL_CENTRAL =
        0x02, ///< Radio can act as both a peripheral and a central, but prefers peripheral
    CENTRAL_PERIPHERAL =
        0x03, ///< Radio can act as both a peripheral and a central, but prefers central
  };

  /**
   * @brief Types of configurable encryption for WiFi networks
   */
  enum class WifiEncryptionType {
    NONE = 0x01, ///< No encryption
    WEP = 0x02,  ///< WEP
    TKIP = 0x04, ///< TKIP
    AES = 0x08,  ///< AES
  };

  /**
   * @brief WiFi network authentication
   */
  enum class WifiAuthenticationType {
    OPEN = 0x01,              ///< Open / no security
    WPA_PERSONAL = 0x02,      ///< WPA personal
    SHARED = 0x04,            ///< Shared key
    WPA_ENTERPRISE = 0x08,    ///< WPA enterprise
    WPA2_ENTERPRISE = 0x10,   ///< WPA2 Enterprise
    WPA2_PERSONAL = 0x20,     ///< WPA2 personal
    WPA_WPA2_PERSONAL = 0x22, ///< Both WPA and WPA2 personal
  };

  static constexpr uint8_t HANDOVER_VERSION = 0x13; ///< Connection Handover version 1.3

  /**
   * @brief Makes an NDEF record with header and payload.
   * @param tnf The TNF for this packet.
   * @param type String view for the type of this packet
   * @param payload The payload data for the packet
   */
  explicit Ndef(espp::Ndef::TNF tnf, std::string_view type, std::string_view payload)
      : tnf_(tnf)
      , type_(type)
      , payload_(payload) {}

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
  static Ndef make_uri(std::string_view uri, espp::Ndef::Uic uic = espp::Ndef::Uic::NONE) {
    // prepend URI with identifier code
    std::vector<uint8_t> full;
    full.resize(1 + uri.size());
    full[0] = (uint8_t)uic;
    memcpy(&full[1], uri.data(), uri.size());
    return Ndef(TNF::WELL_KNOWN, "U", std::string_view{(const char *)full.data(), full.size()});
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
   * @brief Configuration structure for wifi configuration ndef structure.
   */
  struct WifiConfig {
    std::string_view ssid; ///< SSID for the network
    std::string_view key;  ///< Security key / password for the network
    espp::Ndef::WifiAuthenticationType authentication =
        espp::Ndef::WifiAuthenticationType::WPA2_PERSONAL; ///< Authentication type the network
                                                           ///< uses.
    espp::Ndef::WifiEncryptionType encryption =
        espp::Ndef::WifiEncryptionType::AES; ///< Encryption type the network uses.
    uint64_t mac_address = 0xFFFFFFFFFFFF;   ///< Broadcast MAC address FF:FF:FF:FF:FF:FF
  };

  /**
   * @brief Create a WiFi credential tag.
   * @param config WifiConfig describing the WiFi network.
   * @return NDEF record object.
   */
  static Ndef make_wifi_config(const espp::Ndef::WifiConfig &config) {
    // make the payload
    std::vector<uint8_t> _payload;
    add_wifi_field(_payload, WifiFieldId::SSID, config.ssid);
    add_wifi_field(_payload, WifiFieldId::NETWORK_KEY, config.key);
    uint8_t auth_bytes[] = {
        (uint8_t)(0x00),
        (uint8_t)(config.authentication),
    };
    auto sv_auth = std::string_view{(const char *)auth_bytes, 2};
    add_wifi_field(_payload, WifiFieldId::AUTH_TYPE, sv_auth);

    // now encapsulate it into a wifi credential
    std::vector<uint8_t> data;
    auto sv_payload = std::string_view{(const char *)_payload.data(), _payload.size()};
    add_wifi_field(data, WifiFieldId::CREDENTIAL, sv_payload);

    // TODO: add fields for the credential
    // * Network Index (deprecated, always set to 1)
    // * Encryption type
    // * MAC address (enrollee's or broadcast address)
    auto sv_data = std::string_view{(const char *)data.data(), data.size()};
    return Ndef(TNF::MIME_MEDIA, "application/vnd.wfa.wsc", sv_data);
  }

  /*
   * @brief Create a collision resolution record.
   * @param random_number Random number to use for the collision resolution.
   * @return NDEF record object.
   */
  static Ndef make_collision_resolution_record(uint16_t random_number) {
    std::vector<uint8_t> _payload;
    _payload.push_back(random_number >> 8);
    _payload.push_back(random_number & 0xFF);
    return Ndef(TNF::WELL_KNOWN, "cr",
                std::string_view{(const char *)_payload.data(), _payload.size()});
  }

  /**
   * @brief Create a Handover Select record for a Bluetooth device.
   * @see
   * https://members.nfc-forum.org/apps/group_public/download.php/18688/NFCForum-AD-BTSSP_1_1.pdf
   * @param carrier_data_ref Reference to the carrier data record, which is the
   *        record that contains the actual bluetooth data. This should be the
   *        same as the id of the carrier data record, such as '0'.
   * @return NDEF record object.
   */
  static Ndef make_handover_select(int carrier_data_ref) {
    std::vector<uint8_t> _payload;
    _payload.push_back(HANDOVER_VERSION);

    Ndef alternative_carrier =
        make_alternative_carrier(CarrierPowerState::ACTIVE, carrier_data_ref);
    auto alternative_carrier_data = alternative_carrier.serialize(true, true);
    _payload.insert(_payload.end(), alternative_carrier_data.begin(),
                    alternative_carrier_data.end());

    return Ndef(TNF::WELL_KNOWN, "Hs",
                std::string_view{(const char *)_payload.data(), _payload.size()});
  }

  /**
   * @brief Create a Handover request record for a Bluetooth device.
   * @see
   * https://members.nfc-forum.org/apps/group_public/download.php/18688/NFCForum-AD-BTSSP_1_1.pdf
   * @param carrier_data_ref Reference to the carrier data record, which is the
   *        record that contains the actual bluetooth data. This should be the
   *        same as the id of the carrier data record, such as '0'.
   * @return NDEF record object.
   */
  static Ndef make_handover_request(int carrier_data_ref) {
    std::vector<uint8_t> _payload;
    _payload.push_back(HANDOVER_VERSION);

    // Handover request requires a collision resolution record, so we'll just
    // add collision resolution record which contains a random number.

    uint16_t random_number = 0;
#if defined(ESP_PLATFORM)
    random_number = esp_random() & 0xFFFF;
#else
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int16_t> dis(0, std::numeric_limits<int16_t>::max());
    random_number = dis(gen);
#endif
    Ndef collision_resolution_record = make_collision_resolution_record(random_number);
    auto collision_resolution_data = collision_resolution_record.serialize(true, false);
    _payload.insert(_payload.end(), collision_resolution_data.begin(),
                    collision_resolution_data.end());

    // now make the alternative carrier record
    Ndef alternative_carrier =
        make_alternative_carrier(CarrierPowerState::ACTIVE, carrier_data_ref);
    auto alternative_carrier_data = alternative_carrier.serialize(true, true);
    _payload.insert(_payload.end(), alternative_carrier_data.begin(),
                    alternative_carrier_data.end());

    return Ndef(TNF::WELL_KNOWN, "Hr",
                std::string_view{(const char *)_payload.data(), _payload.size()});
  }

  /**
   * @brief Create a Handover Request record for a Bluetooth device.
   * @details See page 18 of https://core.ac.uk/download/pdf/250136576.pdf for more details.
   * @param power_state Power state of the alternative carrier.
   * @param carrier_data_ref Reference to the carrier data record, which is the
   *        record that contains the actual bluetooth data. This should be the
   *        same as the id of the carrier data record, such as '0'.
   * @return NDEF record object.
   */
  static Ndef make_alternative_carrier(const CarrierPowerState &power_state, int carrier_data_ref) {
    std::vector<uint8_t> _payload;
    // first byte is carrier power state
    _payload.push_back((uint8_t)power_state);
    // second byte is carrier data reference length
    _payload.push_back(0x01);
    // third byte is carrier data reference (ascii), e.g. '0'
    _payload.push_back((uint8_t)carrier_data_ref);
    // fourth byte is auxiliary data reference count
    _payload.push_back(0x00); // no auxiliary data
    return Ndef(TNF::WELL_KNOWN, "ac",
                std::string_view{(const char *)_payload.data(), _payload.size()});
  }

  /**
   * @brief Static function to make an NDEF record for BT classic OOB Pairing (Android).
   * @param mac_addr 48 bit MAC Address of the BT radio
   * @note If the address is e.g. f4:12:fa:42:fe:9e then the mac_addr should be
   *       0xf412fa42fe9e.
   * @param device_class The bluetooth device class for this radio.
   * @param name Name of the BT device.
   * @param random_value The Simple pairing randomizer R for the pairing.
   * @param confirm_value The Simple pairing hash C (confirm value) for the
   *                      pairing.
   * @return NDEF record object.
   */
  static Ndef make_oob_pairing(uint64_t mac_addr, uint32_t device_class, std::string_view name,
                               std::string_view random_value = "",
                               std::string_view confirm_value = "") {
    std::vector<uint8_t> data;
    // NOTE: for the extended inquiry response (EIR) data types see the
    // BT_HANDOVER_TYPE_ codes here:
    // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

    // See
    // https://members.nfc-forum.org/apps/group_public/download.php/18688/NFCForum-AD-BTSSP_1_1.pdf
    // for examples

    // fill the data - first two bytes are length, bluetooth type, then 6
    // bytes MAC addr.
    data.resize(8);
    data[1] = (uint8_t)BtType::BREDR;
    // 6 bytes of BT device MAC address in reverse order
    data[2] = (mac_addr >> 0) & 0xFF;
    data[3] = (mac_addr >> 8) & 0xFF;
    data[4] = (mac_addr >> 16) & 0xFF;
    data[5] = (mac_addr >> 24) & 0xFF;
    data[6] = (mac_addr >> 32) & 0xFF;
    data[7] = (mac_addr >> 40) & 0xFF;

    // add optional EIR data (no specific order required)
    add_bt_eir(data, BtEir::LONG_LOCAL_NAME, name);

    // (optional  0x22) secure connections confirmation value (16 bytes)
    if (confirm_value.size() == 16) {
      add_bt_eir(data, BtEir::SP_HASH_C192, confirm_value);
    }

    // (optional  0x23) secure connections random value (16 bytes)
    if (random_value.size() == 16) {
      add_bt_eir(data, BtEir::SP_RANDOM_R192, random_value);
    }

    // now make sure the length is updated (includes length field)
    int length = data.size();
    data[0] = length;
    auto sv_data = std::string_view{(const char *)data.data(), data.size()};
    return Ndef(TNF::MIME_MEDIA, "application/vnd.bluetooth.ep.oob", sv_data);
  }

  /**
   * @brief Static function to make an NDEF record for BLE OOB Pairing (Android).
   * @param mac_addr 48 bit MAC Address of the BLE radio.
   * @note If the address is e.g. f4:12:fa:42:fe:9e then the mac_addr should be
   *       0xf412fa42fe9e.
   * @param role The BLE role of the device (central / peripheral / dual)
   * @param name Name of the BLE device. Optional.
   * @param appearance BtAppearance of the device. Optional.
   * @param random_value The Simple pairing randomizer R for the pairing. (16 bytes, optional)
   * @param confirm_value The Simple pairing hash C (confirm value) for the pairing. (16 bytes,
   * optional)
   * @param tk Temporary key for the pairing (16 bytes, optional)
   * @return NDEF record object.
   */
  static Ndef
  make_le_oob_pairing(uint64_t mac_addr, espp::Ndef::BleRole role, std::string_view name = "",
                      espp::Ndef::BtAppearance appearance = espp::Ndef::BtAppearance::UNKNOWN,
                      std::string_view random_value = "", std::string_view confirm_value = "",
                      std::string_view tk = "") {
    std::vector<uint8_t> data;
    // NOTE: for the extended inquiry response (EIR) data types see the
    // BT_HANDOVER_TYPE_ codes here:
    // https://android.googlesource.com/platform/packages/apps/Nfc/+/master/src/com/android/nfc/handover/HandoverDataParser.java#61

    // fill the data - NOTE: all BLE data is passed as EIR data with no additional data

    // See
    // https://members.nfc-forum.org/apps/group_public/download.php/18688/NFCForum-AD-BTSSP_1_1.pdf
    // for examples

    // (mandatory 0x1B) LE device address in reverse order. According to section
    // 3.3.1 of NFCForum-AD-BTSSP_1_1.pdf, the data value consists of 7 octets,
    // with the 6 least significant octets being the MAC address in reverse
    // order. The least significant bit in the most significant octet is the
    // random address bit.
    uint8_t mac_addr_bytes[] = {
        (uint8_t)(mac_addr >> 0 & 0xFF),
        (uint8_t)(mac_addr >> 8 & 0xFF),
        (uint8_t)(mac_addr >> 16 & 0xFF),
        (uint8_t)(mac_addr >> 24 & 0xFF),
        (uint8_t)(mac_addr >> 32 & 0xFF),
        (uint8_t)(mac_addr >> 40 & 0xFF),
        (uint8_t)(0x00) // Public address type
    };
    add_bt_eir(data, BtEir::MAC,
               std::string_view{(const char *)&mac_addr_bytes[0], sizeof(mac_addr_bytes)});

    // (mandatory 0x1C) LE role
    add_bt_eir(data, BtEir::LE_ROLE, std::string_view{(const char *)&role, 1});

    // optional temporary key (0x10)
    // Security Manager TK value (LE legacy pairing) (16 bytes)
    // The TK value requirements for such exchange is described in
    // [BLUETOOTH_CORE] Volume 3, Part H, Section 2.3.5.4.
    if (tk.size() == 16) {
      add_bt_eir(data, BtEir::SECURITY_MANAGER_TK, tk);
    }

    // (optional  0x22) secure connections confirmation value (16 bytes)
    if (confirm_value.size() == 16) {
      add_bt_eir(data, BtEir::LE_SC_CONFIRMATION, confirm_value);
    }

    // (optional  0x23) secure connections random value (16 bytes)
    if (random_value.size() == 16) {
      add_bt_eir(data, BtEir::LE_SC_RANDOM, random_value);
    }

    // optional appearance (0x19)
    uint8_t appearance_bytes[] = {(uint8_t)((uint16_t)appearance >> 8),
                                  (uint8_t)((uint16_t)appearance & 0xFF)};
    add_bt_eir(data, BtEir::APPEARANCE,
               std::string_view{(const char *)&appearance_bytes[0], sizeof(appearance_bytes)});

    // optional local name (0x09)
    if (name.size() > 0) {
      add_bt_eir(data, BtEir::LONG_LOCAL_NAME, name);
    }

    // optional Flags (0x01)
    // 0x06: BR/EDR not supported, LE supported, Simultaneous LE/BT to same
    // device capable (controller)
    uint8_t flags_bytes[] = {0x06};
    add_bt_eir(data, BtEir::FLAGS,
               std::string_view{(const char *)&flags_bytes[0], sizeof(flags_bytes)});

    auto sv_data = std::string_view{(const char *)data.data(), data.size()};
    return Ndef(TNF::MIME_MEDIA, "application/vnd.bluetooth.le.oob", sv_data);
  }

  /**
   * @brief Serialize the NDEF record into a sequence of bytes.
   * @param message_begin True if this is the first record in the message.
   * @param message_end True if this is the last record in the message.
   * @return The vector<uint8_t> of bytes representing the NDEF record.
   */
  std::vector<uint8_t> serialize(bool message_begin = true, bool message_end = true) {
    std::vector<uint8_t> data;
    int size = get_size();
    set_flags(message_begin, message_end);
    data.resize(size);
    int offset = 0;
    data[offset++] = flags_.raw;
    data[offset++] = type_.size();
    if (flags_.SR) {
      data[offset++] = (uint8_t)payload_.size();
    } else {
      uint32_t _num_bytes = payload_.size();
      data[offset++] = (uint8_t)(_num_bytes >> 24 & 0xFF);
      data[offset++] = (uint8_t)(_num_bytes >> 16 & 0xFF);
      data[offset++] = (uint8_t)(_num_bytes >> 8 & 0xFF);
      data[offset++] = (uint8_t)(_num_bytes >> 0 & 0xFF);
    }
    // If we have an ID, indicate the number of bytes of the ID
    if (flags_.IL) {
      data[offset++] = 1; // TODO: this is hardcoded to 1 byte for now
    }
    // copy the type
    memcpy(&data[offset], type_.data(), type_.size());
    offset += type_.size();
    // copy the ID (if present)
    if (flags_.IL) {
      data[offset++] = (uint8_t)id_;
    }
    // copy the payload data
    memcpy(&data[offset], payload_.data(), payload_.size());
    return data;
  }

  /**
   * @brief Return just the payload as a vector of bytes.
   * @return Payload of the NDEF record as a vector of bytes.
   */
  std::vector<uint8_t> payload() {
    return std::vector<uint8_t>(payload_.data(), payload_.data() + payload_.size());
  }

  /**
   * @brief Set the payload ID of the NDEF record.
   * @param id ID of the NDEF record.
   */
  void set_id(int id) { id_ = id; }

  /**
   * @brief Get the ID of the NDEF record.
   * @return ID of the NDEF record.
   */
  int get_id() const { return id_; }

  /**
   * @brief Get the number of bytes needed for the NDEF record.
   * @return Size of the NDEF record (bytes), for serialization.
   */
  int get_size() const {
    int num_id_length_bytes = 0;
    int num_id_bytes = 0;
    if (id_ != -1) {
      num_id_length_bytes = 1;
      num_id_bytes = 1;
    }
    int num_payload_bytes = payload_.size();
    uint8_t num_payload_length_bytes = 1;
    if (num_payload_bytes > 255) {
      num_payload_length_bytes = 4;
    }
    uint8_t type_length = type_.size();
    int total_length = sizeof(flags_.raw) + sizeof(type_length) + num_payload_length_bytes +
                       num_id_length_bytes + type_length + num_id_bytes + num_payload_bytes;
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
      uint8_t raw = 0;
    };
  };

  enum class WifiFieldId : uint16_t {
    CREDENTIAL = 0x100E,
    SSID = 0x1045,
    NETWORK_KEY = 0x1027,
    AUTH_TYPE = 0x1003,
  };

  void set_flags(bool message_begin, bool message_end) {
    flags_.TNF = (uint8_t)tnf_;
    flags_.IL = id_ != -1 ? 1 : 0;
    // short record?
    flags_.SR = payload_.size() < 256 ? 1 : 0;
    flags_.CF = 0; // first record of a chunk
    flags_.ME = message_end ? 1 : 0;
    flags_.MB = message_begin ? 1 : 0;
  }

  static int add_wifi_field(std::vector<uint8_t> &data, WifiFieldId field,
                            std::string_view _payload) {
    // field ID = short
    // field size = short
    // payload

    // add 2 byte field id
    uint8_t field_bytes[] = {
        (uint8_t)((uint16_t)field >> 8 & 0xFF),
        (uint8_t)((uint16_t)field >> 0 & 0xFF),
    };
    data.push_back(field_bytes[0]);
    data.push_back(field_bytes[1]);
    // add 2 byte size
    int size = _payload.size();
    uint8_t size_bytes[] = {
        (uint8_t)(size >> 8 & 0xFF),
        (uint8_t)(size >> 0 & 0xFF),
    };
    data.push_back(size_bytes[0]);
    data.push_back(size_bytes[1]);
    // add _payload
    std::copy(_payload.begin(), _payload.end(), std::back_inserter(data));
    // return how many bytes this field is
    return 4 + _payload.size();
  }

  static int add_bt_eir(std::vector<uint8_t> &data, BtEir eir_type, std::string_view _payload) {
    uint8_t num_eir_bytes = 0;
    switch (eir_type) {
    case BtEir::UUIDS_16_BIT_PARTIAL:
    case BtEir::UUIDS_16_BIT_COMPLETE:
    case BtEir::UUIDS_32_BIT_PARTIAL:
    case BtEir::UUIDS_32_BIT_COMPLETE:
    case BtEir::UUIDS_128_BIT_PARTIAL:
    case BtEir::UUIDS_128_BIT_COMPLETE:
      num_eir_bytes = _payload.size();
      break;
    case BtEir::FLAGS:
      num_eir_bytes = 1;
      break;
    case BtEir::SHORT_LOCAL_NAME:
    case BtEir::LONG_LOCAL_NAME:
      num_eir_bytes = _payload.size();
      break;
    case BtEir::CLASS_OF_DEVICE:
      num_eir_bytes = 3; // 24 bit class of device structure
      break;
    case BtEir::SECURITY_MANAGER_TK:
      num_eir_bytes = _payload.size();
      break;
    case BtEir::APPEARANCE:
      num_eir_bytes = 2; // 2 byte appearance type
      break;
    case BtEir::MAC:
      num_eir_bytes = _payload.size();
      break;
    case BtEir::LE_ROLE:
      num_eir_bytes = 1;
      break;
    case BtEir::SP_HASH_C192:
    case BtEir::LE_SC_CONFIRMATION:
      // NOTE: should be 16
      num_eir_bytes = _payload.size();
      break;
    case BtEir::SP_RANDOM_R192:
    case BtEir::LE_SC_RANDOM:
      // NOTE: should be 16
      num_eir_bytes = _payload.size();
      break;
    default:
      break;
    }
    data.push_back(num_eir_bytes + 1); // total size of EIR (including eir data type)
    data.push_back((uint8_t)eir_type);
    for (int i = 0; i < num_eir_bytes; i++) {
      data.push_back(_payload[i]);
    }
    // return the total number of bytes written
    return 2 + num_eir_bytes;
  }

  TNF tnf_;
  Flags flags_{};
  int id_{-1};
  std::string type_{""};
  std::string payload_{""};
};
} // namespace espp
