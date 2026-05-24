#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <variant>
#include <vector>

#include "base_component.hpp"

namespace espp {

enum class SnmpErrc {
  success = 0,
  invalid_configuration,
  invalid_argument,
  unsupported_version,
  unsupported_value_type,
  unsupported_security_level,
  name_resolution_failed,
  encode_failed,
  decode_failed,
  timeout,
  transport_error,
  response_error,
  request_id_mismatch,
  report_received,
  authentication_failed,
  privacy_failed,
  discovery_failed,
  not_in_time_window,
};

std::error_code make_error_code(SnmpErrc e);

class Snmp : public BaseComponent {
public:
  enum class Version { V2C, V3 };

  enum class RequestType {
    Get,
    GetNext,
    GetBulk,
    Set,
    Response,
    Report,
  };

  enum class ValueType {
    Integer,
    OctetString,
    Null,
    ObjectIdentifier,
    IpAddress,
    Counter32,
    Unsigned32,
    Gauge32,
    TimeTicks,
    Opaque,
    Counter64,
    NoSuchObject,
    NoSuchInstance,
    EndOfMibView,
  };

  enum class SecurityLevel { AuthPriv };

  enum class InterfaceAdminStatus : int32_t {
    Up = 1,
    Down = 2,
    Testing = 3,
  };

  enum class InterfaceOperStatus : int32_t {
    Up = 1,
    Down = 2,
    Testing = 3,
    Unknown = 4,
    Dormant = 5,
    NotPresent = 6,
    LowerLayerDown = 7,
  };

  struct Endpoint {
    std::string host{"127.0.0.1"};
    uint16_t port{161};
  };

  struct V2cConfig {
    std::string community{"public"};
  };

  struct V3Config {
    std::string username;
    std::string auth_password;
    std::string privacy_password;
    std::string context_name;
    SecurityLevel security_level{SecurityLevel::AuthPriv};
  };

  struct Config {
    Endpoint endpoint;
    Version version{Version::V2C};
    V2cConfig v2c;
    V3Config v3;
    std::chrono::milliseconds timeout{1500};
    size_t retries{2};
    size_t max_message_size{1500};
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  struct Oid {
    std::vector<uint32_t> nodes;

    Oid() = default;
    Oid(std::initializer_list<uint32_t> init);
    explicit Oid(const std::vector<uint32_t> &values);

    static std::optional<Oid> try_from_string(std::string_view oid_string);
    static Oid from_string(std::string_view oid_string);
    std::string to_string() const;
    bool starts_with(const Oid &prefix) const;
    bool operator<(const Oid &other) const;

    friend bool operator==(const Oid &lhs, const Oid &rhs) = default;
  };

  struct IpAddress {
    std::array<uint8_t, 4> bytes{0, 0, 0, 0};

    std::string to_string() const;
    friend bool operator==(const IpAddress &lhs, const IpAddress &rhs) = default;
  };

  struct Value {
    using octets_t = std::vector<uint8_t>;
    using storage_t =
        std::variant<std::monostate, int32_t, uint32_t, uint64_t, octets_t, Oid, IpAddress>;

    ValueType type{ValueType::Null};
    storage_t data{};

    static Value integer(int32_t value);
    static Value octet_string(std::string_view value);
    static Value octet_string(const octets_t &value);
    static Value null();
    static Value object_identifier(const Oid &value);
    static Value ip_address(const IpAddress &value);
    static Value counter32(uint32_t value);
    static Value unsigned32(uint32_t value);
    static Value gauge32(uint32_t value);
    static Value time_ticks(uint32_t value);
    static Value opaque(const octets_t &value);
    static Value counter64(uint64_t value);
    static Value no_such_object();
    static Value no_such_instance();
    static Value end_of_mib_view();

    bool is_exception() const;
    bool is_end_of_mib_view() const;
    std::optional<int32_t> as_integer() const;
    std::optional<uint32_t> as_unsigned32() const;
    std::optional<uint64_t> as_counter64() const;
    std::optional<octets_t> as_octets() const;
    std::optional<std::string> as_string() const;
    std::optional<Oid> as_oid() const;
    std::optional<IpAddress> as_ip_address() const;
    std::string to_string() const;
  };

  struct VarBind {
    Oid oid;
    Value value{Value::null()};
  };

  struct Response {
    RequestType type{RequestType::Response};
    int32_t request_id{0};
    int32_t error_status{0};
    int32_t error_index{0};
    std::vector<VarBind> varbinds;

    bool is_report() const { return type == RequestType::Report; }
    bool has_error() const { return error_status != 0; }
  };

  struct BulkOptions {
    int32_t non_repeaters{0};
    int32_t max_repetitions{10};
  };

  struct WalkOptions {
    int32_t max_repetitions{10};
    size_t max_rows{0};
    bool use_get_bulk{true};
  };

  struct SystemGroup {
    std::string sys_descr;
    Oid sys_object_id;
    uint32_t sys_up_time{0};
    std::string sys_contact;
    std::string sys_name;
    std::string sys_location;
  };

  struct InterfaceInfo {
    uint32_t if_index{0};
    std::string if_descr;
    uint32_t if_type{0};
    uint32_t if_mtu{0};
    uint32_t if_speed{0};
    InterfaceAdminStatus if_admin_status{InterfaceAdminStatus::Down};
    InterfaceOperStatus if_oper_status{InterfaceOperStatus::Down};
    std::string if_alias;
  };

  explicit Snmp(const Config &config);
  ~Snmp();

  void set_config(const Config &config);
  Config get_config() const;

  bool get(const Oid &oid, Value &value, std::error_code &ec) const;
  bool get(const std::vector<Oid> &oids, Response &response, std::error_code &ec) const;

  bool get_next(const Oid &oid, VarBind &varbind, std::error_code &ec) const;
  bool get_next(const std::vector<Oid> &oids, Response &response, std::error_code &ec) const;

  bool get_bulk(const Oid &oid, std::vector<VarBind> &varbinds, std::error_code &ec) const;
  bool get_bulk(const Oid &oid, std::vector<VarBind> &varbinds, std::error_code &ec,
                const BulkOptions &options) const;
  bool get_bulk(const std::vector<Oid> &oids, Response &response, std::error_code &ec) const;
  bool get_bulk(const std::vector<Oid> &oids, Response &response, std::error_code &ec,
                const BulkOptions &options) const;

  bool set(const VarBind &varbind, Response &response, std::error_code &ec) const;
  bool set(const std::vector<VarBind> &varbinds, Response &response, std::error_code &ec) const;

  bool walk(const Oid &root, std::vector<VarBind> &varbinds, std::error_code &ec) const;
  bool walk(const Oid &root, std::vector<VarBind> &varbinds, std::error_code &ec,
            const WalkOptions &options) const;

  bool get_system_group(SystemGroup &system, std::error_code &ec) const;
  bool get_interface(uint32_t if_index, InterfaceInfo &info, std::error_code &ec) const;
  bool walk_interfaces(std::vector<InterfaceInfo> &interfaces, std::error_code &ec) const;

  bool set_system_name(std::string_view name, std::error_code &ec) const;
  bool set_system_contact(std::string_view contact, std::error_code &ec) const;
  bool set_system_location(std::string_view location, std::error_code &ec) const;
  bool set_interface_admin_status(uint32_t if_index, InterfaceAdminStatus status,
                                  std::error_code &ec) const;
  bool set_interface_alias(uint32_t if_index, std::string_view alias, std::error_code &ec) const;

  static Oid sys_descr_oid();
  static Oid sys_object_id_oid();
  static Oid sys_up_time_oid();
  static Oid sys_contact_oid();
  static Oid sys_name_oid();
  static Oid sys_location_oid();
  static Oid if_index_oid(uint32_t if_index);
  static Oid if_descr_oid(uint32_t if_index);
  static Oid if_type_oid(uint32_t if_index);
  static Oid if_mtu_oid(uint32_t if_index);
  static Oid if_speed_oid(uint32_t if_index);
  static Oid if_admin_status_oid(uint32_t if_index);
  static Oid if_oper_status_oid(uint32_t if_index);
  static Oid if_alias_oid(uint32_t if_index);

protected:
  bool request(RequestType type, const std::vector<VarBind> &request_varbinds, Response &response,
               std::error_code &ec, const BulkOptions &bulk_options) const;

  mutable std::mutex config_mutex_;
  Config config_;
};
} // namespace espp

namespace std {
template <> struct is_error_code_enum<espp::SnmpErrc> : true_type {};
} // namespace std
