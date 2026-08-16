#include <array>
#include <string>
#include <vector>

#include "cdr.hpp"
#include "logger.hpp"

namespace {

// A plain aggregate is all it takes — the compiler generates the
// serialization code from the struct definition itself. Types default to
// @appendable extensibility (XTypes default); opt into @final with
// `static constexpr auto cdr_extensibility = cdr::extensibility::final;`.
struct ImuSample {
  uint64_t stamp_us{};
  std::array<float, 3> accel{};
  std::array<float, 3> gyro{};
  float temperature{};
  std::string frame_id{};
};

} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "cdr_example", .level = espp::Logger::Verbosity::INFO});

  //! [cdr example]
  const ImuSample sample{
      .stamp_us = 123456789,
      .accel = {0.0f, 0.0f, 9.81f},
      .gyro = {0.01f, -0.02f, 0.0f},
      .temperature = 25.5f,
      .frame_id = "imu_link",
  };

  // XCDR2 (appendable): what FastDDS / OpenDDS speak by default.
  auto bytes = cdr::serialize(sample);
  // XCDR1 (plain CDR): what ROS 2 and CycloneDDS speak by default.
  auto ros2_bytes = cdr::serialize<cdr::xcdr1>(sample);
  if (!bytes) {
    logger.error("XCDR2 serialize failed: {}", cdr::to_string(bytes.error().code));
    return;
  }
  if (!ros2_bytes) {
    logger.error("XCDR1 serialize failed: {}", cdr::to_string(ros2_bytes.error().code));
    return;
  }
  logger.info("ImuSample: {} bytes as XCDR2, {} bytes as XCDR1 (ROS 2)", bytes->size(),
              ros2_bytes->size());

  // Deserialize picks version + endianness from the encapsulation header and
  // returns std::expected — errors carry a code, payload offset, and the
  // field that failed.
  auto restored = cdr::deserialize<ImuSample>(*bytes);
  if (!restored) {
    logger.error("deserialize failed at {} offset {} in field '{}'",
                 cdr::to_string(restored.error().code), restored.error().offset,
                 restored.error().field);
    return;
  }
  logger.info("round-trip ok: stamp={} frame='{}' accel.z={}", restored->stamp_us,
              restored->frame_id, restored->accel[2]);

  // Zero-allocation path for hot loops: serialize into a fixed buffer.
  std::array<std::byte, 128> fixed{};
  if (auto n = cdr::serialize_into(sample, fixed)) {
    logger.info("serialize_into wrote {} bytes (exact size query: {})", *n,
                cdr::serialized_size(sample));
  }

  // PL_CDR parameter lists — the encoding RTPS discovery (SPDP/SEDP) uses.
  std::vector<std::byte> pl_buf;
  cdr::param_list_writer pl(pl_buf);
  pl.add(uint16_t{0x0050}, std::array<uint8_t, 16>{1, 2, 3, 4}); // PID_PARTICIPANT_GUID
  pl.add(uint16_t{0x0062}, std::string("esp32_node"));           // PID_ENTITY_NAME
  if (pl.finish()) {
    auto reader = cdr::param_list_reader::from_encapsulated(pl_buf);
    while (reader) {
      auto param = reader->next();
      if (!param || !*param)
        break;
      logger.info("  parameter pid=0x{:04x} ({} bytes)", (*param)->pid, (*param)->value.size());
    }
  }
  //! [cdr example]

  logger.info("example complete");
}
