#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "cdr.hpp"
#include "logger.hpp"

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "cdr_example", .level = espp::Logger::Verbosity::INFO});

  std::array<uint8_t, 4> input_magic{'C', 'D', 'R', '!'};
  std::array<uint16_t, 3> input_values{10, 20, 30};

  //! [cdr example]
  espp::CdrWriter writer({
      .encapsulation = espp::CdrEncapsulation::CDR_LE,
      .include_encapsulation = true,
  });
  writer.write<uint32_t>(42);
  writer.write<float>(3.25f);
  writer.write_string("hello cdr");
  writer.write_sequence<uint16_t>(input_values);

  auto payload = writer.take_buffer();
  logger.info("Serialized {} bytes of CDR data", payload.size());

  auto inline_writer = espp::CdrWriter::make_body_writer(espp::CdrEncapsulation::CDR_LE);
  inline_writer.write_array(input_magic);
  inline_writer.write_string("embedded field");
  auto inline_payload =
      espp::CdrWriter::encapsulate(inline_writer.payload(), espp::CdrEncapsulation::PL_CDR_LE);

  espp::CdrReader reader(payload);
  espp::CdrReader inline_reader(inline_payload);
  uint32_t decoded_count = 0;
  float decoded_scale = 0.0f;
  std::string decoded_text;
  std::vector<uint16_t> decoded_values;
  std::array<uint8_t, 4> decoded_magic{};
  std::string decoded_inline_text;

  bool ok = reader.read<uint32_t>(decoded_count) && reader.read<float>(decoded_scale) &&
            reader.read_string(decoded_text) && reader.read_sequence<uint16_t>(decoded_values);
  bool inline_ok = inline_reader.encapsulation() == espp::CdrEncapsulation::PL_CDR_LE &&
                   inline_reader.read_array(decoded_magic) &&
                   inline_reader.read_string(decoded_inline_text);
  //! [cdr example]

  if (!ok || !inline_ok) {
    logger.error("Failed to decode CDR payload");
    return;
  }

  logger.info("Decoded count={}, scale={:.2f}, text='{}', sequence size={}, embedded='{}'",
              decoded_count, decoded_scale, decoded_text, decoded_values.size(),
              decoded_inline_text);

  if (decoded_count != 42 || decoded_scale != 3.25f || decoded_text != "hello cdr" ||
      decoded_values.size() != input_values.size() ||
      !std::equal(decoded_values.begin(), decoded_values.end(), input_values.begin()) ||
      decoded_magic != input_magic || decoded_inline_text != "embedded field") {
    logger.error("CDR round-trip mismatch");
    return;
  }

  logger.info("CDR round-trip succeeded");
}
