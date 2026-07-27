#include "nmea_parser.hpp"

#include <array>
#include <charconv>
#include <cstdlib>
#include <cstring>

using namespace espp;

namespace {
// split an NMEA sentence body into its comma-separated fields; empty fields
// are preserved. Returns the number of fields.
size_t split_fields(std::string_view body, std::array<std::string_view, 24> &fields) {
  size_t count = 0;
  size_t start = 0;
  while (count < fields.size()) {
    size_t comma = body.find(',', start);
    if (comma == std::string_view::npos) {
      fields[count++] = body.substr(start);
      break;
    }
    fields[count++] = body.substr(start, comma - start);
    start = comma + 1;
  }
  return count;
}

float to_float(std::string_view s, float default_value = 0) {
  if (s.empty()) {
    return default_value;
  }
  // std::from_chars<float> is not available on all toolchains; strtof needs a
  // null-terminated buffer
  char buffer[24];
  size_t len = std::min(s.size(), sizeof(buffer) - 1);
  std::memcpy(buffer, s.data(), len);
  buffer[len] = '\0';
  return std::strtof(buffer, nullptr);
}

int to_int(std::string_view s, int default_value = 0) {
  int value = default_value;
  std::from_chars(s.data(), s.data() + s.size(), value);
  return value;
}

// convert an NMEA "[d]ddmm.mmmm" coordinate + hemisphere to decimal degrees
double to_degrees(std::string_view value, std::string_view hemisphere) {
  if (value.empty()) {
    return 0;
  }
  size_t dot = value.find('.');
  if (dot == std::string_view::npos || dot < 3) {
    return 0;
  }
  // degrees are everything more than two digits left of the decimal point
  int degrees = to_int(value.substr(0, dot - 2));
  char buffer[24];
  std::string_view minutes_str = value.substr(dot - 2);
  size_t len = std::min(minutes_str.size(), sizeof(buffer) - 1);
  std::memcpy(buffer, minutes_str.data(), len);
  buffer[len] = '\0';
  double minutes = std::strtod(buffer, nullptr);
  double result = degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    result = -result;
  }
  return result;
}
} // namespace

bool NmeaParser::checksum_valid(std::string_view sentence) {
  if (sentence.size() < 4 || sentence[0] != '$') {
    return false;
  }
  size_t star = sentence.find('*');
  if (star == std::string_view::npos || star + 3 > sentence.size()) {
    return false;
  }
  uint8_t checksum = 0;
  for (size_t i = 1; i < star; i++) {
    checksum ^= (uint8_t)sentence[i];
  }
  auto hex_value = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return -1;
  };
  int high = hex_value(sentence[star + 1]);
  int low = hex_value(sentence[star + 2]);
  if (high < 0 || low < 0) {
    return false;
  }
  return checksum == (uint8_t)((high << 4) | low);
}

bool NmeaParser::parse(std::string_view sentence) {
  // strip trailing CR/LF
  while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n')) {
    sentence.remove_suffix(1);
  }
  if (!checksum_valid(sentence)) {
    return false;
  }
  // "$GPRMC,...*XX" -> type "RMC" (skipping the 2-character talker id),
  // body between the first comma and the '*'
  size_t star = sentence.find('*');
  size_t comma = sentence.find(',');
  if (comma == std::string_view::npos || comma < 4 || comma > star) {
    return false;
  }
  std::string_view type = sentence.substr(3, comma - 3);
  std::string_view body = sentence.substr(comma + 1, star - comma - 1);
  if (type == "RMC") {
    return parse_rmc(body);
  } else if (type == "GGA") {
    return parse_gga(body);
  }
  return false;
}

bool NmeaParser::parse_rmc(std::string_view body) {
  // hhmmss.ss,A,ddmm.mm,N,dddmm.mm,W,speed,course,ddmmyy,...
  std::array<std::string_view, 24> f;
  size_t count = split_fields(body, f);
  if (count < 9) {
    return false;
  }
  if (f[0].size() >= 6) {
    fix_.hour = to_int(f[0].substr(0, 2));
    fix_.minute = to_int(f[0].substr(2, 2));
    fix_.second = to_float(f[0].substr(4));
  }
  fix_.valid = (f[1] == "A");
  if (fix_.valid) {
    fix_.latitude = to_degrees(f[2], f[3]);
    fix_.longitude = to_degrees(f[4], f[5]);
    fix_.speed_knots = to_float(f[6]);
    fix_.course_degrees = to_float(f[7]);
  }
  if (f[8].size() >= 6) {
    fix_.day = to_int(f[8].substr(0, 2));
    fix_.month = to_int(f[8].substr(2, 2));
    fix_.year = 2000 + to_int(f[8].substr(4, 2));
  }
  return true;
}

bool NmeaParser::parse_gga(std::string_view body) {
  // hhmmss.ss,ddmm.mm,N,dddmm.mm,W,quality,numsats,hdop,alt,M,...
  std::array<std::string_view, 24> f;
  size_t count = split_fields(body, f);
  if (count < 9) {
    return false;
  }
  fix_.fix_quality = to_int(f[5]);
  fix_.num_satellites = to_int(f[6]);
  fix_.hdop = to_float(f[7]);
  if (fix_.fix_quality > 0) {
    fix_.latitude = to_degrees(f[1], f[2]);
    fix_.longitude = to_degrees(f[3], f[4]);
    fix_.altitude = to_float(f[8]);
  }
  return true;
}
