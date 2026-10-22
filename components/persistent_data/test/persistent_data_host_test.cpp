// Host-buildable test for the PersistentData file format, version list,
// migration chain and file helpers (include/detail/persistent_data_codec.hpp --
// standard library + alpaca only). Build & run from the repo root, as one
// command (alpaca is third-party, hence -isystem so its own warnings do not
// trip -Werror):
//   c++ -std=c++23 -Wall -Wextra -Werror
//       -I components/persistent_data/include -I components/serialization/include
//       -isystem components/serialization/detail/alpaca/include
//       components/persistent_data/test/persistent_data_host_test.cpp -o test && ./test

#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "detail/persistent_data_codec.hpp"

namespace pd = espp::persistent_data;

static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    ++g_checks;                                                                                    \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// ------------------------------ test types -----------------------------------

struct ConfigV1 {
  static constexpr uint32_t VERSION = 1;
  uint32_t counter{0};
  std::string name{};
};

struct ConfigV2 {
  static constexpr uint32_t VERSION = 2;
  uint32_t counter{0};
  std::string name{};
  std::vector<float> gains{};

  static ConfigV2 migrate(const ConfigV1 &v1) {
    return {.counter = v1.counter, .name = v1.name, .gains = {1.0f, 1.0f}};
  }
};

struct ConfigV3 {
  static constexpr uint32_t VERSION = 3;
  // the counter was dropped, a nested struct and a flag were added
  struct Limits {
    float min{0};
    float max{1};
  };
  std::string name{};
  std::vector<float> gains{};
  Limits limits{};
  bool enabled{true};

  static ConfigV3 migrate(const ConfigV2 &v2) {
    return {.name = v2.name + " (migrated)",
            .gains = v2.gains,
            .limits = {.min = -1.0f, .max = 1.0f},
            .enabled = v2.counter > 0};
  }
};

// a different type that claims VERSION 3 as well, with another layout
struct OtherV3 {
  static constexpr uint32_t VERSION = 3;
  uint64_t a{0};
  uint64_t b{0};
};

// a hypothetical future version this firmware does not know about
struct ConfigV4 {
  static constexpr uint32_t VERSION = 4;
  std::string name{};
  uint8_t flags{0};
};

using Full = pd::VersionList<ConfigV3, ConfigV2, ConfigV1>;
using Recent = pd::VersionList<ConfigV3, ConfigV2>;
using Single = pd::VersionList<ConfigV3>;

static_assert(Full::count == 3);
static_assert(Full::current_version == 3);
static_assert(Full::supports(1) && Full::supports(2) && Full::supports(3) && !Full::supports(4));
static_assert(!Recent::supports(1));
static_assert(std::is_same_v<Full::Current, ConfigV3>);
static_assert(pd::VersionedType<ConfigV1> && !pd::VersionedType<int>);
static_assert(pd::MigratableFrom<ConfigV1, ConfigV2> && !pd::MigratableFrom<ConfigV1, ConfigV3>);

// ------------------------------ helpers --------------------------------------

template <typename T> static std::vector<uint8_t> encoded(const T &value) {
  std::vector<uint8_t> bytes;
  std::error_code ec;
  bool ok = pd::encode(value, T::VERSION, bytes, ec);
  CHECK(ok && !ec);
  return bytes;
}

static bool decode_full(const std::vector<uint8_t> &bytes, ConfigV3 &out, uint32_t &version,
                        std::error_code &ec) {
  std::span<const uint8_t> payload;
  pd::Header header;
  if (!pd::parse(bytes, header, payload, ec))
    return false;
  version = header.version;
  return Full::decode(header, payload, out, ec);
}

// ------------------------------ tests ----------------------------------------

static void test_header_layout() {
  std::printf("header layout\n");
  ConfigV3 v3{.name = "abc", .gains = {0.5f}, .limits = {.min = 0, .max = 2}, .enabled = false};
  auto bytes = encoded(v3);
  CHECK(bytes.size() > pd::HEADER_SIZE);
  CHECK(pd::get_u32(bytes.data()) == pd::MAGIC);
  CHECK(bytes[0] == 'E' && bytes[1] == 'S' && bytes[2] == 'P' && bytes[3] == 'D');
  CHECK(pd::get_u32(bytes.data() + 4) == 3);
  CHECK(pd::get_u32(bytes.data() + 8) == pd::layout::fingerprint<ConfigV3>());
  CHECK(pd::get_u32(bytes.data() + 12) == bytes.size() - pd::HEADER_SIZE);
  // encoding is deterministic
  CHECK(encoded(v3) == bytes);
}

static void test_round_trip_current() {
  std::printf("round trip of the current version\n");
  ConfigV3 v3{
      .name = "abc", .gains = {0.5f, 0.25f}, .limits = {.min = -3, .max = 3}, .enabled = false};
  auto bytes = encoded(v3);
  ConfigV3 out{};
  uint32_t version = 0;
  std::error_code ec;
  CHECK(decode_full(bytes, out, version, ec));
  CHECK(!ec);
  CHECK(version == 3);
  CHECK(out.name == "abc");
  CHECK(out.gains == std::vector<float>({0.5f, 0.25f}));
  CHECK(out.limits.min == -3 && out.limits.max == 3);
  CHECK(out.enabled == false);
}

static void test_migration_chain() {
  std::printf("migration chain v1 -> v2 -> v3\n");
  {
    auto bytes = encoded(ConfigV1{.counter = 7, .name = "one"});
    ConfigV3 out{};
    uint32_t version = 0;
    std::error_code ec;
    CHECK(decode_full(bytes, out, version, ec));
    CHECK(version == 1);
    CHECK(out.name == "one (migrated)");
    CHECK(out.gains == std::vector<float>({1.0f, 1.0f})); // from v2's migrate
    CHECK(out.limits.min == -1.0f && out.limits.max == 1.0f);
    CHECK(out.enabled == true); // counter 7 > 0
  }
  {
    auto bytes = encoded(ConfigV2{.counter = 0, .name = "two", .gains = {2.0f}});
    ConfigV3 out{};
    uint32_t version = 0;
    std::error_code ec;
    CHECK(decode_full(bytes, out, version, ec));
    CHECK(version == 2);
    CHECK(out.name == "two (migrated)");
    CHECK(out.gains == std::vector<float>({2.0f}));
    CHECK(out.enabled == false); // counter 0
  }
  {
    // a list that dropped v1 support reports unknown_version for a v1 file
    auto bytes = encoded(ConfigV1{.counter = 7, .name = "one"});
    std::span<const uint8_t> payload;
    pd::Header header;
    std::error_code ec;
    CHECK(pd::parse(bytes, header, payload, ec));
    ConfigV3 out{};
    CHECK(!Recent::decode(header, payload, out, ec));
    CHECK(ec == espp::PersistentDataErrc::unknown_version);
    // and the single-version list behaves the same for a v2 file
    bytes = encoded(ConfigV2{.counter = 1, .name = "two"});
    CHECK(pd::parse(bytes, header, payload, ec));
    CHECK(!Single::decode(header, payload, out, ec));
    CHECK(ec == espp::PersistentDataErrc::unknown_version);
  }
}

static void test_newer_version_rejected() {
  std::printf("file from newer firmware\n");
  auto bytes = encoded(ConfigV4{.name = "future", .flags = 3});
  ConfigV3 out{.name = "untouched"};
  uint32_t version = 0;
  std::error_code ec;
  CHECK(!decode_full(bytes, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::unknown_version);
  CHECK(version == 4); // parse() reports what the header says
  CHECK(out.name == "untouched");
}

static void test_same_version_different_layout() {
  std::printf("same version number, different layout\n");
  // the header's layout fingerprint catches a type that reuses a version number
  auto bytes = encoded(OtherV3{.a = 1, .b = 2});
  ConfigV3 out{};
  uint32_t version = 0;
  std::error_code ec;
  CHECK(!decode_full(bytes, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::layout_mismatch);
  // a forged fingerprint gets past the header check and fails in alpaca instead
  pd::put_u32(bytes.data() + 8, pd::layout::fingerprint<ConfigV3>());
  uint32_t crc = crc32_fast(bytes.data(), 16);
  crc = crc32_fast(bytes.data() + pd::HEADER_SIZE, bytes.size() - pd::HEADER_SIZE, crc);
  pd::put_u32(bytes.data() + 16, crc);
  CHECK(!decode_full(bytes, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::deserialize_failed);
}

static void test_layout_fingerprint() {
  std::printf("layout fingerprint grammar\n");
  using ft = alpaca::detail::field_type;
  auto c = [](ft f) { return static_cast<uint8_t>(f); };
  // ConfigV3 { string name; vector<float> gains; Limits{float, float} limits; bool enabled; }
  std::vector<uint8_t> expected{
      c(ft::struct_), 4, 0, c(ft::string),  c(ft::vector),  c(ft::float32),
      c(ft::struct_), 2, 0, c(ft::float32), c(ft::float32), c(ft::bool_)};
  CHECK(pd::layout::describe<ConfigV3>() == expected);
  CHECK(pd::layout::fingerprint<ConfigV3>() == crc32_fast(expected.data(), expected.size()));
  // nothing platform-dependent went in: the description is exactly the grammar
  CHECK(pd::layout::describe<ConfigV3>().size() == expected.size());
  // different layouts, different fingerprints; same layout, same fingerprint
  CHECK(pd::layout::fingerprint<ConfigV1>() != pd::layout::fingerprint<ConfigV2>());
  CHECK(pd::layout::fingerprint<ConfigV3>() != pd::layout::fingerprint<OtherV3>());
  struct SameAsV1 {
    uint32_t counter{0};
    std::string name{};
  };
  CHECK(pd::layout::fingerprint<SameAsV1>() == pd::layout::fingerprint<ConfigV1>());
  // enums carry their underlying type, arrays their length
  enum class Small : uint8_t { a };
  enum class Wide : uint32_t { a };
  struct E1 {
    Small s{};
  };
  struct E2 {
    Wide w{};
  };
  CHECK(pd::layout::fingerprint<E1>() != pd::layout::fingerprint<E2>());
  CHECK(pd::layout::describe<E1>() ==
        std::vector<uint8_t>({c(ft::struct_), 1, 0, c(ft::enum_class), c(ft::uint8)}));
  struct A3 {
    std::array<uint8_t, 3> v{};
  };
  struct A4 {
    std::array<uint8_t, 4> v{};
  };
  CHECK(pd::layout::fingerprint<A3>() != pd::layout::fingerprint<A4>());
  CHECK(pd::layout::describe<A3>() ==
        std::vector<uint8_t>({c(ft::struct_), 1, 0, c(ft::array), 3, 0, 0, 0, c(ft::uint8)}));
}

static void test_corruption() {
  std::printf("corrupt / truncated / foreign files\n");
  ConfigV3 v3{.name = "abc", .gains = {0.5f}, .limits = {}, .enabled = true};
  auto good = encoded(v3);
  ConfigV3 out{};
  uint32_t version = 0;
  std::error_code ec;

  // empty file
  CHECK(!decode_full({}, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_header);
  // shorter than a header
  CHECK(!decode_full(std::vector<uint8_t>(good.begin(), good.begin() + 10), out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_header);
  // wrong magic
  auto bad = good;
  bad[0] = 'X';
  CHECK(!decode_full(bad, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_header);
  // truncated payload (a reset mid-write without the atomic rename)
  CHECK(!decode_full(std::vector<uint8_t>(good.begin(), good.end() - 1), out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_header);
  // trailing junk
  bad = good;
  bad.push_back(0);
  CHECK(!decode_full(bad, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_header);
  // one flipped payload bit
  bad = good;
  bad[pd::HEADER_SIZE + 2] ^= 0x01;
  CHECK(!decode_full(bad, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_checksum);
  // a flipped version byte is caught by the crc too, so the version field can be trusted
  bad = good;
  bad[4] ^= 0x01;
  CHECK(!decode_full(bad, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_checksum);
  // a length that claims more than the file holds
  bad = good;
  bad[12] += 1;
  CHECK(!decode_full(bad, out, version, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_header);
}

static void test_files() {
  std::printf("file helpers (atomic save, load, remove)\n");
  namespace fs = std::filesystem;
  fs::path dir = fs::temp_directory_path() / "espp_persistent_data_test";
  fs::remove_all(dir);
  fs::create_directories(dir);
  fs::path path = dir / "config.bin";
  fs::path tmp = dir / "config.bin.tmp";
  std::error_code ec;

  // nothing there yet
  CHECK(!pd::file_exists(path));
  ConfigV3 out{};
  pd::LoadInfo info;
  CHECK(!pd::load_file<Full>(path, 1024, out, info, ec));
  CHECK(ec == espp::PersistentDataErrc::file_not_found);
  // removing a missing file is fine
  CHECK(pd::remove_file(path, ec));
  CHECK(!ec);

  // save + load the current version
  ConfigV3 v3{.name = "disk", .gains = {1, 2, 3}, .limits = {.min = 0, .max = 9}, .enabled = true};
  CHECK(pd::save_file<Full>(path, v3, ec));
  CHECK(!ec);
  CHECK(pd::file_exists(path));
  CHECK(!fs::exists(tmp)); // the temporary was renamed away
  CHECK(pd::load_file<Full>(path, 1024, out, info, ec));
  CHECK(!ec);
  CHECK(out.name == "disk" && out.gains.size() == 3 && out.limits.max == 9);
  CHECK(info.file_version == 3 && !info.migrated);

  // a save replaces the existing file (rename over it) and keeps no temporary
  v3.name = "disk2";
  CHECK(pd::save_file<Full>(path, v3, ec));
  CHECK(!fs::exists(tmp));
  CHECK(pd::load_file<Full>(path, 1024, out, info, ec));
  CHECK(out.name == "disk2");

  // an old-version file on disk is migrated on load
  CHECK(pd::save_file<pd::VersionList<ConfigV1>>(path, ConfigV1{.counter = 1, .name = "old"}, ec));
  CHECK(pd::load_file<Full>(path, 1024, out, info, ec));
  CHECK(!ec);
  CHECK(info.file_version == 1 && info.migrated);
  CHECK(out.name == "old (migrated)" && out.enabled);

  // a leftover temporary from an interrupted save does not affect the load...
  {
    FILE *f = std::fopen(tmp.c_str(), "wb");
    CHECK(f != nullptr);
    if (f) {
      std::fputs("partial", f);
      std::fclose(f);
    }
  }
  CHECK(pd::load_file<Full>(path, 1024, out, info, ec));
  CHECK(out.name == "old (migrated)");
  // ...and the next save overwrites it
  CHECK(pd::save_file<Full>(path, v3, ec));
  CHECK(!fs::exists(tmp));

  // the size cap rejects before reading
  CHECK(!pd::load_file<Full>(path, 8, out, info, ec));
  CHECK(ec == espp::PersistentDataErrc::file_too_large);

  // a corrupt file on disk
  {
    std::vector<uint8_t> bytes;
    CHECK(pd::read_file(path, 1024, bytes, ec));
    bytes[pd::HEADER_SIZE] ^= 0xFF;
    CHECK(pd::write_file_atomic(path, bytes, ec));
  }
  CHECK(!pd::load_file<Full>(path, 1024, out, info, ec));
  CHECK(ec == espp::PersistentDataErrc::bad_checksum);

  // remove (also clears a temporary)
  {
    FILE *f = std::fopen(tmp.c_str(), "wb");
    CHECK(f != nullptr);
    if (f)
      std::fclose(f);
  }
  CHECK(pd::remove_file(path, ec));
  CHECK(!pd::file_exists(path));
  CHECK(!fs::exists(tmp));

  // a save into a directory that does not exist reports io_error
  CHECK(!pd::save_file<Full>(dir / "missing" / "config.bin", v3, ec));
  CHECK(ec == espp::PersistentDataErrc::io_error);

  fs::remove_all(dir);
}

int main() {
  test_header_layout();
  test_round_trip_current();
  test_migration_chain();
  test_newer_version_rejected();
  test_same_version_different_layout();
  test_layout_fingerprint();
  test_corruption();
  test_files();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
