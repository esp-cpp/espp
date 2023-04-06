#include <chrono>
#include <thread>
#include <vector>

#include "format.hpp"
#include "serialization.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting serialization example!\n");
    //! [serialization example]
    struct MyStruct {
      uint8_t value; // NOTE: you cannot use int, you must use strongly sized types.
    };
    std::error_code ec;
    MyStruct object{5};
    {
      uint8_t buffer[10];
      auto bytes_written = alpaca::serialize(object, buffer);
      fmt::print("Serialized {}B to c-array (alpaca)\n", bytes_written);
      auto new_object = alpaca::deserialize<MyStruct>(buffer, bytes_written, ec);
      if (!ec && new_object.value == object.value) {
        fmt::print("Deserialized successfully!\n");
      } else {
        fmt::print("Deserialization failed: {}\n", ec.message());
      }
    }
    {
      uint8_t buffer[10];
      auto bytes_written = espp::serialize(object, buffer);
      fmt::print("Serialized {}B to c-array (espp wrapper)\n", bytes_written);
      auto new_object = espp::deserialize<MyStruct>(buffer, bytes_written, ec);
      if (!ec && new_object.value == object.value) {
        fmt::print("Deserialized successfully!\n");
      } else {
        fmt::print("Deserialization failed: {}\n", ec.message());
      }
    }
    {
      std::array<uint8_t, 5> buffer;
      auto bytes_written = espp::serialize(object, buffer);
      fmt::print("Serialized {}B to std::array\n", bytes_written);
      auto new_object = espp::deserialize<MyStruct>(buffer, ec);
      if (!ec && new_object.value == object.value) {
        fmt::print("Deserialized successfully!\n");
      } else {
        fmt::print("Deserialization failed: {}\n", ec.message());
      }
    }
    {
      std::vector<uint8_t> buffer;
      auto bytes_written = espp::serialize(object, buffer);
      fmt::print("Serialized {}B to std::vector\n", bytes_written);
      auto new_object = espp::deserialize<MyStruct>(buffer, ec);
      if (!ec && new_object.value == object.value) {
        fmt::print("Deserialized successfully!\n");
      } else {
        fmt::print("Deserialization failed: {}\n", ec.message());
      }
    }
    {
      std::array<uint8_t, 15> buffer;
      constexpr auto OPTIONS = alpaca::options::fixed_length_encoding;
      auto bytes_written = espp::serialize<OPTIONS>(object, buffer);
      fmt::print("Serialized {}B to std::arary with custom options\n", bytes_written);
      auto new_object = espp::deserialize<OPTIONS, MyStruct>(buffer, ec);
      if (!ec && new_object.value == object.value) {
        fmt::print("Deserialized successfully!\n");
      } else {
        fmt::print("Deserialization failed: {}\n", ec.message());
      }
    }
    //! [serialization example]
  }

  {
    fmt::print("Starting complex serialization example!\n");
    //! [complex serialization example]
    struct MyStruct {
      uint8_t value;
      std::vector<uint32_t> data;
      std::string name;
    };
    std::error_code ec;
    MyStruct object{42, {1, 3, 3, 7}, "the meaning of life"};
    std::vector<uint8_t> buffer;
    auto bytes_written = espp::serialize(object, buffer);
    fmt::print("Serialized {}B to std::vector\n", bytes_written);
    auto new_object = espp::deserialize<MyStruct>(buffer, ec);
    if (!ec && new_object.value == object.value) {
      fmt::print("Deserialized successfully!\n");
      fmt::print("\tvalue: {}\n"
                 "\tdata:  {}\n"
                 "\tname:  {}\n",
                 new_object.value, new_object.data, new_object.name);
    } else {
      fmt::print("Deserialization failed: {}\n", ec.message());
    }
    //! [complex serialization example]
  }

  fmt::print("Serialization example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
