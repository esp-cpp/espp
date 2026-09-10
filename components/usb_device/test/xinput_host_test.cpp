// Host-buildable unit tests for the espp X-Input (Xbox 360) helpers in
// include/xinput.hpp: the 20-byte input-report packing and the interface + XID +
// endpoint descriptor builder. No ESP-IDF / TinyUSB headers required.
//
/* Build & run (a block comment so the line-continuation doesn't trip -Wcomment):
     c++ -std=c++20 -Wall -Wextra -Werror -I components/usb_device/include \
         components/usb_device/test/xinput_host_test.cpp -o test && ./test        */

#include <cstdio>

#include "xinput.hpp"

using namespace espp::xinput;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL (%s:%d): %s\n", __FILE__, __LINE__, #cond);                                \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static int16_t rd16(const std::array<uint8_t, kReportInSize> &r, size_t i) {
  return static_cast<int16_t>(r[i] | (r[i + 1] << 8));
}

int main() {
  // --- input report packing ---
  GamepadState s;
  s.set(Button::A, true);
  s.set(Button::DpadUp, true);
  CHECK(s.get(Button::A) && s.get(Button::DpadUp) && !s.get(Button::B));
  s.left_trigger = 200;
  s.right_trigger = 50;
  s.lx = 1000;
  s.ly = -2000;
  s.rx = 32767;
  s.ry = -32768;
  const auto r = s.report();
  CHECK(r.size() == 20);
  CHECK(r[0] == 0x00 && r[1] == 0x14); // type + length
  CHECK(r[2] == 0x01);                 // dpad-up -> byte2 bit0
  CHECK(r[3] == 0x10);                 // A -> byte3 bit4
  CHECK(r[4] == 200 && r[5] == 50);    // triggers
  CHECK(rd16(r, 6) == 1000 && rd16(r, 8) == -2000);
  CHECK(rd16(r, 10) == 32767 && rd16(r, 12) == -32768);
  for (size_t i = 14; i < 20; ++i)
    CHECK(r[i] == 0);

  // clearing a button
  s.set(Button::A, false);
  CHECK(s.report()[3] == 0x00);

  // --- descriptor builder (itf 3, string 5, endpoint number 2 -> IN 0x82, OUT 0x02) ---
  const auto d = interface_descriptor(3, 5, 2);
  CHECK(d.size() == kInterfaceDescriptorLen && d.size() == 40);
  // interface descriptor
  CHECK(d[0] == 0x09 && d[1] == 0x04 && d[2] == 3 && d[4] == 0x02);
  CHECK(d[5] == kInterfaceClass && d[6] == kInterfaceSubClass && d[7] == kInterfaceProtocol);
  CHECK(d[8] == 5);
  // XID blob, with the IN endpoint address patched in
  CHECK(d[9] == 0x11 && d[10] == 0x21 && d[15] == 0x82 && d[16] == 0x14 && d[23] == 0x08);
  // IN endpoint (interrupt, size 32)
  CHECK(d[26] == 0x07 && d[27] == 0x05 && d[28] == 0x82 && d[29] == 0x03 && d[30] == kEpSize);
  // OUT endpoint
  CHECK(d[33] == 0x07 && d[34] == 0x05 && d[35] == 0x02 && d[36] == 0x03 && d[37] == kEpSize);

  if (g_failures == 0)
    std::printf("all xinput host tests passed\n");
  else
    std::printf("%d FAILURE(S)\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
