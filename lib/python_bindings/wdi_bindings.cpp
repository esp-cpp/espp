// Hand-written pybind11 bindings for the espp `wdi` (Wheelchair Digital
// Interface) protocol core. Header-only and dependency-free, so it binds cleanly
// on the host; kept out of the generated pybind_espp.cpp (see
// dispatcher_bindings.cpp) so regeneration never clobbers it.
//
// Exposes espp.wdi.{ReportId, ControlBit, FeedbackBit, ManufacturerId,
//                   ControlReport, FeedbackReport, HostUuid} + size constants.
// This is enough to build and test a WDI *host* (the wheelchair side) from Python
// -- parse Control reports and build Feedback reports -- and to interop-test
// against the on-device peripheral. The full WdiDevice role class is available in
// the C++ host library (wdi.hpp).

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "wdi.hpp"

namespace py = pybind11;
namespace wdi = espp::wdi;

namespace {
std::span<const uint8_t> as_span(const std::string &s) {
  return {reinterpret_cast<const uint8_t *>(s.data()), s.size()};
}
template <size_t N> py::bytes to_bytes(const std::array<uint8_t, N> &a) {
  return py::bytes(reinterpret_cast<const char *>(a.data()), a.size());
}
} // namespace

void py_init_wdi(py::module &m) {
  auto wm = m.def_submodule("wdi", "Wheelchair Digital Interface (Open-Mobility-Hub "
                                   "Wheelchair HID) protocol core.");

  wm.attr("kControlSize") = wdi::kControlSize;
  wm.attr("kFeedbackSize") = wdi::kFeedbackSize;
  wm.attr("kRequestFeedbackSize") = wdi::kRequestFeedbackSize;
  wm.attr("kKeepaliveSize") = wdi::kKeepaliveSize;
  wm.attr("kKeepaliveResponseSize") = wdi::kKeepaliveResponseSize;
  wm.attr("kTriggerValue") = wdi::kTriggerValue;
  wm.attr("kAppKeepaliveIntervalMs") = wdi::kAppKeepaliveIntervalMs;
  wm.attr("kHostKeepaliveWindowMs") = wdi::kHostKeepaliveWindowMs;
  wm.attr("kHostMissedWindowsToDisconnect") = wdi::kHostMissedWindowsToDisconnect;

  py::enum_<wdi::ReportId>(wm, "ReportId", "HID report ids (device-POV direction).")
      .value("Control", wdi::ReportId::Control)
      .value("Feedback", wdi::ReportId::Feedback)
      .value("RequestFeedback", wdi::ReportId::RequestFeedback)
      .value("Keepalive", wdi::ReportId::Keepalive)
      .value("KeepaliveResponse", wdi::ReportId::KeepaliveResponse);

  py::enum_<wdi::ControlBit>(wm, "ControlBit", "Bits of the Control report's Standard1 bitfield.")
      .value("Modifier", wdi::ControlBit::Modifier)
      .value("Stop", wdi::ControlBit::Stop)
      .value("DriveEnable", wdi::ControlBit::DriveEnable)
      .value("CycleProfile", wdi::ControlBit::CycleProfile)
      .value("Hazards", wdi::ControlBit::Hazards)
      .value("CycleMode", wdi::ControlBit::CycleMode)
      .value("SpeedDown", wdi::ControlBit::SpeedDown)
      .value("SpeedUp", wdi::ControlBit::SpeedUp)
      .value("LeftBlinker", wdi::ControlBit::LeftBlinker)
      .value("RightBlinker", wdi::ControlBit::RightBlinker)
      .value("Menu", wdi::ControlBit::Menu)
      .value("ProfileUp", wdi::ControlBit::ProfileUp)
      .value("DriveDisable", wdi::ControlBit::DriveDisable)
      .value("Headlights", wdi::ControlBit::Headlights)
      .value("Horn", wdi::ControlBit::Horn)
      .value("ProfileDown", wdi::ControlBit::ProfileDown)
      .value("Memory1", wdi::ControlBit::Memory1)
      .value("Memory2", wdi::ControlBit::Memory2)
      .value("Memory3", wdi::ControlBit::Memory3)
      .value("Memory4", wdi::ControlBit::Memory4)
      .value("Memory5", wdi::ControlBit::Memory5)
      .value("Memory6", wdi::ControlBit::Memory6)
      .value("MemoryHome", wdi::ControlBit::MemoryHome)
      .value("Tilt", wdi::ControlBit::Tilt)
      .value("Recline", wdi::ControlBit::Recline)
      .value("Legs", wdi::ControlBit::Legs)
      .value("Elevate", wdi::ControlBit::Elevate)
      .value("Footplates", wdi::ControlBit::Footplates)
      .value("Stand", wdi::ControlBit::Stand);

  py::enum_<wdi::FeedbackBit>(wm, "FeedbackBit", "Bits of the Feedback report's Standard bitfield.")
      .value("DriveDisabled", wdi::FeedbackBit::DriveDisabled)
      .value("DriveEnabled", wdi::FeedbackBit::DriveEnabled)
      .value("ModeDrive", wdi::FeedbackBit::ModeDrive)
      .value("ModeSeating", wdi::FeedbackBit::ModeSeating)
      .value("LeftBlinkerOff", wdi::FeedbackBit::LeftBlinkerOff)
      .value("LeftBlinkerOn", wdi::FeedbackBit::LeftBlinkerOn)
      .value("RightBlinkerOff", wdi::FeedbackBit::RightBlinkerOff)
      .value("RightBlinkerOn", wdi::FeedbackBit::RightBlinkerOn)
      .value("HeadlightsOff", wdi::FeedbackBit::HeadlightsOff)
      .value("HeadlightsOn", wdi::FeedbackBit::HeadlightsOn)
      .value("HazardsOff", wdi::FeedbackBit::HazardsOff)
      .value("HazardsOn", wdi::FeedbackBit::HazardsOn)
      .value("NoMovementRestriction", wdi::FeedbackBit::NoMovementRestriction)
      .value("LimitedSpeed", wdi::FeedbackBit::LimitedSpeed)
      .value("NoMovement", wdi::FeedbackBit::NoMovement);

  py::enum_<wdi::ManufacturerId>(wm, "ManufacturerId", "Registered WDI manufacturer ids.")
      .value("Unknown", wdi::ManufacturerId::Unknown)
      .value("LuciMobility", wdi::ManufacturerId::LuciMobility)
      .value("LifeDrive", wdi::ManufacturerId::LifeDrive);

  py::class_<wdi::ControlReport>(wm, "ControlReport",
                                 "Control report (0x01): joystick + control flags.")
      .def(py::init<>())
      .def_readwrite("x", &wdi::ControlReport::x)
      .def_readwrite("y", &wdi::ControlReport::y)
      .def_readwrite("standard1", &wdi::ControlReport::standard1)
      .def_readwrite("standard2", &wdi::ControlReport::standard2)
      .def_readwrite("vendor1", &wdi::ControlReport::vendor1)
      .def_readwrite("vendor2", &wdi::ControlReport::vendor2)
      .def("has", &wdi::ControlReport::has, py::arg("bit"))
      .def("set", &wdi::ControlReport::set, py::arg("bit"), py::arg("on") = true)
      .def("is_release", &wdi::ControlReport::is_release)
      .def("serialize", [](const wdi::ControlReport &c) { return to_bytes(c.serialize()); })
      .def_static(
          "parse", [](const std::string &b) { return wdi::ControlReport::parse(as_span(b)); },
          py::arg("data"));

  py::class_<wdi::FeedbackReport>(wm, "FeedbackReport",
                                  "Feedback report (0x02): status + speed/velocity/odometer.")
      .def(py::init<>())
      .def_readwrite("standard", &wdi::FeedbackReport::standard)
      .def_readwrite("vendor1", &wdi::FeedbackReport::vendor1)
      .def_readwrite("vendor2", &wdi::FeedbackReport::vendor2)
      .def_readwrite("speed", &wdi::FeedbackReport::speed)
      .def_readwrite("profile", &wdi::FeedbackReport::profile)
      .def_readwrite("velocity_whole", &wdi::FeedbackReport::velocity_whole)
      .def_readwrite("velocity_tenths", &wdi::FeedbackReport::velocity_tenths)
      .def_readwrite("odometer", &wdi::FeedbackReport::odometer)
      .def("has", &wdi::FeedbackReport::has, py::arg("bit"))
      .def("set", &wdi::FeedbackReport::set, py::arg("bit"), py::arg("on") = true)
      .def("velocity_mph", &wdi::FeedbackReport::velocity_mph)
      .def("serialize", [](const wdi::FeedbackReport &f) { return to_bytes(f.serialize()); })
      .def_static(
          "parse", [](const std::string &b) { return wdi::FeedbackReport::parse(as_span(b)); },
          py::arg("data"));

  py::class_<wdi::HostUuid>(wm, "HostUuid",
                            "The host's 128-bit identity (Keepalive Response 0x05).")
      .def(py::init<>())
      .def("manufacturer_id", &wdi::HostUuid::manufacturer_id)
      .def("serialize", [](const wdi::HostUuid &u) { return to_bytes(u.serialize()); })
      .def("bytes", [](const wdi::HostUuid &u) { return to_bytes(u.bytes); })
      .def_static(
          "parse", [](const std::string &b) { return wdi::HostUuid::parse(as_span(b)); },
          py::arg("data"));
}
