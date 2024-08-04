#include <chrono>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "espp.hpp"

namespace py = pybind11;
using namespace espp;

PYBIND11_MODULE(espp, m) {

  // Logger Verbosity
  py::enum_<Logger::Verbosity>(m, "Verbosity")
      .value("DEBUG", Logger::Verbosity::DEBUG)
      .value("INFO", Logger::Verbosity::INFO)
      .value("WARN", Logger::Verbosity::WARN)
      .value("ERROR", Logger::Verbosity::ERROR)
      .value("NONE", Logger::Verbosity::NONE);

  // Task
  // NOTE: we cannot use the Task::Config since its callback function has a
  //       std::mutex and std::condition_variable which pybind does not support.
  //       Therefore we use the SimpleConfig, whose callback function has no
  //       arguments and return bool
  py::class_<Task::BaseConfig>(m, "TaskBaseConfig")
      .def(py::init<const std::string &, size_t, size_t, int>(), py::arg("name"),
           py::arg("stack_size") = 4 * 1024, py::arg("priority") = 0, py::arg("core_id") = -1);
  py::class_<Task::Config>(m, "TaskConfig")
      .def(py::init<const std::string &, Task::callback_fn, size_t, size_t, int,
                    Logger::Verbosity>(),
           py::arg("name"), py::arg("callback"), py::arg("stack_size") = 4 * 1024,
           py::arg("priority") = 0, py::arg("core_id") = -1,
           py::arg("verbosity") = Logger::Verbosity::WARN);
  py::class_<Task::SimpleConfig>(m, "TaskSimpleConfig")
      .def(py::init<Task::simple_callback_fn, Task::BaseConfig, Logger::Verbosity>(),
           py::arg("callback"), py::arg("task_config"),
           py::arg("verbosity") = Logger::Verbosity::WARN);
  py::class_<Task::AdvancedConfig>(m, "TaskAdvancedConfig")
      .def(py::init<Task::callback_fn, Task::BaseConfig, Logger::Verbosity>(), py::arg("callback"),
           py::arg("task_config"), py::arg("verbosity") = Logger::Verbosity::WARN);

  py::class_<Task>(m, "Task")
      .def(py::init<const Task::Config &>())
      .def(py::init<const Task::SimpleConfig &>())
      .def(py::init<const Task::AdvancedConfig &>())
      .def("start", &Task::start)
      .def("stop", &Task::stop)
      .def("is_started", &Task::is_started)
      .def("is_running", &Task::is_running);

  // Timer
  py::class_<Timer::Config>(m, "TimerConfig")
      .def(py::init<const std::string &, std::chrono::duration<float>, std::chrono::duration<float>,
                    Timer::callback_fn, bool, size_t, size_t, int, Logger::Verbosity>(),
           py::arg("name"), py::arg("period"), py::arg("delay") = std::chrono::duration<float>(0),
           py::arg("callback"), py::arg("auto_start") = true, py::arg("stack_size") = 4 * 1024,
           py::arg("priority") = 0, py::arg("core_id") = -1,
           py::arg("verbosity") = Logger::Verbosity::WARN);

  py::class_<Timer>(m, "Timer")
      .def(py::init<const Timer::Config &>())
      .def("start", py::overload_cast<>(&Timer::start))
      .def("start", py::overload_cast<std::chrono::duration<float>>(&Timer::start))
      .def("cancel", &Timer::cancel)
      .def("is_running", &Timer::is_running);

  // Socket Info
  py::class_<Socket::Info>(m, "SocketInfo")
      .def(py::init<>())
      .def_readwrite("address", &Socket::Info::address)
      .def_readwrite("port", &Socket::Info::port);

  // UDP Socket Config
  py::class_<UdpSocket::Config>(m, "UdpSocketConfig")
      .def(py::init<Logger::Verbosity>(), py::arg("log_level") = Logger::Verbosity::WARN);

  // UDP Socket SendConfig
  py::class_<UdpSocket::SendConfig>(m, "UdpSendConfig")
      .def(py::init<std::string, size_t, bool, bool, size_t, Socket::response_callback_fn,
                    std::chrono::duration<float>>(),
           py::arg("ip_address"), py::arg("port"), py::arg("is_multicast_endpoint") = false,
           py::arg("wait_for_response") = false, py::arg("response_size") = 0,
           py::arg("on_response_callback") = nullptr, py::arg("response_timeout") = 0.5f);

  // UDP Socket ReceiveConfig
  py::class_<UdpSocket::ReceiveConfig>(m, "UdpReceiveConfig")
      .def(py::init<size_t, size_t, bool, std::string, Socket::receive_callback_fn>(),
           py::arg("port"), py::arg("buffer_size"), py::arg("is_multicast_endpoint") = false,
           py::arg("multicast_group") = "", py::arg("on_receive_callback") = nullptr);

  // UDP Socket
  py::class_<UdpSocket>(m, "UdpSocket")
      .def(py::init<const UdpSocket::Config &>(), py::arg("config"))
      .def("send", py::overload_cast<const std::vector<uint8_t> &, const UdpSocket::SendConfig &>(
                       &UdpSocket::send))
      .def("send",
           py::overload_cast<std::string_view, const UdpSocket::SendConfig &>(&UdpSocket::send))
      .def("receive", &UdpSocket::receive)
      .def("start_receiving", &UdpSocket::start_receiving);

  // TCP Socket Config
  py::class_<TcpSocket::Config>(m, "TcpSocketConfig")
      .def(py::init<Logger::Verbosity>(), py::arg("log_level") = Logger::Verbosity::WARN);

  // TCP Socket ConnectConfig
  py::class_<TcpSocket::ConnectConfig>(m, "TcpConnectConfig")
      .def(py::init<std::string, size_t>(), py::arg("ip_address"), py::arg("port"));

  // espp::detail TcpTransmitConfig
  py::class_<detail::TcpTransmitConfig>(m, "TcpTransmitConfig")
      .def(py::init<bool, size_t, Socket::response_callback_fn, std::chrono::duration<float>>(),
           py::arg("wait_for_response") = false, py::arg("response_size") = 0,
           py::arg("on_response_callback") = nullptr, py::arg("response_timeout") = 0.5f);

  // TCP Socket
  py::class_<TcpSocket>(m, "TcpSocket")
      .def(py::init<const TcpSocket::Config &>(), py::arg("config"))
      .def("reinit", &TcpSocket::reinit)
      .def("close", &TcpSocket::close)
      .def("is_connected", &TcpSocket::is_connected)
      .def("connect", &TcpSocket::connect)
      .def("bind", &TcpSocket::bind)
      .def("listen", &TcpSocket::listen)
      .def("accept", &TcpSocket::accept)
      .def("get_remote_info", &TcpSocket::get_remote_info)
      .def("transmit",
           py::overload_cast<const std::vector<uint8_t> &, const detail::TcpTransmitConfig &>(
               &TcpSocket::transmit))
      .def("transmit",
           py::overload_cast<const std::vector<char> &, const detail::TcpTransmitConfig &>(
               &TcpSocket::transmit))
      .def("transmit", py::overload_cast<std::string_view, const detail::TcpTransmitConfig &>(
                           &TcpSocket::transmit))
      .def("receive", py::overload_cast<std::vector<uint8_t> &, size_t>(&TcpSocket::receive))
      .def("receive", py::overload_cast<uint8_t *, size_t>(&TcpSocket::receive));
}
