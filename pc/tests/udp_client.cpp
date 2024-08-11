#include "espp.hpp"

using namespace std::chrono_literals;

static auto start = std::chrono::high_resolution_clock::now();
static auto client = espp::UdpSocket({.log_level = espp::Logger::Verbosity::DEBUG});

bool task_func() {
  auto send_config = espp::UdpSocket::SendConfig{.ip_address = "127.0.01", .port = 5555};
  fmt::print("Sending message\n");
  client.send("Hello world\n", send_config);
  std::this_thread::sleep_for(500ms);
  return false; // don't want to stop the task
}

int main() {
  auto task = espp::Task(espp::Task::SimpleConfig{
      .callback = task_func, .task_config = espp::Task::BaseConfig{.name = "test_task"}});
  task.start();

  std::this_thread::sleep_for(5s);

  return 0;
}
