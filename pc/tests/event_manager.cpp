// Host test for espp::EventManager: the pub/sub API plus the concurrency cases
// that used to hang, crash, or corrupt memory (removing the last subscriber
// while its callback ran, a callback removing itself, subscribers added or
// removed during a delivery, publish racing the last removal, a re-subscribe
// racing the last removal, new topics racing publish, destruction with live
// subscribers). Runs under TSan and ASan in host_sanitizers.yml.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "event_manager.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

namespace {
// The constructor is protected (EventManager is a singleton); a derived class
// gives each case a fresh instance, and lets a case destroy one.
struct TestEventManager : public espp::EventManager {
  TestEventManager() { set_log_level(espp::Logger::Verbosity::ERROR); }
};

using Data = std::vector<uint8_t>;

struct Latch {
  std::mutex m;
  std::condition_variable cv;
  bool open = false;
  void release() {
    {
      std::lock_guard<std::mutex> lk(m);
      open = true;
    }
    cv.notify_all();
  }
  bool wait(std::chrono::milliseconds timeout = 10s) {
    std::unique_lock<std::mutex> lk(m);
    return cv.wait_for(lk, timeout, [this] { return open; });
  }
};

template <typename Predicate>
bool wait_until(Predicate &&pred, std::chrono::milliseconds timeout = 10s) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return pred();
}

// A subscriber callback that always throws.
void throw_expected(const Data &) { throw std::runtime_error("expected: a callback that throws"); }

// Run fn on another thread; false if it hasn't returned within the timeout (the
// thread is then left running and the test fails).
template <typename Fn> bool returns_within(Fn &&fn, std::chrono::milliseconds timeout) {
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread([fn = std::forward<Fn>(fn), done]() mutable {
    fn();
    *done = true;
  }).detach();
  return wait_until([&] { return done->load(); }, timeout);
}
} // namespace

int main() {
  espp::Logger logger({.tag = "EventManager Test", .level = espp::Logger::Verbosity::INFO});
  int total_passed = 0;
  int total_tests = 0;
  auto check = [&](bool condition, const std::string &desc) -> bool {
    ++total_tests;
    if (condition) {
      ++total_passed;
      logger.info("  PASS: {}", desc);
    } else {
      logger.error("  FAIL: {}", desc);
    }
    return condition;
  };

  // A hang (the old remove_subscriber join) fails the test instead of hanging CI.
  std::thread([] {
    std::this_thread::sleep_for(240s);
    std::fprintf(stderr, "FAIL: event_manager test did not finish within 240 s\n");
    std::_Exit(1);
  }).detach();

  logger.info("--- registration and delivery ---");
  {
    TestEventManager em;
    const Data payload{1, 2, 3};
    check(!em.publish("t", payload), "publish without subscribers returns false");
    std::mutex m;
    std::vector<std::string> calls;
    auto record = [&](const std::string &who) {
      return [&, who](const Data &data) {
        std::lock_guard<std::mutex> lk(m);
        calls.push_back(who + ":" + std::to_string(data.size()));
      };
    };
    check(em.add_subscriber("t", "a", record("a")), "add_subscriber");
    check(!em.add_subscriber("t", "a", record("a")), "duplicate add_subscriber returns false");
    check(!em.add_subscriber("t", "empty", espp::EventManager::event_callback_fn{}),
          "add_subscriber with an empty callback returns false");
    check(em.add_subscriber("t", "b", record("b"),
                            espp::Task::BaseConfig{.name = "t sub", .stack_size_bytes = 8192}),
          "add_subscriber with a task config");
    check(em.publish("t", payload), "publish with subscribers returns true");
    check(em.publish("t", Data{9}), "second publish");
    check(wait_until([&] {
            std::lock_guard<std::mutex> lk(m);
            return calls.size() == 4;
          }),
          "both events delivered to both subscribers");
    {
      std::lock_guard<std::mutex> lk(m);
      check(calls == std::vector<std::string>{"a:3", "b:3", "a:1", "b:1"},
            "delivered in publish order, subscribers in registration order");
    }
    check(!em.remove_subscriber("t", "nobody"), "remove_subscriber of an unknown component");
    check(!em.remove_subscriber("nowhere", "a"), "remove_subscriber of an unknown topic");
    check(em.add_publisher("t", "p"), "add_publisher");
    check(!em.add_publisher("t", "p"), "duplicate add_publisher returns false");
    check(em.remove_publisher("t", "p"), "remove_publisher");
    check(!em.remove_publisher("t", "p"), "remove_publisher twice returns false");
    check(em.remove_subscriber("t", "a") && em.remove_subscriber("t", "b"),
          "remove both subscribers");
    check(!em.publish("t", payload), "publish after the last subscriber left returns false");
  }

  logger.info("--- a callback removes its own subscription ---");
  {
    TestEventManager em;
    std::atomic<int> calls{0};
    std::atomic<bool> removed{false};
    em.add_subscriber("once", "one-shot", [&](const Data &) {
      calls++;
      removed = em.remove_subscriber("once", "one-shot");
    });
    em.publish("once", Data{1});
    check(wait_until([&] { return removed.load(); }),
          "remove_subscriber from its own callback returns true (used to std::terminate)");
    check(!em.publish("once", Data{2}), "the topic is gone afterwards");
    std::atomic<int> again{0};
    check(em.add_subscriber("once", "again", [&](const Data &) { again++; }),
          "the topic can be subscribed to again");
    em.publish("once", Data{3});
    check(wait_until([&] { return again == 1; }), "and delivers to the new subscriber");
    check(calls == 1, "the one-shot callback ran exactly once");
  }

  logger.info("--- a callback removes its own subscription, others remain ---");
  {
    TestEventManager em;
    std::atomic<int> a_calls{0}, b_calls{0};
    em.add_subscriber("t", "a", [&](const Data &) {
      a_calls++;
      em.remove_subscriber("t", "a");
    });
    em.add_subscriber("t", "b", [&](const Data &) { b_calls++; });
    for (int i = 0; i < 5; ++i) {
      em.publish("t", Data{1});
    }
    check(wait_until([&] { return b_calls == 5; }), "the other subscriber gets every event");
    check(a_calls == 1, "the removed callback ran once");
  }

  logger.info("--- a callback removes a later subscriber of the same delivery ---");
  {
    TestEventManager em;
    std::atomic<int> b_calls{0}, c_calls{0};
    em.add_subscriber("t", "a", [&](const Data &) { em.remove_subscriber("t", "b"); });
    em.add_subscriber("t", "b", [&](const Data &) { b_calls++; });
    em.add_subscriber("t", "c", [&](const Data &) { c_calls++; });
    em.publish("t", Data{1});
    check(wait_until([&] { return c_calls == 1; }), "the delivery continues past it");
    check(b_calls == 0, "the removed subscriber is skipped in that same delivery");
  }

  logger.info("--- subscribers added during a delivery ---");
  {
    TestEventManager em;
    Latch in_a, finish_a;
    std::atomic<int> b_calls{0}, late_calls{0};
    em.add_subscriber("t", "a", [&](const Data &) {
      in_a.release();
      finish_a.wait();
    });
    em.add_subscriber("t", "b", [&](const Data &) { b_calls++; });
    em.publish("t", Data{1});
    check(in_a.wait(), "first callback running");
    for (int i = 0; i < 16; ++i) { // grows the list the delivery iterates (used to be a UAF)
      em.add_subscriber("t", "late " + std::to_string(i), [&](const Data &) { late_calls++; });
    }
    finish_a.release();
    check(wait_until([&] { return b_calls == 1; }), "the delivery in progress finishes");
    em.publish("t", Data{2});
    check(wait_until([&] { return late_calls == 16 && b_calls == 2; }),
          "the added subscribers get the next event");
  }

  logger.info("--- remove_subscriber waits for that callback to return ---");
  {
    TestEventManager em;
    Latch in_a, finish_a;
    std::atomic<bool> a_running{false};
    std::atomic<int> a_calls{0}, b_calls{0};
    em.add_subscriber("t", "a", [&](const Data &) {
      a_calls++;
      a_running = true;
      in_a.release();
      finish_a.wait();
      a_running = false;
    });
    em.add_subscriber("t", "b", [&](const Data &) { b_calls++; });
    em.publish("t", Data{1});
    check(in_a.wait(), "callback a running");
    std::atomic<bool> removed{false};
    std::atomic<bool> running_when_removed{true};
    std::thread remover([&] {
      em.remove_subscriber("t", "a");
      running_when_removed = a_running.load();
      removed = true;
    });
    std::this_thread::sleep_for(100ms);
    check(!removed, "remove_subscriber is still waiting while the callback runs");
    finish_a.release();
    remover.join();
    check(!running_when_removed, "the callback had returned when remove_subscriber returned");
    em.publish("t", Data{2});
    check(wait_until([&] { return b_calls == 2; }), "the remaining subscriber keeps receiving");
    check(a_calls == 1, "the removed callback is not called again");
  }

  logger.info("--- removing the last subscriber while its callback runs ---");
  {
    // The old code could leave the topic thread waiting forever and hang this
    // removal in join(); also checks the callback is done when it returns.
    TestEventManager em;
    bool ok = true;
    for (int round = 0; round < 200 && ok; ++round) {
      std::atomic<bool> running{false};
      std::atomic<int> calls{0};
      em.add_subscriber("t", "s", [&](const Data &) {
        running = true;
        calls++;
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        running = false;
      });
      for (int i = 0; i < 4; ++i) {
        em.publish("t", Data{1});
      }
      std::this_thread::sleep_for(std::chrono::microseconds(round % 5 * 40));
      ok = returns_within([&] { em.remove_subscriber("t", "s"); }, 5s);
      int calls_at_removal = calls;
      ok = ok && !running;
      std::this_thread::sleep_for(1ms);
      ok = ok && calls == calls_at_removal;
    }
    check(ok, "200 removals returned, with the callback finished and never called again");
  }

  logger.info("--- publish racing the last removal ---");
  {
    TestEventManager em;
    std::atomic<bool> stop{false};
    std::thread publisher([&] {
      while (!stop) {
        em.publish("t", Data{1, 2, 3});
      }
    });
    for (int i = 0; i < 1000; ++i) {
      em.add_subscriber("t", "s", [](const Data &) {});
      em.remove_subscriber("t", "s");
    }
    stop = true;
    publisher.join();
    check(true, "1000 subscribe / unsubscribe cycles while publishing (used to be a UAF)");
  }

  logger.info("--- re-subscribing while the last subscriber leaves ---");
  {
    TestEventManager em;
    int lost = 0;
    for (int round = 0; round < 300; ++round) {
      std::atomic<bool> got{false};
      em.add_subscriber("t", "old", [](const Data &) {});
      std::thread a([&] { em.remove_subscriber("t", "old"); });
      std::thread b([&] { em.add_subscriber("t", "new", [&](const Data &) { got = true; }); });
      a.join();
      b.join();
      em.publish("t", Data{1});
      if (!wait_until([&] { return got.load(); }, 2s)) {
        ++lost;
      }
      em.remove_subscriber("t", "new");
    }
    check(lost == 0, "the new subscriber received data in every one of 300 rounds (lost " +
                         std::to_string(lost) + ")");
  }

  logger.info("--- new topics while another topic is busy ---");
  {
    TestEventManager em;
    std::atomic<int> got{0};
    em.add_subscriber("busy", "rx", [&](const Data &) { got++; });
    std::atomic<bool> stop{false};
    std::thread publisher([&] {
      while (!stop) {
        em.publish("busy", Data{1});
      }
    });
    for (int i = 0; i < 200; ++i) {
      auto topic = "new " + std::to_string(i);
      em.add_subscriber(topic, "x", [](const Data &) {});
      em.publish(topic, Data{2});
      em.remove_subscriber(topic, "x");
    }
    stop = true;
    publisher.join();
    check(got > 0, "200 topics created and removed while publishing (used to race the map)");
  }

  logger.info("--- a callback publishing on its own topic ---");
  {
    TestEventManager em;
    std::atomic<int> calls{0};
    em.add_subscriber("t", "echo", [&](const Data &data) {
      calls++;
      if (data[0] < 5) {
        em.publish("t", Data{static_cast<uint8_t>(data[0] + 1)});
      }
    });
    em.publish("t", Data{1});
    check(wait_until([&] { return calls == 5; }), "each event queues the next one");
  }

  logger.info("--- a callback removing a subscriber of another topic ---");
  {
    TestEventManager em;
    std::atomic<int> other_calls{0};
    std::atomic<bool> removed{false};
    em.add_subscriber("other", "o", [&](const Data &) { other_calls++; });
    em.add_subscriber("other", "o2", [](const Data &) {});
    em.add_subscriber("t", "s",
                      [&](const Data &) { removed = em.remove_subscriber("other", "o"); });
    em.publish("other", Data{1});
    check(wait_until([&] { return other_calls == 1; }), "other topic delivered");
    em.publish("t", Data{1});
    check(wait_until([&] { return removed.load(); }), "removed from a callback of another topic");
    em.publish("other", Data{2});
    std::this_thread::sleep_for(20ms);
    check(other_calls == 1, "and it is not called again");
  }

  logger.info("--- a callback that throws ---");
  {
    TestEventManager em;
    std::atomic<int> after{0};
    em.add_subscriber("t", "thrower", throw_expected);
    em.add_subscriber("t", "after", [&](const Data &) { after++; });
    em.publish("t", Data{1});
    em.publish("t", Data{2});
    check(wait_until([&] { return after == 2; }),
          "the other callbacks and later events still run (used to std::terminate)");
  }

  logger.info("--- clear() ---");
  {
    TestEventManager em;
    std::atomic<int> calls{0};
    em.add_publisher("t", "p");
    em.add_subscriber("t", "a", [&](const Data &) { calls++; });
    em.add_subscriber("u", "b", [&](const Data &) { calls++; });
    em.clear();
    check(!em.publish("t", Data{1}) && !em.publish("u", Data{1}), "nothing to publish to");
    check(!em.remove_publisher("t", "p"), "publishers are gone too");
    check(em.add_subscriber("t", "a", [&](const Data &) { calls++; }), "usable afterwards");
    em.publish("t", Data{1});
    check(wait_until([&] { return calls == 1; }), "and delivers");
    std::atomic<bool> cleared{false};
    em.add_subscriber("c", "clearer", [&](const Data &) {
      em.clear();
      cleared = true;
    });
    em.publish("c", Data{1});
    check(wait_until([&] { return cleared.load(); }), "clear() from inside a callback returns");
    check(!em.publish("t", Data{1}), "and removed everything");
  }

  logger.info("--- destroying it with live subscribers ---");
  {
    auto em = std::make_unique<TestEventManager>();
    Latch in_callback;
    std::atomic<bool> finished{false};
    em->add_subscriber("idle", "i", [](const Data &) {});
    em->add_subscriber("busy", "b", [&](const Data &) {
      in_callback.release();
      std::this_thread::sleep_for(100ms);
      finished = true;
    });
    em->publish("busy", Data{1});
    check(in_callback.wait(), "a callback is running");
    auto *raw = em.release();
    check(returns_within([raw] { delete raw; }, 5s),
          "the destructor returns (used to hang joining an idle topic thread)");
    check(finished, "after the running callback returned");
  }

  logger.info("{} / {} checks passed", total_passed, total_tests);
  if (total_passed != total_tests) {
    logger.error("FAILED");
    return 1;
  }
  logger.info("ALL PASS");
  return 0;
}
