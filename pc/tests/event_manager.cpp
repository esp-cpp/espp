// Host test for espp::EventManager: the pub/sub API plus the concurrency cases
// that used to hang, crash, or corrupt memory (removing the last subscriber
// while its callback ran, a callback removing itself, subscribers added or
// removed during a delivery, publish racing the last removal, a re-subscribe
// racing the last removal, new topics racing publish, destruction with live
// subscribers), and the cases where remove_subscriber() must still wait, or
// must not wait because it would deadlock (a callback that removed itself,
// clear() running, callbacks of several topics removing each other's, a
// callback's destructor calling back in, subscribing during clear()). Runs
// under TSan and ASan in host_sanitizers.yml.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <functional>
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
// gives each case a fresh instance, lets a case destroy one, and shows a little
// of its state.
struct TestEventManager : public espp::EventManager {
  TestEventManager() { set_log_level(espp::Logger::Verbosity::ERROR); }
  ~TestEventManager() {
    if (on_destroy) {
      on_destroy();
    }
  }
  // topics whose last subscriber was removed and whose thread is not joined yet
  size_t retiring() {
    std::lock_guard<std::mutex> lk(mutex_);
    return retiring_.size();
  }
  bool clearing() {
    std::lock_guard<std::mutex> lk(mutex_);
    return clearing_ > 0;
  }
  std::function<void()> on_destroy;
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

// Run fn on another thread and return true once it has returned. If it hasn't
// within the timeout, fail the whole test right away: the stuck thread still
// uses the case's state, so carrying on would only crash later checks.
template <typename Fn>
bool returns_within(Fn &&fn, std::chrono::milliseconds timeout, const char *what) {
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread([fn = std::forward<Fn>(fn), done]() mutable {
    fn();
    *done = true;
  }).detach();
  if (!wait_until([&] { return done->load(); }, timeout)) {
    std::fprintf(stderr, "FAIL: %s: did not return within %lld ms (hang)\n", what,
                 static_cast<long long>(timeout.count()));
    std::_Exit(1);
  }
  return true;
}

// Lets n threads continue only once all of them have arrived.
struct Rendezvous {
  explicit Rendezvous(int parties)
      : parties(parties) {}
  bool arrive_and_wait() {
    arrived++;
    return wait_until([this] { return arrived >= parties; }, 5s);
  }
  const int parties;
  std::atomic<int> arrived{0};
};

// Something a subscriber callback owns, which calls back into the
// EventManager when destroyed (RAII unsubscribe).
struct Owner {
  explicit Owner(std::function<void()> on_destroy)
      : on_destroy(std::move(on_destroy)) {}
  ~Owner() { on_destroy(); }
  std::function<void()> on_destroy;
};
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
    check(late_calls == 0, "without calling the subscribers added during it");
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
    Latch removing;
    std::thread remover([&] {
      removing.release();
      em.remove_subscriber("t", "a");
      running_when_removed = a_running.load();
      removed = true;
    });
    removing.wait();
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
      returns_within([&] { em.remove_subscriber("t", "s"); }, 5s,
                     "removing the last subscriber while its callback runs");
      int calls_at_removal = calls;
      ok = !running;
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
    std::atomic<int> other_calls{0}, o2_calls{0};
    std::atomic<bool> removed{false};
    em.add_subscriber("other", "o", [&](const Data &) { other_calls++; });
    em.add_subscriber("other", "o2", [&](const Data &) { o2_calls++; });
    em.add_subscriber("t", "s",
                      [&](const Data &) { removed = em.remove_subscriber("other", "o"); });
    em.publish("other", Data{1});
    check(wait_until([&] { return other_calls == 1; }), "other topic delivered");
    em.publish("t", Data{1});
    check(wait_until([&] { return removed.load(); }), "removed from a callback of another topic");
    em.publish("other", Data{2});
    // o2 is called after o, so once o2 has the event o would have had it too
    check(wait_until([&] { return o2_calls == 2; }) && other_calls == 1,
          "and it is not called again");
  }

  logger.info("--- remove_subscriber waits for a callback that removed itself ---");
  {
    // e.g. a one-shot callback that unsubscribes first and then keeps using
    // its object, whose destructor calls remove_subscriber() as well
    TestEventManager em;
    for (bool last : {false, true}) {
      const std::string which = last ? " (it was the topic's last subscriber)" : "";
      Latch removed_itself, finish;
      std::atomic<bool> running{false};
      em.add_subscriber("t", "one-shot", [&](const Data &) {
        running = true;
        em.remove_subscriber("t", "one-shot");
        removed_itself.release();
        finish.wait();
        running = false;
      });
      if (!last) {
        em.add_subscriber("t", "keep", [](const Data &) {});
      }
      em.publish("t", Data{1});
      check(removed_itself.wait(), "the callback removed itself and keeps running" + which);
      std::atomic<bool> returned{false}, result{true}, running_at_return{true};
      Latch removing;
      std::thread owner([&] {
        removing.release();
        result = em.remove_subscriber("t", "one-shot");
        running_at_return = running.load();
        returned = true;
      });
      removing.wait();
      std::this_thread::sleep_for(50ms);
      check(!returned, "a second remove_subscriber waits for it" + which);
      finish.release();
      owner.join();
      check(!result && !running_at_return,
            "and returns false once the callback has returned (used to return at once)" + which);
      if (!last) {
        em.remove_subscriber("t", "keep");
      }
    }
  }

  logger.info("--- remove_subscriber while clear() runs ---");
  {
    TestEventManager em;
    Latch in_a, finish_a;
    std::atomic<bool> running{false};
    em.add_subscriber("t", "a", [&](const Data &) {
      running = true;
      in_a.release();
      finish_a.wait();
      running = false;
    });
    em.publish("t", Data{1});
    check(in_a.wait(), "callback running");
    std::thread clearer([&] { em.clear(); });
    check(wait_until([&] { return em.clearing(); }), "clear() removed it and waits for it");
    std::atomic<bool> returned{false}, running_at_return{true};
    std::thread remover([&] {
      em.remove_subscriber("t", "a");
      running_at_return = running.load();
      returned = true;
    });
    std::this_thread::sleep_for(50ms);
    check(!returned, "remove_subscriber waits for the callback too");
    finish_a.release();
    remover.join();
    clearer.join();
    check(!running_at_return, "and returns once it has returned (used to return at once)");
  }

  logger.info("--- callbacks of two topics removing each other's subscribers ---");
  {
    // used to deadlock: each topic's thread waited for the other's delivery
    TestEventManager em;
    Rendezvous meet(2);
    std::atomic<int> done{0};
    em.add_subscriber("A", "a1", [&](const Data &) {
      meet.arrive_and_wait();
      em.remove_subscriber("B", "b2");
      done++;
    });
    em.add_subscriber("A", "a2", [](const Data &) {});
    em.add_subscriber("B", "b1", [&](const Data &) {
      meet.arrive_and_wait();
      em.remove_subscriber("A", "a2");
      done++;
    });
    em.add_subscriber("B", "b2", [](const Data &) {});
    em.publish("A", Data{1});
    em.publish("B", Data{1});
    check(wait_until([&] { return done == 2; }, 5s),
          "both removals of a subscriber that isn't running return (used to deadlock)");
  }
  {
    // each removes the callback the other topic is running: one of them can't
    // wait (that would deadlock), the other waits for it
    TestEventManager em;
    Rendezvous meet(2);
    std::atomic<int> done{0}, a1_calls{0}, b1_calls{0}, a2_calls{0}, b2_calls{0};
    std::atomic<bool> removed_a1{false}, removed_b1{false};
    em.add_subscriber("A", "a1", [&](const Data &) {
      a1_calls++;
      meet.arrive_and_wait();
      removed_b1 = em.remove_subscriber("B", "b1");
      done++;
    });
    em.add_subscriber("A", "a2", [&](const Data &) { a2_calls++; });
    em.add_subscriber("B", "b1", [&](const Data &) {
      b1_calls++;
      meet.arrive_and_wait();
      removed_a1 = em.remove_subscriber("A", "a1");
      done++;
    });
    em.add_subscriber("B", "b2", [&](const Data &) { b2_calls++; });
    em.publish("A", Data{1});
    em.publish("B", Data{1});
    check(wait_until([&] { return done == 2; }, 5s),
          "both removals of the other topic's running callback return");
    check(removed_a1 && removed_b1, "and both removed it");
    em.publish("A", Data{2});
    em.publish("B", Data{2});
    check(wait_until([&] { return a2_calls == 2 && b2_calls == 2; }) && a1_calls == 1 &&
              b1_calls == 1,
          "neither removed callback is called again");
  }
  {
    // the same around three topics: X waits for Y, Y for Z, Z would wait for X
    TestEventManager em;
    Rendezvous meet(3);
    std::atomic<int> done{0};
    const std::vector<std::string> names{"X", "Y", "Z"};
    for (size_t i = 0; i < names.size(); ++i) {
      const auto next = names[(i + 1) % names.size()];
      em.add_subscriber(names[i], "running", [&, next](const Data &) {
        meet.arrive_and_wait();
        em.remove_subscriber(next, "running");
        done++;
      });
      em.add_subscriber(names[i], "other", [](const Data &) {});
    }
    for (const auto &name : names) {
      em.publish(name, Data{1});
    }
    check(wait_until([&] { return done == 3; }, 5s),
          "a ring of three topics removing the next one's running callback returns");
  }
  {
    // clear() from a callback of A while a callback of B removes A's
    TestEventManager em;
    Rendezvous meet(2);
    std::atomic<int> done{0};
    em.add_subscriber("A", "a1", [&](const Data &) {
      meet.arrive_and_wait();
      em.clear();
      done++;
    });
    em.add_subscriber("A", "a2", [](const Data &) {});
    em.add_subscriber("B", "b1", [&](const Data &) {
      meet.arrive_and_wait();
      em.remove_subscriber("A", "a1");
      done++;
    });
    em.publish("A", Data{1});
    em.publish("B", Data{1});
    check(wait_until([&] { return done == 2; }, 5s),
          "clear() in one topic's callback while another topic's removes its callback returns");
    check(!em.publish("A", Data{1}) && !em.publish("B", Data{1}), "and everything was removed");
  }

  logger.info("--- a callback's destructor calls back into the EventManager ---");
  for (int mode = 0; mode < 2; ++mode) {
    // A one-shot callback that owns the last reference to an object removes
    // itself, so the object is destroyed on the topic's thread when that
    // delivery lets go of the callback; its destructor then removes another
    // subscriber of the same topic (mode 0), or clears everything (mode 1).
    TestEventManager em;
    std::atomic<bool> destroyed{false};
    {
      auto owner = std::make_shared<Owner>([&em, &destroyed, mode] {
        if (mode == 0) {
          em.remove_subscriber("t", "extra");
        } else {
          em.clear();
        }
        destroyed = true;
      });
      em.add_subscriber("t", "owner",
                        [&em, owner](const Data &) { em.remove_subscriber("t", "owner"); });
      em.add_subscriber("t", "extra", [](const Data &) {});
      em.add_subscriber("t", "other", [](const Data &) {});
    }
    em.publish("t", Data{1});
    check(wait_until([&] { return destroyed.load(); }, 5s),
          mode == 0 ? "destructor removing another subscriber returns (used to deadlock)"
                    : "destructor calling clear() returns (used to std::terminate)");
  }
  {
    // Removed from another thread while not running: the callback (and what it
    // owns) is destroyed by remove_subscriber(), on the caller's thread.
    TestEventManager em;
    std::atomic<bool> destroyed{false};
    std::thread::id destroyed_on;
    {
      auto owner = std::make_shared<Owner>([&] {
        destroyed_on = std::this_thread::get_id();
        destroyed = true;
      });
      em.add_subscriber("t", "owner", [owner](const Data &) {});
      em.add_subscriber("t", "other", [](const Data &) {});
    }
    em.publish("t", Data{1});
    em.remove_subscriber("t", "owner");
    check(destroyed && destroyed_on == std::this_thread::get_id(),
          "a removed callback is destroyed before remove_subscriber() returns");
  }

  logger.info("--- subscribing while clear() or the destructor runs ---");
  {
    TestEventManager em;
    Latch in_a;
    std::atomic<bool> added{true}, finished{false};
    em.add_subscriber("t", "a", [&](const Data &) {
      in_a.release();
      wait_until([&] { return em.clearing(); }); // clear() is waiting for us
      added = em.add_subscriber("late", "x", [](const Data &) {});
      finished = true;
    });
    em.publish("t", Data{1});
    check(in_a.wait(), "callback running");
    em.clear();
    check(finished && !added, "add_subscriber() from a callback that clear() waits for fails");
    check(!em.publish("late", Data{1}), "and nothing is left subscribed");
  }
  {
    // the destructor clears too; a subscriber added then used to be destroyed
    // without being stopped, and joining its thread hung the destructor
    auto em = std::make_unique<TestEventManager>();
    Latch in_a;
    std::atomic<bool> destroying{false};
    em->on_destroy = [&] { destroying = true; };
    espp::EventManager *base = em.get();
    em->add_subscriber("t", "a", [&, base](const Data &) {
      in_a.release();
      wait_until([&] { return destroying.load(); });
      std::this_thread::sleep_for(20ms); // let ~EventManager start waiting for us
      base->add_subscriber("late", "x", [](const Data &) {});
    });
    em->publish("t", Data{1});
    check(in_a.wait(), "callback running");
    auto *raw = em.release();
    check(returns_within([raw] { delete raw; }, 5s, "destructor while a callback subscribes"),
          "the destructor returns while a callback it waits for subscribes (used to hang)");
  }

  logger.info("--- a topic that removed its last subscriber itself is joined later ---");
  {
    TestEventManager em;
    std::atomic<bool> done{false};
    em.add_subscriber("t", "once", [&](const Data &) {
      em.remove_subscriber("t", "once");
      done = true;
    });
    em.add_subscriber("other", "o", [](const Data &) {});
    em.publish("t", Data{1});
    check(wait_until([&] { return done.load(); }), "the callback removed itself");
    check(em.retiring() == 1, "its thread can't be joined from itself, so it waits");
    check(wait_until([&] {
            em.publish("other", Data{1});
            return em.retiring() == 0;
          }),
          "a later publish() joins it (on ESP-IDF that frees its stack)");
  }

  logger.info("--- a callback that removes the last subscriber and subscribes again ---");
  {
    TestEventManager em;
    std::mutex m;
    std::vector<std::thread::id> threads;
    std::atomic<int> calls{0};
    std::function<void(const Data &)> rearm = [&](const Data &) {
      {
        std::lock_guard<std::mutex> lk(m);
        threads.push_back(std::this_thread::get_id());
      }
      em.remove_subscriber("t", "rearm");
      em.add_subscriber("t", "rearm", rearm);
      calls++;
    };
    em.add_subscriber("t", "rearm", rearm);
    bool delivered = true;
    for (int i = 1; i <= 5 && delivered; ++i) {
      em.publish("t", Data{1});
      delivered = wait_until([&] { return calls == i; });
    }
    check(delivered, "every event is delivered to the re-added subscriber");
    {
      std::lock_guard<std::mutex> lk(m);
      bool one_thread = threads.size() == 5;
      for (const auto &id : threads) {
        one_thread = one_thread && id == threads.front();
      }
      check(one_thread && em.retiring() == 0,
            "on the topic's one thread (used to start a new thread every time)");
    }
    em.remove_subscriber("t", "rearm");
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
    check(returns_within([raw] { delete raw; }, 5s, "destructor with live subscribers"),
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
