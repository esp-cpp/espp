#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "task.hpp"

#include "Complex_generated_states.hpp"

using namespace std::chrono_literals;

// NOTE: These constants & functions are only for the test bench example which
//       allows the user to manually spawn events through a pseudo-cli
const int numEvents = 5;
const int TickSelection = numEvents + 1;
const int RestartSelection = numEvents + 2;
const int ExitSelection = numEvents + 3;
void display_event_menu();
int get_user_selection();
void make_event(espp::state_machine::Complex::Root &root, int eventIndex);

extern "C" void app_main(void) {
  {
    fmt::print("Starting hfsm example!\n");
    //! [hfsm example]
    const espp::state_machine::Complex::GeneratedEventBase *e = nullptr;
    bool handled = false;

    // create the HFSM
    espp::state_machine::Complex::Root complex_root;

    // set the log callback to print to stdout
    complex_root.set_log_callback([](std::string_view msg) { fmt::print("{}\n", msg); });

    // initialize the HFSM
    complex_root.initialize();

    // start a task to run the hfsm
    auto task_fn = [&complex_root](std::mutex &m, std::condition_variable &cv) {
      // execute the state machine
      complex_root.handle_all_events();
      complex_root.tick();
      // NOTE: if we call tick above, then we need to call handle_all_events()
      //      again to handle any events that were spawned by the tick()
      complex_root.handle_all_events();
      // get the active state period (the state may have changed from handling
      // events, so we always need to get the active leaf)
      auto current_hfsm_period = complex_root.getActiveLeaf()->getTimerPeriod();
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, std::chrono::duration<float>(current_hfsm_period));
      }
      // stop the task if the hfsm has stopped (reached its end state)
      return complex_root.has_stopped();
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "HFSM"},
                            .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();

    // from other contexts you can spawn events into the HFSM. the functions are
    // generated based on the event names from the model, and the data
    // structures are generated into <state machine name>_event_data.hpp so that
    // you can enforce that events must have certain data. The sequence below
    // should transition the HFSM through a few states until it gets to a state
    // where the ENDEVENT will cause the whole HFSM to terminate.
    complex_root.spawn_EVENT4_event({});
    complex_root.spawn_EVENT1_event({});
    complex_root.spawn_EVENT2_event({});
    complex_root.spawn_EVENT3_event({});
    complex_root.spawn_EVENT1_event({});
    complex_root.spawn_ENDEVENT_event({});

    // give the hfsm some time to handle the events before we fully exit
    std::this_thread::sleep_for(1s);
    //! [hfsm example]
  }

  {
    fmt::print("Starting test bench hfsm example!\n");
    //! [hfsm test bench example]

    // NOTE: we need to configure stdin/stdout to use std::cin for
    //       get_user_selection() function.
    espp::Cli::configure_stdin_stdout();

    const espp::state_machine::Complex::GeneratedEventBase *e = nullptr;

    // create the HFSM
    espp::state_machine::Complex::Root complex_root;

    // set the log callback to print to stdout
    complex_root.set_log_callback([](std::string_view msg) { fmt::print("{}\n", msg); });

    // initialize the HFSM
    complex_root.initialize();

    // start a task to run the hfsm
    auto task_fn = [&complex_root](std::mutex &m, std::condition_variable &cv) {
      // execute the state machine
      complex_root.handle_all_events();

      // NOTE: we would normally call the tick() function here, but we want to
      //       be able to manually tick the HFSM from the test bench and we
      //       don't want to clutter the log with the tick() messages.
      // complex_root.tick();

      // NOTE: if we call tick above, then we need to call handle_all_events()
      //       again to handle any events that were spawned by the tick()
      //       function
      // complex_root.handle_all_events();

      // get the active state period (the state may have changed from handling
      // events, so we always need to get the active leaf)
      auto current_hfsm_period = complex_root.getActiveLeaf()->getTimerPeriod();
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, std::chrono::duration<float>(current_hfsm_period));
      }
      // stop the task if the hfsm has stopped (reached its end state)
      return complex_root.has_stopped();
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "HFSM"},
                            .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();

    // NOTE: this is just a copy of the HFSM code from the generated test bench,
    //       and is not intended to show how to actually run the HFSM in
    //       production.
    while (!complex_root.has_stopped()) {
      do {
        std::this_thread::sleep_for(100ms);
      } while (complex_root.has_events());
      display_event_menu();
      int selection = get_user_selection();
      if (selection == ExitSelection) {
        complex_root.terminate();
        break;
      } else if (selection == RestartSelection) {
        complex_root.restart();
      } else if (selection == TickSelection) {
        complex_root.tick();
      } else {
        make_event(complex_root, selection);
      }
    }
    //! [hfsm test bench example]
  }

  fmt::print("hfsm example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

void display_event_menu() {
  std::cout << "\n-----\nSelect which event to spawn:" << std::endl
            << "\t0. ENDEVENT" << std::endl
            << "\t1. EVENT1" << std::endl
            << "\t2. EVENT2" << std::endl
            << "\t3. EVENT3" << std::endl
            << "\t4. EVENT4" << std::endl
            << "\t5. None" << std::endl
            << "\t" << TickSelection << ". HFSM Tick" << std::endl
            << "\t" << RestartSelection << ". Restart HFSM" << std::endl
            << "\t" << ExitSelection << ". Exit HFSM" << std::endl
            << "selection: ";
}

int get_user_selection() {
  int s = 0;
  std::cin >> s;
  return s;
}

void make_event(espp::state_machine::Complex::Root &root, int eventIndex) {
  if (eventIndex < numEvents && eventIndex > -1) {
    switch (eventIndex) {
    case 0: {
      espp::state_machine::Complex::ENDEVENTEventData data{};
      root.spawn_ENDEVENT_event(data);
      break;
    }
    case 1: {
      espp::state_machine::Complex::EVENT1EventData data{};
      root.spawn_EVENT1_event(data);
      break;
    }
    case 2: {
      espp::state_machine::Complex::EVENT2EventData data{};
      root.spawn_EVENT2_event(data);
      break;
    }
    case 3: {
      espp::state_machine::Complex::EVENT3EventData data{};
      root.spawn_EVENT3_event(data);
      break;
    }
    case 4: {
      espp::state_machine::Complex::EVENT4EventData data{};
      root.spawn_EVENT4_event(data);
      break;
    }
    default:
      break;
    }
  }
}
