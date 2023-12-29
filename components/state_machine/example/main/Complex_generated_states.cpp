#include "Complex_generated_states.hpp"

using namespace espp::state_machine;
using namespace espp::state_machine::Complex;

// User Definitions for the HFSM
//::::/c::::Definitions::::

/* * *  Definitions for Root : /c  * * */
// Generated Definitions for the root state
void Root::initialize(void) {
  // Run the model's Initialization code
  log("\033[36mComplex:/c HFSM Initialization\033[0m");
  //::::/c::::Initialization::::

  // now set the states up properly
  // External Transition : Action for: /c/m
  _root->log("\033[36mTRANSITION::ACTION for /c/m\033[0m");

  //::::/c/m::::Action::::

  // State : entry for: /c/Y
  _root->COMPLEX_OBJ__STATE_1_OBJ.entry();

  // initialize our new active state
  _root->COMPLEX_OBJ__STATE_1_OBJ.initialize();
};

void Root::handle_all_events(void) {
  GeneratedEventBase *e;
  // get the next event and check if it's nullptr
  while ((e = event_factory.get_next_event())) {
    [[maybe_unused]] bool did_handle = handleEvent(e);
    log("\033[0mHANDLED " + e->to_string() + (did_handle ? ": \033[32mtrue" : ": \033[31mfalse") +
        "\033[0m");
    // free the memory that was allocated when it was spawned
    consume_event(e);
  }
}

void Root::terminate(void) {
  // will call exit() and exitChildren() on _activeState if it
  // exists
  exitChildren();
};

void Root::restart(void) {
  terminate();
  initialize();
};

bool Root::has_stopped(void) {
  bool reachedEnd = false;
  // Get the currently active leaf state
  const StateBase *activeLeaf = getActiveLeaf();
  if (activeLeaf != nullptr && activeLeaf != this &&
      activeLeaf == static_cast<StateBase *>(&_root->COMPLEX_OBJ__END_STATE_OBJ)) {
    reachedEnd = true;
  }
  return reachedEnd;
};

bool Root::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // Get the currently active leaf state
  StateBase *activeLeaf = getActiveLeaf();

  if (activeLeaf != nullptr && activeLeaf != this) {
    // have the active leaf handle the event, this will bubble up until
    // the event is handled or it reaches the root.
    handled = activeLeaf->handleEvent(event);
  }

  return handled;
}

/* * *  Definitions for State_1 : /c/Y  * * */

// User Definitions for the HFSM
//::::/c/Y::::Definitions::::

void Root::State_1::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State_1::entry(void) {
  _root->log("\033[36mENTRY::State_1::/c/Y\033[0m");
  // Entry action for this state
  //::::/c/Y::::Entry::::
  [[maybe_unused]] int a = 2;
  printf("SerialTask :: initializing State 1\n");
}

void Root::State_1::exit(void) {
  _root->log("\033[36mEXIT::State_1::/c/Y\033[0m");
  // Call the Exit Action for this state
  //::::/c/Y::::Exit::::
  printf("Exiting State 1\n");
}

void Root::State_1::tick(void) {
  _root->log("\033[36mTICK::State_1::/c/Y\033[0m");
  // Call the Tick Action for this state
  //::::/c/Y::::Tick::::
  printf("SerialTask::State 1::tick()\n");
  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_1::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State_1::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::ENDEVENT:
  case EventType::EVENT3:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  case EventType::EVENT1:
    if (false) { // makes generation easier :)
    }
    //::::/c/Y/t::::Guard::::
    else if (_root->someNumber < _root->someValue) {
      _root->log("\033[37mGUARD [ _root->someNumber < _root->someValue ] for INTERNAL "
                 "TRANSITION:/c/Y/t evaluated to TRUE\033[0m");
      // run transition action
      //::::/c/Y/t::::Action::::
      int testVal = 32;
      for (int i = 0; i < testVal; i++) {
        printf("Action iterating: %d\n", i);
      }
      // make sure nothing else handles this event
      handled = true;
    }
    break;
  case EventType::EVENT2:
    if (false) { // makes generation easier :)
    }
    //::::/c/Y/X::::Guard::::
    else if (_root->someNumber > _root->someValue) {
      _root->log("\033[37mGUARD [ _root->someNumber > _root->someValue ] for INTERNAL "
                 "TRANSITION:/c/Y/X evaluated to TRUE\033[0m");
      // run transition action
      //::::/c/Y/X::::Action::::

      // make sure nothing else handles this event
      handled = true;
    }
    break;
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT4:
      if (false) {
      } // makes generation easier :)
      //::::/c/I::::Guard::::
      else if (_root->someTest) {
        _root->log("\033[37mGUARD [ _root->someTest ] for EXTERNAL TRANSITION:/c/I evaluated to "
                   "TRUE\033[0m");
        // Going into a choice pseudo-state, let it handle its
        // guards and perform the state transition
        if (false) {
        } // makes geneeration easier :)
        //::::/c/h::::Guard::::
        else if (_root->goToHistory) {
          _root->log("\033[37mGUARD [ _root->goToHistory ] for EXTERNAL TRANSITION:/c/h evaluated "
                     "to TRUE\033[0m");
          // Transitioning states!
          // Call all from prev state down exits
          _root->COMPLEX_OBJ__STATE_1_OBJ.exitChildren();
          // State : exit for: /c/Y
          _root->COMPLEX_OBJ__STATE_1_OBJ.exit();
          // External Transition : Action for: /c/I
          _root->log("\033[36mTRANSITION::ACTION for /c/I\033[0m");

          //::::/c/I::::Action::::

          // External Transition : Action for: /c/h
          _root->log("\033[36mTRANSITION::ACTION for /c/h\033[0m");

          //::::/c/h::::Action::::

          // State : entry for: /c/T
          _root->COMPLEX_OBJ__STATE3_OBJ.entry();
          _root->log(
              "\033[31mSTATE TRANSITION: State_1->State3::Shallow_History_Pseudostate\033[0m");

          // going into shallow history pseudo-state
          _root->COMPLEX_OBJ__STATE3_OBJ.setShallowHistory();
          // make sure nothing else handles this event
          handled = true;
        }
        //::::/c/k::::Guard::::
        else if (_root->nextState) {
          _root->log("\033[37mGUARD [ _root->nextState ] for EXTERNAL TRANSITION:/c/k evaluated to "
                     "TRUE\033[0m");
          // Transitioning states!
          // Call all from prev state down exits
          _root->COMPLEX_OBJ__STATE_1_OBJ.exitChildren();
          // State : exit for: /c/Y
          _root->COMPLEX_OBJ__STATE_1_OBJ.exit();
          // External Transition : Action for: /c/I
          _root->log("\033[36mTRANSITION::ACTION for /c/I\033[0m");

          //::::/c/I::::Action::::

          // External Transition : Action for: /c/k
          _root->log("\033[36mTRANSITION::ACTION for /c/k\033[0m");

          //::::/c/k::::Action::::

          // State : entry for: /c/v
          _root->COMPLEX_OBJ__STATE_2_OBJ.entry();
          _root->log("\033[31mSTATE TRANSITION: State_1->State_2\033[0m");

          // going into regular state
          _root->COMPLEX_OBJ__STATE_2_OBJ.initialize();
          // make sure nothing else handles this event
          handled = true;
        } else if (true) {
          _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/r\033[0m");
          // Transitioning states!
          // Call all from prev state down exits
          _root->COMPLEX_OBJ__STATE_1_OBJ.exitChildren();
          // State : exit for: /c/Y
          _root->COMPLEX_OBJ__STATE_1_OBJ.exit();
          // External Transition : Action for: /c/I
          _root->log("\033[36mTRANSITION::ACTION for /c/I\033[0m");

          //::::/c/I::::Action::::

          // External Transition : Action for: /c/r
          _root->log("\033[36mTRANSITION::ACTION for /c/r\033[0m");

          //::::/c/r::::Action::::

          // State : entry for: /c/T
          _root->COMPLEX_OBJ__STATE3_OBJ.entry();
          _root->log("\033[31mSTATE TRANSITION: State_1->State3\033[0m");

          // going into regular state
          _root->COMPLEX_OBJ__STATE3_OBJ.initialize();
          // make sure nothing else handles this event
          handled = true;
        }
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  // can't buble up, we are a root state.
  return handled;
}
/* * *  Definitions for State_2 : /c/v  * * */

// User Definitions for the HFSM
//::::/c/v::::Definitions::::

void Root::State_2::initialize(void) {
  // External Transition : Action for: /c/v/u
  _root->log("\033[36mTRANSITION::ACTION for /c/v/u\033[0m");

  //::::/c/v/u::::Action::::

  // State : entry for: /c/v/K
  _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ.entry();

  // initialize our new active state
  _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ.initialize();
}

void Root::State_2::entry(void) {
  _root->log("\033[36mENTRY::State_2::/c/v\033[0m");
  // Entry action for this state
  //::::/c/v::::Entry::::
}

void Root::State_2::exit(void) {
  _root->log("\033[36mEXIT::State_2::/c/v\033[0m");
  // Call the Exit Action for this state
  //::::/c/v::::Exit::::
}

void Root::State_2::tick(void) {
  _root->log("\033[36mTICK::State_2::/c/v\033[0m");
  // Call the Tick Action for this state
  //::::/c/v::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_2::getTimerPeriod(void) { return (double)(0); }

bool Root::State_2::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::ENDEVENT:
  case EventType::EVENT1:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT4:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/Q\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ.exitChildren();
        // State : exit for: /c/v
        _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
        // External Transition : Action for: /c/Q
        _root->log("\033[36mTRANSITION::ACTION for /c/Q\033[0m");

        //::::/c/Q::::Action::::

        // State : entry for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State_2->State3::Deep_History_Pseudostate\033[0m");

        // going into deep history pseudo-state
        _root->COMPLEX_OBJ__STATE3_OBJ.setDeepHistory();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    case EventType::EVENT2:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/E\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ.exitChildren();
        // State : exit for: /c/v
        _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
        // External Transition : Action for: /c/E
        _root->log("\033[36mTRANSITION::ACTION for /c/E\033[0m");

        //::::/c/E::::Action::::

        // State : entry for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State_2->State3\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE3_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    case EventType::EVENT3:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/t\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ.exitChildren();
        // State : exit for: /c/v
        _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
        // External Transition : Action for: /c/t
        _root->log("\033[36mTRANSITION::ACTION for /c/t\033[0m");

        //::::/c/t::::Action::::

        // State : entry for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State_2->State3::Shallow_History_Pseudostate\033[0m");

        // going into shallow history pseudo-state
        _root->COMPLEX_OBJ__STATE3_OBJ.setShallowHistory();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  // can't buble up, we are a root state.
  return handled;
}
/* * *  Definitions for State_2::ChildState : /c/v/K  * * */

// User Definitions for the HFSM
//::::/c/v/K::::Definitions::::

void Root::State_2::ChildState::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State_2::ChildState::entry(void) {
  _root->log("\033[36mENTRY::State_2::ChildState::/c/v/K\033[0m");
  // Entry action for this state
  //::::/c/v/K::::Entry::::
}

void Root::State_2::ChildState::exit(void) {
  _root->log("\033[36mEXIT::State_2::ChildState::/c/v/K\033[0m");
  // Call the Exit Action for this state
  //::::/c/v/K::::Exit::::
}

void Root::State_2::ChildState::tick(void) {
  _root->log("\033[36mTICK::State_2::ChildState::/c/v/K\033[0m");
  // Call the Tick Action for this state
  //::::/c/v/K::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_2::ChildState::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State_2::ChildState::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::ENDEVENT:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT1:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/S\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ.exitChildren();
        // State : exit for: /c/v/K
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ.exit();
        // External Transition : Action for: /c/v/S
        _root->log("\033[36mTRANSITION::ACTION for /c/v/S\033[0m");

        //::::/c/v/S::::Action::::

        // State : entry for: /c/v/e
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE2_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State_2::ChildState->State_2::ChildState2\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE2_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State_2::ChildState2 : /c/v/e  * * */

// User Definitions for the HFSM
//::::/c/v/e::::Definitions::::

void Root::State_2::ChildState2::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State_2::ChildState2::entry(void) {
  _root->log("\033[36mENTRY::State_2::ChildState2::/c/v/e\033[0m");
  // Entry action for this state
  //::::/c/v/e::::Entry::::
}

void Root::State_2::ChildState2::exit(void) {
  _root->log("\033[36mEXIT::State_2::ChildState2::/c/v/e\033[0m");
  // Call the Exit Action for this state
  //::::/c/v/e::::Exit::::
}

void Root::State_2::ChildState2::tick(void) {
  _root->log("\033[36mTICK::State_2::ChildState2::/c/v/e\033[0m");
  // Call the Tick Action for this state
  //::::/c/v/e::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_2::ChildState2::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State_2::ChildState2::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::ENDEVENT:
  case EventType::EVENT1:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT2:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/W\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE2_OBJ.exitChildren();
        // State : exit for: /c/v/e
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE2_OBJ.exit();
        // External Transition : Action for: /c/v/W
        _root->log("\033[36mTRANSITION::ACTION for /c/v/W\033[0m");

        //::::/c/v/W::::Action::::

        // State : entry for: /c/v/z
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State_2::ChildState2->State_2::ChildState3\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State_2::ChildState3 : /c/v/z  * * */

// User Definitions for the HFSM
//::::/c/v/z::::Definitions::::

void Root::State_2::ChildState3::initialize(void) {
  // External Transition : Action for: /c/v/z/8
  _root->log("\033[36mTRANSITION::ACTION for /c/v/z/8\033[0m");

  //::::/c/v/z/8::::Action::::

  // State : entry for: /c/v/z/6
  _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ.entry();

  // initialize our new active state
  _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ.initialize();
}

void Root::State_2::ChildState3::entry(void) {
  _root->log("\033[36mENTRY::State_2::ChildState3::/c/v/z\033[0m");
  // Entry action for this state
  //::::/c/v/z::::Entry::::
}

void Root::State_2::ChildState3::exit(void) {
  _root->log("\033[36mEXIT::State_2::ChildState3::/c/v/z\033[0m");
  // Call the Exit Action for this state
  //::::/c/v/z::::Exit::::
}

void Root::State_2::ChildState3::tick(void) {
  _root->log("\033[36mTICK::State_2::ChildState3::/c/v/z\033[0m");
  // Call the Tick Action for this state
  //::::/c/v/z::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_2::ChildState3::getTimerPeriod(void) { return (double)(0); }

bool Root::State_2::ChildState3::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::ENDEVENT:
  case EventType::EVENT1:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT3:
      if (false) {
      } // makes generation easier :)
      //::::/c/v/P::::Guard::::
      else if (_root->someGuard) {
        _root->log("\033[37mGUARD [ _root->someGuard ] for EXTERNAL TRANSITION:/c/v/P evaluated to "
                   "TRUE\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exitChildren();
        // State : exit for: /c/v/z
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
        // External Transition : Action for: /c/v/P
        _root->log("\033[36mTRANSITION::ACTION for /c/v/P\033[0m");

        //::::/c/v/P::::Action::::

        // State : entry for: /c/v/K
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State_2::ChildState3->State_2::ChildState\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State_2::ChildState3::Grand : /c/v/z/6  * * */

// User Definitions for the HFSM
//::::/c/v/z/6::::Definitions::::

void Root::State_2::ChildState3::Grand::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State_2::ChildState3::Grand::entry(void) {
  _root->log("\033[36mENTRY::State_2::ChildState3::Grand::/c/v/z/6\033[0m");
  // Entry action for this state
  //::::/c/v/z/6::::Entry::::
}

void Root::State_2::ChildState3::Grand::exit(void) {
  _root->log("\033[36mEXIT::State_2::ChildState3::Grand::/c/v/z/6\033[0m");
  // Call the Exit Action for this state
  //::::/c/v/z/6::::Exit::::
}

void Root::State_2::ChildState3::Grand::tick(void) {
  _root->log("\033[36mTICK::State_2::ChildState3::Grand::/c/v/z/6\033[0m");
  // Call the Tick Action for this state
  //::::/c/v/z/6::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_2::ChildState3::Grand::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State_2::ChildState3::Grand::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::ENDEVENT:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT1:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/z/z\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ.exitChildren();
        // State : exit for: /c/v/z/6
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ.exit();
        // External Transition : Action for: /c/v/z/z
        _root->log("\033[36mTRANSITION::ACTION for /c/v/z/z\033[0m");

        //::::/c/v/z/z::::Action::::

        // State : entry for: /c/v/z/c
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: "
                   "State_2::ChildState3::Grand->State_2::ChildState3::Grand2\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State_2::ChildState3::Grand2 : /c/v/z/c  * * */

// User Definitions for the HFSM
//::::/c/v/z/c::::Definitions::::

void Root::State_2::ChildState3::Grand2::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State_2::ChildState3::Grand2::entry(void) {
  _root->log("\033[36mENTRY::State_2::ChildState3::Grand2::/c/v/z/c\033[0m");
  // Entry action for this state
  //::::/c/v/z/c::::Entry::::
}

void Root::State_2::ChildState3::Grand2::exit(void) {
  _root->log("\033[36mEXIT::State_2::ChildState3::Grand2::/c/v/z/c\033[0m");
  // Call the Exit Action for this state
  //::::/c/v/z/c::::Exit::::
}

void Root::State_2::ChildState3::Grand2::tick(void) {
  _root->log("\033[36mTICK::State_2::ChildState3::Grand2::/c/v/z/c\033[0m");
  // Call the Tick Action for this state
  //::::/c/v/z/c::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State_2::ChildState3::Grand2::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State_2::ChildState3::Grand2::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::ENDEVENT:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/z/9\033[0m");
        // Going into an end pseudo-state that is not the root end state,
        // follow its parent end transition
        if (false) {
        } else if (true) {
          _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/F\033[0m");
          // Going into a choice pseudo-state, let it handle its
          // guards and perform the state transition
          if (false) {
          } // makes geneeration easier :)
          //::::/c/v/g::::Guard::::
          else if (_root->killedState) {
            _root->log("\033[37mGUARD [ _root->killedState ] for EXTERNAL TRANSITION:/c/v/g "
                       "evaluated to TRUE\033[0m");
            // Going into an end pseudo-state that is not the root end state,
            // follow its parent end transition
            if (false) {
            } else if (true) {
              _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/F\033[0m");
              // Transitioning states!
              // Call all from prev state down exits
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
              // State : exit for: /c/v/z/c
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
              // State : exit for: /c/v/z
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
              // State : exit for: /c/v
              _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
              // External Transition : Action for: /c/v/z/9
              _root->log("\033[36mTRANSITION::ACTION for /c/v/z/9\033[0m");

              //::::/c/v/z/9::::Action::::

              // External Transition : Action for: /c/v/F
              _root->log("\033[36mTRANSITION::ACTION for /c/v/F\033[0m");

              //::::/c/v/F::::Action::::

              // External Transition : Action for: /c/v/g
              _root->log("\033[36mTRANSITION::ACTION for /c/v/g\033[0m");

              //::::/c/v/g::::Action::::

              // External Transition : Action for: /c/F
              _root->log("\033[36mTRANSITION::ACTION for /c/F\033[0m");

              //::::/c/F::::Action::::

              _root->log(
                  "\033[31mSTATE TRANSITION: State_2::ChildState3::Grand2->End_State\033[0m");

              // going into end pseudo-state THIS SHOULD BE TOP LEVEL END STATE
              _root->COMPLEX_OBJ__END_STATE_OBJ.makeActive();
              // make sure nothing else handles this event
              handled = true;
            }
          } else if (true) {
            _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/2\033[0m");
            // Transitioning states!
            // Call all from prev state down exits
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
            // State : exit for: /c/v/z/c
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
            // State : exit for: /c/v/z
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
            // External Transition : Action for: /c/v/z/9
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/9\033[0m");

            //::::/c/v/z/9::::Action::::

            // External Transition : Action for: /c/v/F
            _root->log("\033[36mTRANSITION::ACTION for /c/v/F\033[0m");

            //::::/c/v/F::::Action::::

            // External Transition : Action for: /c/v/2
            _root->log("\033[36mTRANSITION::ACTION for /c/v/2\033[0m");

            //::::/c/v/2::::Action::::

            // State : entry for: /c/v/z
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.entry();
            _root->log("\033[31mSTATE TRANSITION: "
                       "State_2::ChildState3::Grand2->State_2::ChildState3\033[0m");

            // going into regular state
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.initialize();
            // make sure nothing else handles this event
            handled = true;
          }
        }
      }
      break;
    case EventType::EVENT2:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/z/R\033[0m");
        // Going into a choice pseudo-state, let it handle its
        // guards and perform the state transition
        if (false) {
        } // makes geneeration easier :)
        //::::/c/v/z/j::::Guard::::
        else if (_root->goToEnd) {
          _root->log("\033[37mGUARD [ _root->goToEnd ] for EXTERNAL TRANSITION:/c/v/z/j evaluated "
                     "to TRUE\033[0m");
          // Going into an end pseudo-state that is not the root end state,
          // follow its parent end transition
          if (false) {
          } else if (true) {
            _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/F\033[0m");
            // Going into a choice pseudo-state, let it handle its
            // guards and perform the state transition
            if (false) {
            } // makes geneeration easier :)
            //::::/c/v/g::::Guard::::
            else if (_root->killedState) {
              _root->log("\033[37mGUARD [ _root->killedState ] for EXTERNAL TRANSITION:/c/v/g "
                         "evaluated to TRUE\033[0m");
              // Going into an end pseudo-state that is not the root end state,
              // follow its parent end transition
              if (false) {
              } else if (true) {
                _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/F\033[0m");
                // Transitioning states!
                // Call all from prev state down exits
                _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
                // State : exit for: /c/v/z/c
                _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
                // State : exit for: /c/v/z
                _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
                // State : exit for: /c/v
                _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
                // External Transition : Action for: /c/v/z/R
                _root->log("\033[36mTRANSITION::ACTION for /c/v/z/R\033[0m");

                //::::/c/v/z/R::::Action::::

                // External Transition : Action for: /c/v/z/j
                _root->log("\033[36mTRANSITION::ACTION for /c/v/z/j\033[0m");

                //::::/c/v/z/j::::Action::::

                // External Transition : Action for: /c/v/F
                _root->log("\033[36mTRANSITION::ACTION for /c/v/F\033[0m");

                //::::/c/v/F::::Action::::

                // External Transition : Action for: /c/v/g
                _root->log("\033[36mTRANSITION::ACTION for /c/v/g\033[0m");

                //::::/c/v/g::::Action::::

                // External Transition : Action for: /c/F
                _root->log("\033[36mTRANSITION::ACTION for /c/F\033[0m");

                //::::/c/F::::Action::::

                _root->log(
                    "\033[31mSTATE TRANSITION: State_2::ChildState3::Grand2->End_State\033[0m");

                // going into end pseudo-state THIS SHOULD BE TOP LEVEL END STATE
                _root->COMPLEX_OBJ__END_STATE_OBJ.makeActive();
                // make sure nothing else handles this event
                handled = true;
              }
            } else if (true) {
              _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/2\033[0m");
              // Transitioning states!
              // Call all from prev state down exits
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
              // State : exit for: /c/v/z/c
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
              // State : exit for: /c/v/z
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
              // External Transition : Action for: /c/v/z/R
              _root->log("\033[36mTRANSITION::ACTION for /c/v/z/R\033[0m");

              //::::/c/v/z/R::::Action::::

              // External Transition : Action for: /c/v/z/j
              _root->log("\033[36mTRANSITION::ACTION for /c/v/z/j\033[0m");

              //::::/c/v/z/j::::Action::::

              // External Transition : Action for: /c/v/F
              _root->log("\033[36mTRANSITION::ACTION for /c/v/F\033[0m");

              //::::/c/v/F::::Action::::

              // External Transition : Action for: /c/v/2
              _root->log("\033[36mTRANSITION::ACTION for /c/v/2\033[0m");

              //::::/c/v/2::::Action::::

              // State : entry for: /c/v/z
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.entry();
              _root->log("\033[31mSTATE TRANSITION: "
                         "State_2::ChildState3::Grand2->State_2::ChildState3\033[0m");

              // going into regular state
              _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.initialize();
              // make sure nothing else handles this event
              handled = true;
            }
          }
        }
        //::::/c/v/z/g::::Guard::::
        else if (_root->goToChoice) {
          _root->log("\033[37mGUARD [ _root->goToChoice ] for EXTERNAL TRANSITION:/c/v/z/g "
                     "evaluated to TRUE\033[0m");
          // Going into a choice pseudo-state, let it handle its
          // guards and perform the state transition
          if (false) {
          } // makes geneeration easier :)
          //::::/c/h::::Guard::::
          else if (_root->goToHistory) {
            _root->log("\033[37mGUARD [ _root->goToHistory ] for EXTERNAL TRANSITION:/c/h "
                       "evaluated to TRUE\033[0m");
            // Transitioning states!
            // Call all from prev state down exits
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
            // State : exit for: /c/v/z/c
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
            // State : exit for: /c/v/z
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
            // State : exit for: /c/v
            _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
            // External Transition : Action for: /c/v/z/R
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/R\033[0m");

            //::::/c/v/z/R::::Action::::

            // External Transition : Action for: /c/v/z/g
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/g\033[0m");

            //::::/c/v/z/g::::Action::::

            // External Transition : Action for: /c/h
            _root->log("\033[36mTRANSITION::ACTION for /c/h\033[0m");

            //::::/c/h::::Action::::

            // State : entry for: /c/T
            _root->COMPLEX_OBJ__STATE3_OBJ.entry();
            _root->log("\033[31mSTATE TRANSITION: "
                       "State_2::ChildState3::Grand2->State3::Shallow_History_Pseudostate\033[0m");

            // going into shallow history pseudo-state
            _root->COMPLEX_OBJ__STATE3_OBJ.setShallowHistory();
            // make sure nothing else handles this event
            handled = true;
          }
          //::::/c/k::::Guard::::
          else if (_root->nextState) {
            _root->log("\033[37mGUARD [ _root->nextState ] for EXTERNAL TRANSITION:/c/k evaluated "
                       "to TRUE\033[0m");
            // Transitioning states!
            // Call all from prev state down exits
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
            // State : exit for: /c/v/z/c
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
            // State : exit for: /c/v/z
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
            // State : exit for: /c/v
            _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
            // External Transition : Action for: /c/v/z/R
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/R\033[0m");

            //::::/c/v/z/R::::Action::::

            // External Transition : Action for: /c/v/z/g
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/g\033[0m");

            //::::/c/v/z/g::::Action::::

            // External Transition : Action for: /c/k
            _root->log("\033[36mTRANSITION::ACTION for /c/k\033[0m");

            //::::/c/k::::Action::::

            // State : entry for: /c/v
            _root->COMPLEX_OBJ__STATE_2_OBJ.entry();
            _root->log("\033[31mSTATE TRANSITION: State_2::ChildState3::Grand2->State_2\033[0m");

            // going into regular state
            _root->COMPLEX_OBJ__STATE_2_OBJ.initialize();
            // make sure nothing else handles this event
            handled = true;
          } else if (true) {
            _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/r\033[0m");
            // Transitioning states!
            // Call all from prev state down exits
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
            // State : exit for: /c/v/z/c
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
            // State : exit for: /c/v/z
            _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ.exit();
            // State : exit for: /c/v
            _root->COMPLEX_OBJ__STATE_2_OBJ.exit();
            // External Transition : Action for: /c/v/z/R
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/R\033[0m");

            //::::/c/v/z/R::::Action::::

            // External Transition : Action for: /c/v/z/g
            _root->log("\033[36mTRANSITION::ACTION for /c/v/z/g\033[0m");

            //::::/c/v/z/g::::Action::::

            // External Transition : Action for: /c/r
            _root->log("\033[36mTRANSITION::ACTION for /c/r\033[0m");

            //::::/c/r::::Action::::

            // State : entry for: /c/T
            _root->COMPLEX_OBJ__STATE3_OBJ.entry();
            _root->log("\033[31mSTATE TRANSITION: State_2::ChildState3::Grand2->State3\033[0m");

            // going into regular state
            _root->COMPLEX_OBJ__STATE3_OBJ.initialize();
            // make sure nothing else handles this event
            handled = true;
          }
        } else if (true) {
          _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/z/O\033[0m");
          // Transitioning states!
          // Call all from prev state down exits
          _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
          // State : exit for: /c/v/z/c
          _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
          // External Transition : Action for: /c/v/z/R
          _root->log("\033[36mTRANSITION::ACTION for /c/v/z/R\033[0m");

          //::::/c/v/z/R::::Action::::

          // External Transition : Action for: /c/v/z/O
          _root->log("\033[36mTRANSITION::ACTION for /c/v/z/O\033[0m");

          //::::/c/v/z/O::::Action::::

          // State : entry for: /c/v/z/c
          _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.entry();
          _root->log("\033[31mSTATE TRANSITION: "
                     "State_2::ChildState3::Grand2->State_2::ChildState3::Grand2\033[0m");

          // going into regular state
          _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.initialize();
          // make sure nothing else handles this event
          handled = true;
        }
      }
      break;
    case EventType::EVENT1:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/v/z/a\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exitChildren();
        // State : exit for: /c/v/z/c
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ.exit();
        // External Transition : Action for: /c/v/z/a
        _root->log("\033[36mTRANSITION::ACTION for /c/v/z/a\033[0m");

        //::::/c/v/z/a::::Action::::

        // State : entry for: /c/v/z/6
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: "
                   "State_2::ChildState3::Grand2->State_2::ChildState3::Grand\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State3 : /c/T  * * */

// User Definitions for the HFSM
//::::/c/T::::Definitions::::

void Root::State3::initialize(void) {
  // External Transition : Action for: /c/T/I
  _root->log("\033[36mTRANSITION::ACTION for /c/T/I\033[0m");

  //::::/c/T/I::::Action::::

  // State : entry for: /c/T/W
  _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ.entry();

  // initialize our new active state
  _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ.initialize();
}

void Root::State3::entry(void) {
  _root->log("\033[36mENTRY::State3::/c/T\033[0m");
  // Entry action for this state
  //::::/c/T::::Entry::::
}

void Root::State3::exit(void) {
  _root->log("\033[36mEXIT::State3::/c/T\033[0m");
  // Call the Exit Action for this state
  //::::/c/T::::Exit::::
}

void Root::State3::tick(void) {
  _root->log("\033[36mTICK::State3::/c/T\033[0m");
  // Call the Tick Action for this state
  //::::/c/T::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State3::getTimerPeriod(void) { return (double)(0); }

bool Root::State3::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::EVENT1:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::ENDEVENT:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/L\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ.exitChildren();
        // State : exit for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.exit();
        // External Transition : Action for: /c/L
        _root->log("\033[36mTRANSITION::ACTION for /c/L\033[0m");

        //::::/c/L::::Action::::

        // State : entry for: /c/Y
        _root->COMPLEX_OBJ__STATE_1_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3->State_1\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_1_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    case EventType::EVENT2:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/z\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ.exitChildren();
        // State : exit for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.exit();
        // External Transition : Action for: /c/z
        _root->log("\033[36mTRANSITION::ACTION for /c/z\033[0m");

        //::::/c/z::::Action::::

        // State : entry for: /c/v
        _root->COMPLEX_OBJ__STATE_2_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3->State_2\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE_2_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    case EventType::EVENT4:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/w\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ.exitChildren();
        // State : exit for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.exit();
        // External Transition : Action for: /c/w
        _root->log("\033[36mTRANSITION::ACTION for /c/w\033[0m");

        //::::/c/w::::Action::::

        // State : entry for: /c/v
        _root->COMPLEX_OBJ__STATE_2_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3->State_2::Deep_History_Pseudostate\033[0m");

        // going into deep history pseudo-state
        _root->COMPLEX_OBJ__STATE_2_OBJ.setDeepHistory();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    case EventType::EVENT3:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/C\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ.exitChildren();
        // State : exit for: /c/T
        _root->COMPLEX_OBJ__STATE3_OBJ.exit();
        // External Transition : Action for: /c/C
        _root->log("\033[36mTRANSITION::ACTION for /c/C\033[0m");

        //::::/c/C::::Action::::

        // State : entry for: /c/v
        _root->COMPLEX_OBJ__STATE_2_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3->State_2::Shallow_History_Pseudostate\033[0m");

        // going into shallow history pseudo-state
        _root->COMPLEX_OBJ__STATE_2_OBJ.setShallowHistory();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  // can't buble up, we are a root state.
  return handled;
}
/* * *  Definitions for State3::ChildState2 : /c/T/0  * * */

// User Definitions for the HFSM
//::::/c/T/0::::Definitions::::

void Root::State3::ChildState2::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State3::ChildState2::entry(void) {
  _root->log("\033[36mENTRY::State3::ChildState2::/c/T/0\033[0m");
  // Entry action for this state
  //::::/c/T/0::::Entry::::
}

void Root::State3::ChildState2::exit(void) {
  _root->log("\033[36mEXIT::State3::ChildState2::/c/T/0\033[0m");
  // Call the Exit Action for this state
  //::::/c/T/0::::Exit::::
}

void Root::State3::ChildState2::tick(void) {
  _root->log("\033[36mTICK::State3::ChildState2::/c/T/0\033[0m");
  // Call the Tick Action for this state
  //::::/c/T/0::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State3::ChildState2::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State3::ChildState2::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::EVENT1:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::ENDEVENT:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/T/h\033[0m");
        // Going into an end pseudo-state that is not the root end state,
        // follow its parent end transition
        if (false) {
        } else if (true) {
          _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/A\033[0m");
          // Transitioning states!
          // Call all from prev state down exits
          _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ.exitChildren();
          // State : exit for: /c/T/0
          _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ.exit();
          // State : exit for: /c/T
          _root->COMPLEX_OBJ__STATE3_OBJ.exit();
          // External Transition : Action for: /c/T/h
          _root->log("\033[36mTRANSITION::ACTION for /c/T/h\033[0m");

          //::::/c/T/h::::Action::::

          // External Transition : Action for: /c/A
          _root->log("\033[36mTRANSITION::ACTION for /c/A\033[0m");

          //::::/c/A::::Action::::

          _root->log("\033[31mSTATE TRANSITION: State3::ChildState2->End_State\033[0m");

          // going into end pseudo-state THIS SHOULD BE TOP LEVEL END STATE
          _root->COMPLEX_OBJ__END_STATE_OBJ.makeActive();
          // make sure nothing else handles this event
          handled = true;
        }
      }
      break;
    case EventType::EVENT2:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/T/j\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ.exitChildren();
        // State : exit for: /c/T/0
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ.exit();
        // External Transition : Action for: /c/T/j
        _root->log("\033[36mTRANSITION::ACTION for /c/T/j\033[0m");

        //::::/c/T/j::::Action::::

        // State : entry for: /c/T/w
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE3_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3::ChildState2->State3::ChildState3\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE3_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State3::ChildState : /c/T/W  * * */

// User Definitions for the HFSM
//::::/c/T/W::::Definitions::::

void Root::State3::ChildState::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State3::ChildState::entry(void) {
  _root->log("\033[36mENTRY::State3::ChildState::/c/T/W\033[0m");
  // Entry action for this state
  //::::/c/T/W::::Entry::::
}

void Root::State3::ChildState::exit(void) {
  _root->log("\033[36mEXIT::State3::ChildState::/c/T/W\033[0m");
  // Call the Exit Action for this state
  //::::/c/T/W::::Exit::::
}

void Root::State3::ChildState::tick(void) {
  _root->log("\033[36mTICK::State3::ChildState::/c/T/W\033[0m");
  // Call the Tick Action for this state
  //::::/c/T/W::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State3::ChildState::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State3::ChildState::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT1:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/T/L\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ.exitChildren();
        // State : exit for: /c/T/W
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ.exit();
        // External Transition : Action for: /c/T/L
        _root->log("\033[36mTRANSITION::ACTION for /c/T/L\033[0m");

        //::::/c/T/L::::Action::::

        // State : entry for: /c/T/0
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3::ChildState->State3::ChildState2\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
/* * *  Definitions for State3::ChildState3 : /c/T/w  * * */

// User Definitions for the HFSM
//::::/c/T/w::::Definitions::::

void Root::State3::ChildState3::initialize(void) {
  // if we're a leaf state, make sure we're active
  makeActive();
}

void Root::State3::ChildState3::entry(void) {
  _root->log("\033[36mENTRY::State3::ChildState3::/c/T/w\033[0m");
  // Entry action for this state
  //::::/c/T/w::::Entry::::
}

void Root::State3::ChildState3::exit(void) {
  _root->log("\033[36mEXIT::State3::ChildState3::/c/T/w\033[0m");
  // Call the Exit Action for this state
  //::::/c/T/w::::Exit::::
}

void Root::State3::ChildState3::tick(void) {
  _root->log("\033[36mTICK::State3::ChildState3::/c/T/w\033[0m");
  // Call the Tick Action for this state
  //::::/c/T/w::::Tick::::

  if (_activeState != nullptr && _activeState != this)
    _activeState->tick();
}

double Root::State3::ChildState3::getTimerPeriod(void) { return (double)(0.1); }

bool Root::State3::ChildState3::handleEvent(GeneratedEventBase *event) {
  bool handled = false;

  // take care of all event types that this branch will not handle -
  // for more consistent run-time performnace
  switch (event->get_type()) {
  case EventType::EVENT1:
    handled = true;
    break;
  default:
    handled = false;
    break;
  }

  if (handled) {
    // we didn't actually handle the event, but return anyway
    return false;
  }

  // handle internal transitions first
  switch (event->get_type()) {
  default:
    handled = false;
    break;
  }
  if (!handled) {
    // handle external transitions here
    switch (event->get_type()) {
    case EventType::EVENT3:
      if (false) {
      } // makes generation easier :)
      else if (true) {
        _root->log("\033[37mNO GUARD on EXTERNAL TRANSITION:/c/T/p\033[0m");
        // Transitioning states!
        // Call all from prev state down exits
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE3_OBJ.exitChildren();
        // State : exit for: /c/T/w
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE3_OBJ.exit();
        // External Transition : Action for: /c/T/p
        _root->log("\033[36mTRANSITION::ACTION for /c/T/p\033[0m");

        //::::/c/T/p::::Action::::

        // State : entry for: /c/T/W
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ.entry();
        _root->log("\033[31mSTATE TRANSITION: State3::ChildState3->State3::ChildState\033[0m");

        // going into regular state
        _root->COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ.initialize();
        // make sure nothing else handles this event
        handled = true;
      }
      break;
    default:
      handled = false;
      break;
    }
  }
  if (!handled) {
    // now check parent states
    handled = _parentState->handleEvent(event);
  }
  return handled;
}
