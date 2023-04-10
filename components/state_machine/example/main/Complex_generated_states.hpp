#pragma once

#include "deep_history_state.hpp"
#include "shallow_history_state.hpp"
#include "state_base.hpp"

#include "Complex_event_data.hpp"
#include "magic_enum.hpp"
#include <deque>
#include <mutex>
#include <string>

#ifdef DEBUG_OUTPUT
#include <iostream>
#endif

// User Includes for the HFSM
//::::/c::::Includes::::
#include <stdio.h>

namespace espp {
namespace state_machine {

namespace Complex {

enum class EventType {
  ENDEVENT,
  EVENT1,
  EVENT2,
  EVENT3,
  EVENT4,
}; // ENUMS GENERATED FROM MODEL

/**
 * @brief Class representing all events that this HFSM can respond
 * to / handle. Used as abstract interface for handleEvent().
 */
class GeneratedEventBase : public EventBase {
protected:
  EventType type;

public:
  explicit GeneratedEventBase(const EventType &t) : type(t) {}
  virtual ~GeneratedEventBase() {}
  EventType get_type() const { return type; }
  virtual std::string to_string() const { return std::string(magic_enum::enum_name(type)); }
}; // Class GeneratedEventBase

/**
 * @brief Class representing all events that this HFSM can respond
 * to / handle. Intended to be created / managed by the
 * EventFactory (below).
 */
template <typename T> class Event : public GeneratedEventBase {
  T data;

public:
  explicit Event(const EventType &t, const T &d) : GeneratedEventBase(t), data(d) {}
  virtual ~Event() {}
  T get_data() const { return data; }
}; // Class Event

// free the memory associated with the event
static void consume_event(GeneratedEventBase *e) { delete e; }

typedef Event<ENDEVENTEventData> ENDEVENTEvent;
typedef Event<EVENT1EventData> EVENT1Event;
typedef Event<EVENT2EventData> EVENT2Event;
typedef Event<EVENT3EventData> EVENT3Event;
typedef Event<EVENT4EventData> EVENT4Event;

/**
 * @brief Class handling all Event creation, memory management, and
 *  ordering.
 */
class EventFactory {
public:
  ~EventFactory(void) { clear_events(); }

  void spawn_ENDEVENT_event(const ENDEVENTEventData &data) {
#ifdef DEBUG_OUTPUT
    std::cout << "\033[32mSPAWN: ENDEVENT\033[0m" << std::endl;
#endif
    GeneratedEventBase *new_event = new ENDEVENTEvent{EventType::ENDEVENT, data};
    std::lock_guard<std::mutex> lock(queue_mutex_);
    _eventQ.push_back(new_event);
  }
  void spawn_EVENT1_event(const EVENT1EventData &data) {
#ifdef DEBUG_OUTPUT
    std::cout << "\033[32mSPAWN: EVENT1\033[0m" << std::endl;
#endif
    GeneratedEventBase *new_event = new EVENT1Event{EventType::EVENT1, data};
    std::lock_guard<std::mutex> lock(queue_mutex_);
    _eventQ.push_back(new_event);
  }
  void spawn_EVENT2_event(const EVENT2EventData &data) {
#ifdef DEBUG_OUTPUT
    std::cout << "\033[32mSPAWN: EVENT2\033[0m" << std::endl;
#endif
    GeneratedEventBase *new_event = new EVENT2Event{EventType::EVENT2, data};
    std::lock_guard<std::mutex> lock(queue_mutex_);
    _eventQ.push_back(new_event);
  }
  void spawn_EVENT3_event(const EVENT3EventData &data) {
#ifdef DEBUG_OUTPUT
    std::cout << "\033[32mSPAWN: EVENT3\033[0m" << std::endl;
#endif
    GeneratedEventBase *new_event = new EVENT3Event{EventType::EVENT3, data};
    std::lock_guard<std::mutex> lock(queue_mutex_);
    _eventQ.push_back(new_event);
  }
  void spawn_EVENT4_event(const EVENT4EventData &data) {
#ifdef DEBUG_OUTPUT
    std::cout << "\033[32mSPAWN: EVENT4\033[0m" << std::endl;
#endif
    GeneratedEventBase *new_event = new EVENT4Event{EventType::EVENT4, data};
    std::lock_guard<std::mutex> lock(queue_mutex_);
    _eventQ.push_back(new_event);
  }

  // Retrieves the pointer to the next event in the queue, or
  // nullptr if it doesn't exist
  GeneratedEventBase *get_next_event(void) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    GeneratedEventBase *ptr = nullptr;
    if (_eventQ.size()) {
      ptr = _eventQ.front();
      _eventQ.pop_front(); // remove the event from the Q
    }
    return ptr;
  }

  // Clears the event queue and frees all event memory
  void clear_events(void) {
    GeneratedEventBase *ptr = get_next_event();
    while (ptr != nullptr) {
      consume_event(ptr);
      ptr = get_next_event();
    }
  }

  std::string to_string(void) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::string qStr = "[ ";
    for (int i = 0; i < _eventQ.size(); i++) {
      qStr += _eventQ[i]->to_string();
    }
    qStr += " ]";
    return qStr;
  }

protected:
  std::deque<GeneratedEventBase *> _eventQ;
  std::mutex queue_mutex_;
}; // class EventFactory

/**
 * @brief The ROOT of the HFSM - contains the declarations from
 *  the user as well as the entire substate tree.
 */
class Root : public StateBase {
public:
  // User Declarations for the HFSM
  //::::/c::::Declarations::::
  bool goToEnd = false;
  bool goToChoice = true;
  bool goToHistory = false;
  bool nextState = false;
  bool killedState = false;
  bool someGuard = true;
  bool someTest = true;

  int someNumber = 40;
  int someValue = 50;

public:
  // event factory for spawning / ordering events
  EventFactory event_factory;

  // helper functions for spawning events into the HFSM
  void spawn_ENDEVENT_event(const ENDEVENTEventData &data) {
    event_factory.spawn_ENDEVENT_event(data);
  }
  void spawn_EVENT1_event(const EVENT1EventData &data) { event_factory.spawn_EVENT1_event(data); }
  void spawn_EVENT2_event(const EVENT2EventData &data) { event_factory.spawn_EVENT2_event(data); }
  void spawn_EVENT3_event(const EVENT3EventData &data) { event_factory.spawn_EVENT3_event(data); }
  void spawn_EVENT4_event(const EVENT4EventData &data) { event_factory.spawn_EVENT4_event(data); }

  // Constructors
  Root()
      : StateBase(), COMPLEX_OBJ__STATE_1_OBJ(this, this),
        COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ(this, &COMPLEX_OBJ__STATE_2_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ__DEEP_HISTORY_PSEUDOSTATE_OBJ(&COMPLEX_OBJ__STATE_2_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE2_OBJ(this, &COMPLEX_OBJ__STATE_2_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ(
            this, &COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ(
            this, &COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ(this, &COMPLEX_OBJ__STATE_2_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ__SHALLOW_HISTORY_PSEUDOSTATE_OBJ(&COMPLEX_OBJ__STATE_2_OBJ),
        COMPLEX_OBJ__STATE_2_OBJ(this, this),
        COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ(this, &COMPLEX_OBJ__STATE3_OBJ),
        COMPLEX_OBJ__STATE3_OBJ__SHALLOW_HISTORY_PSEUDOSTATE_OBJ(&COMPLEX_OBJ__STATE3_OBJ),
        COMPLEX_OBJ__STATE3_OBJ__DEEP_HISTORY_PSEUDOSTATE_OBJ(&COMPLEX_OBJ__STATE3_OBJ),
        COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ(this, &COMPLEX_OBJ__STATE3_OBJ),
        COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE3_OBJ(this, &COMPLEX_OBJ__STATE3_OBJ),
        COMPLEX_OBJ__STATE3_OBJ(this, this), COMPLEX_OBJ__END_STATE_OBJ(this), _root(this) {}
  ~Root(void) {}

  /**
   * @brief Fully initializes the HFSM. Runs the HFSM Initialization
   *  code from the model, then sets the inital state and runs the
   *  initial transition and entry actions accordingly.
   */
  void initialize(void);

  /**
   * @brief Handles all events in the event queue, ensuring to free the
   * memory. This will ensure that any events spawned from other event
   * transitions / actions are handled. Returns once there are no more
   * events in the queue to process.
   */
  void handle_all_events(void);

  /**
   * @brief Terminates the HFSM, calling exit functions for the
   *  active leaf state upwards through its parents all the way to
   *  the root.
   */
  void terminate(void);

  /**
   * @brief Restarts the HFSM by calling terminate and then
   *  initialize.
   */
  void restart(void);

  /**
   * @brief Returns true if the HFSM has reached its END State
   */
  bool has_stopped(void);

  /**
   * @brief Calls handleEvent on the activeLeaf.
   *
   * @param[in] EventBase* Event needing to be handled
   *
   * @return true if event is consumed, false otherwise
   */
  bool handleEvent(EventBase *event) {
    return handleEvent(static_cast<GeneratedEventBase *>(event));
  }

  /**
   * @brief Calls handleEvent on the activeLeaf.
   *
   * @param[in] EventBase* Event needing to be handled
   *
   * @return true if event is consumed, false otherwise
   */
  bool handleEvent(GeneratedEventBase *event);

  // Child Substates
  // Declaration for State_1 : /c/Y
  class State_1 : public state_machine::StateBase {
  public:
    // User Declarations for the State
    //::::/c/Y::::Declarations::::

  public:
    // Pointer to the root of the HFSM.
    Root *_root;

    // Constructors
    State_1(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
    ~State_1(void) {}

    // StateBase Interface
    void initialize(void);
    void entry(void);
    void exit(void);
    void tick(void);
    double getTimerPeriod(void);
    virtual bool handleEvent(EventBase *event) {
      return handleEvent(static_cast<GeneratedEventBase *>(event));
    }
    virtual bool handleEvent(GeneratedEventBase *event);
  };
  // Declaration for State_2 : /c/v
  class State_2 : public state_machine::StateBase {
  public:
    // User Declarations for the State
    //::::/c/v::::Declarations::::

  public:
    // Pointer to the root of the HFSM.
    Root *_root;

    // Constructors
    State_2(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
    ~State_2(void) {}

    // StateBase Interface
    void initialize(void);
    void entry(void);
    void exit(void);
    void tick(void);
    double getTimerPeriod(void);
    virtual bool handleEvent(EventBase *event) {
      return handleEvent(static_cast<GeneratedEventBase *>(event));
    }
    virtual bool handleEvent(GeneratedEventBase *event);

    // Declaration for State_2::ChildState : /c/v/K
    class ChildState : public state_machine::StateBase {
    public:
      // User Declarations for the State
      //::::/c/v/K::::Declarations::::

    public:
      // Pointer to the root of the HFSM.
      Root *_root;

      // Constructors
      ChildState(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
      ~ChildState(void) {}

      // StateBase Interface
      void initialize(void);
      void entry(void);
      void exit(void);
      void tick(void);
      double getTimerPeriod(void);
      virtual bool handleEvent(EventBase *event) {
        return handleEvent(static_cast<GeneratedEventBase *>(event));
      }
      virtual bool handleEvent(GeneratedEventBase *event);
    };
    // Declaration for State_2::ChildState2 : /c/v/e
    class ChildState2 : public state_machine::StateBase {
    public:
      // User Declarations for the State
      //::::/c/v/e::::Declarations::::

    public:
      // Pointer to the root of the HFSM.
      Root *_root;

      // Constructors
      ChildState2(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
      ~ChildState2(void) {}

      // StateBase Interface
      void initialize(void);
      void entry(void);
      void exit(void);
      void tick(void);
      double getTimerPeriod(void);
      virtual bool handleEvent(EventBase *event) {
        return handleEvent(static_cast<GeneratedEventBase *>(event));
      }
      virtual bool handleEvent(GeneratedEventBase *event);
    };
    // Declaration for State_2::ChildState3 : /c/v/z
    class ChildState3 : public state_machine::StateBase {
    public:
      // User Declarations for the State
      //::::/c/v/z::::Declarations::::

    public:
      // Pointer to the root of the HFSM.
      Root *_root;

      // Constructors
      ChildState3(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
      ~ChildState3(void) {}

      // StateBase Interface
      void initialize(void);
      void entry(void);
      void exit(void);
      void tick(void);
      double getTimerPeriod(void);
      virtual bool handleEvent(EventBase *event) {
        return handleEvent(static_cast<GeneratedEventBase *>(event));
      }
      virtual bool handleEvent(GeneratedEventBase *event);

      // Declaration for State_2::ChildState3::Grand : /c/v/z/6
      class Grand : public state_machine::StateBase {
      public:
        // User Declarations for the State
        //::::/c/v/z/6::::Declarations::::

      public:
        // Pointer to the root of the HFSM.
        Root *_root;

        // Constructors
        Grand(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
        ~Grand(void) {}

        // StateBase Interface
        void initialize(void);
        void entry(void);
        void exit(void);
        void tick(void);
        double getTimerPeriod(void);
        virtual bool handleEvent(EventBase *event) {
          return handleEvent(static_cast<GeneratedEventBase *>(event));
        }
        virtual bool handleEvent(GeneratedEventBase *event);
      };
      // Declaration for State_2::ChildState3::Grand2 : /c/v/z/c
      class Grand2 : public state_machine::StateBase {
      public:
        // User Declarations for the State
        //::::/c/v/z/c::::Declarations::::

      public:
        // Pointer to the root of the HFSM.
        Root *_root;

        // Constructors
        Grand2(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
        ~Grand2(void) {}

        // StateBase Interface
        void initialize(void);
        void entry(void);
        void exit(void);
        void tick(void);
        double getTimerPeriod(void);
        virtual bool handleEvent(EventBase *event) {
          return handleEvent(static_cast<GeneratedEventBase *>(event));
        }
        virtual bool handleEvent(GeneratedEventBase *event);
      };
    };
  };
  // Declaration for State3 : /c/T
  class State3 : public state_machine::StateBase {
  public:
    // User Declarations for the State
    //::::/c/T::::Declarations::::

  public:
    // Pointer to the root of the HFSM.
    Root *_root;

    // Constructors
    State3(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
    ~State3(void) {}

    // StateBase Interface
    void initialize(void);
    void entry(void);
    void exit(void);
    void tick(void);
    double getTimerPeriod(void);
    virtual bool handleEvent(EventBase *event) {
      return handleEvent(static_cast<GeneratedEventBase *>(event));
    }
    virtual bool handleEvent(GeneratedEventBase *event);

    // Declaration for State3::ChildState2 : /c/T/0
    class ChildState2 : public state_machine::StateBase {
    public:
      // User Declarations for the State
      //::::/c/T/0::::Declarations::::

    public:
      // Pointer to the root of the HFSM.
      Root *_root;

      // Constructors
      ChildState2(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
      ~ChildState2(void) {}

      // StateBase Interface
      void initialize(void);
      void entry(void);
      void exit(void);
      void tick(void);
      double getTimerPeriod(void);
      virtual bool handleEvent(EventBase *event) {
        return handleEvent(static_cast<GeneratedEventBase *>(event));
      }
      virtual bool handleEvent(GeneratedEventBase *event);
    };
    // Declaration for State3::ChildState : /c/T/W
    class ChildState : public state_machine::StateBase {
    public:
      // User Declarations for the State
      //::::/c/T/W::::Declarations::::

    public:
      // Pointer to the root of the HFSM.
      Root *_root;

      // Constructors
      ChildState(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
      ~ChildState(void) {}

      // StateBase Interface
      void initialize(void);
      void entry(void);
      void exit(void);
      void tick(void);
      double getTimerPeriod(void);
      virtual bool handleEvent(EventBase *event) {
        return handleEvent(static_cast<GeneratedEventBase *>(event));
      }
      virtual bool handleEvent(GeneratedEventBase *event);
    };
    // Declaration for State3::ChildState3 : /c/T/w
    class ChildState3 : public state_machine::StateBase {
    public:
      // User Declarations for the State
      //::::/c/T/w::::Declarations::::

    public:
      // Pointer to the root of the HFSM.
      Root *_root;

      // Constructors
      ChildState3(Root *root, StateBase *parent) : StateBase(parent), _root(root) {}
      ~ChildState3(void) {}

      // StateBase Interface
      void initialize(void);
      void entry(void);
      void exit(void);
      void tick(void);
      double getTimerPeriod(void);
      virtual bool handleEvent(EventBase *event) {
        return handleEvent(static_cast<GeneratedEventBase *>(event));
      }
      virtual bool handleEvent(GeneratedEventBase *event);
    };
  };

  // END STATE
  /**
   * @brief This is the terminal END STATE for the HFSM, after which no
   *  events or other actions will be processed.
   */
  class End_State : public state_machine::StateBase {
  public:
    explicit End_State(StateBase *parent) : StateBase(parent) {}
    void entry(void) {}
    void exit(void) {}
    void tick(void) {}
    // Simply returns true since the END STATE trivially handles all
    // events.
    bool handleEvent(state_machine::EventBase *event) { return true; }
    bool handleEvent(GeneratedEventBase *event) { return true; }
  };

  // State Objects
  State_1 COMPLEX_OBJ__STATE_1_OBJ;
  State_2::ChildState COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE_OBJ;
  state_machine::DeepHistoryState COMPLEX_OBJ__STATE_2_OBJ__DEEP_HISTORY_PSEUDOSTATE_OBJ;
  State_2::ChildState2 COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE2_OBJ;
  State_2::ChildState3::Grand COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND_OBJ;
  State_2::ChildState3::Grand2 COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ__GRAND2_OBJ;
  State_2::ChildState3 COMPLEX_OBJ__STATE_2_OBJ__CHILDSTATE3_OBJ;
  state_machine::ShallowHistoryState COMPLEX_OBJ__STATE_2_OBJ__SHALLOW_HISTORY_PSEUDOSTATE_OBJ;
  State_2 COMPLEX_OBJ__STATE_2_OBJ;
  State3::ChildState2 COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE2_OBJ;
  state_machine::ShallowHistoryState COMPLEX_OBJ__STATE3_OBJ__SHALLOW_HISTORY_PSEUDOSTATE_OBJ;
  state_machine::DeepHistoryState COMPLEX_OBJ__STATE3_OBJ__DEEP_HISTORY_PSEUDOSTATE_OBJ;
  State3::ChildState COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE_OBJ;
  State3::ChildState3 COMPLEX_OBJ__STATE3_OBJ__CHILDSTATE3_OBJ;
  State3 COMPLEX_OBJ__STATE3_OBJ;
  // END state object
  End_State COMPLEX_OBJ__END_STATE_OBJ;
  // Keep a _root for easier templating, it will point to us
  Root *_root;
}; // class Root

}; // namespace Complex
} // namespace state_machine
} // namespace espp
