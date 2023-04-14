#pragma once
#include <string>

namespace espp {
namespace state_machine {

// Base Class for Events, abstract so you never instantiate.
class EventBase {
public:
  virtual ~EventBase() {}
  virtual std::string to_string() const = 0;
}; // class EventBase

/**
 * States contain other states and can consume generic
 * state_machine::EventBase objects if they have internal or external
 * transitions on those events and if those transitions' guards are
 * satisfied. Only one transition can consume an event in a given
 * state machine.
 *
 * There is also a different kind of Event, the tick event, which is
 * not consumed, but instead executes from the top-level state all
 * the way to the curently active leaf state.
 *
 * Entry and Exit actions also occur whenever a state is entered or
 * exited, respectively.
 */
class StateBase {
public:
  StateBase() : _activeState(this), _parentState(nullptr) {}
  StateBase(StateBase *parent) : _activeState(this), _parentState(parent) {}
  ~StateBase(void) {}

  /**
   * @brief Will be generated to call entry() then handle any child
   *  initialization. Finally calls makeActive on the leaf.
   */
  virtual void initialize(void){};

  /**
   * @brief Will be generated to run the entry() function defined in
   *  the model.
   */
  virtual void entry(void){};

  /**
   * @brief Will be generated to run the exit() function defined in
   *   the model.
   */
  virtual void exit(void){};

  /**
   * @brief Calls handleEvent on the activeLeaf.
   *
   * @param[in] event EventBase* Event needing to be handled
   *
   * @return true if event is consumed, false otherwise
   */
  virtual bool handleEvent(EventBase *event) { return false; }

  /**
   * @brief Will be generated to run the tick() function defined in
   *  the model and then call _activeState->tick().
   */
  virtual void tick(void) {
    if (_activeState != this && _activeState != nullptr)
      _activeState->tick();
  };

  /**
   */
  virtual double getTimerPeriod(void) { return 0; }

  /**
   * @brief Will be known from the model so will be generated in
   *  derived classes to immediately return the correct initial
   *  state pointer for quickly transitioning to the proper state
   *  during external transition handling.
   *
   * @return StateBase*  Pointer to initial substate
   */
  virtual StateBase *getInitial(void) { return this; };

  /**
   * @brief Recurses down to the leaf state and calls the exit
   *  actions as it unwinds.
   */
  void exitChildren(void) {
    if (_activeState != nullptr && _activeState != this) {
      _activeState->exitChildren();
      _activeState->exit();
    }
  }

  /**
   * @brief Will return _activeState if it exists, otherwise will
   *  return nullptr.
   *
   * @return StateBase*  Pointer to last active substate
   */
  StateBase *getActiveChild(void) { return _activeState; }

  /**
   * @brief Will return the active leaf state, otherwise will return
   *  nullptr.
   *
   * @return StateBase*  Pointer to last active leaf state.
   */
  StateBase *getActiveLeaf(void) {
    if (_activeState != nullptr && _activeState != this)
      return _activeState->getActiveLeaf();
    else
      return this;
  }

  /**
   * @brief Make this state the active substate of its parent and
   *  then recurse up through the tree to the root.
   *
   *  *Should only be called on leaf nodes!*
   */
  void makeActive(void) {
    if (_parentState != nullptr) {
      _parentState->setActiveChild(this);
      _parentState->makeActive();
    }
  }

  /**
   * @brief Update the active child state.
   * @param childState New child state to set to be active state.
   */
  void setActiveChild(StateBase *childState) { _activeState = childState; }

  /**
   * @brief Sets the currentlyActive state to the last active state
   *  and re-initializes them.
   */
  void setShallowHistory(void) {
    if (_activeState != nullptr && _activeState != this) {
      _activeState->entry();
      _activeState->initialize();
    } else {
      initialize();
    }
  }

  /**
   * @brief Go to the last active leaf of this state. If none
   *  exists, re-initialize.
   */
  void setDeepHistory(void) {
    if (_activeState != nullptr && _activeState != this) {
      _activeState->entry();
      _activeState->setDeepHistory();
    } else {
      initialize();
    }
  }

  /**
   * @brief Will set the parent state.
   * @param parent Set the parent state of this state.
   */
  void setParentState(StateBase *parent) { _parentState = parent; }

  /**
   * @brief Will return the parent state.
   */
  StateBase *getParentState(void) { return _parentState; }

  // Pointer to the currently or most recently active substate of this
  // state.
  StateBase *_activeState;

  // Pointer to the parent state of this state.
  StateBase *_parentState;
}; // class StateBase

} // namespace state_machine
} // namespace espp
