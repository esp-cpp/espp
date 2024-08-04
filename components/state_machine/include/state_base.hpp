#pragma once
#include <string>

namespace espp {
namespace state_machine {

// Base Class for Events, abstract so you never instantiate.
class EventBase {
public:
  /// Default constructor
  virtual ~EventBase() {}
  /// Returns a string representation of the event
  virtual std::string to_string() const = 0;
}; // class EventBase

/**
 * States contain other states and can consume generic
 * EventBase objects if they have internal or external
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
  /**
   * @brief Default constructor
   */
  StateBase();

  /**
   * @brief Constructor that sets the parent state.
   * @param[in] parent Pointer to parent state
   */
  explicit StateBase(StateBase *parent);

  /**
   * @brief Destructor
   */
  virtual ~StateBase(void);

  /**
   * @brief Will be generated to call entry() then handle any child
   *  initialization. Finally calls makeActive on the leaf.
   */
  virtual void initialize(void);

  /**
   * @brief Will be generated to run the entry() function defined in
   *  the model.
   */
  virtual void entry(void);

  /**
   * @brief Will be generated to run the exit() function defined in
   *   the model.
   */
  virtual void exit(void);

  /**
   * @brief Calls handleEvent on the activeLeaf.
   * @param[in] event Event needing to be handled
   * @return true if event is consumed, false otherwise
   */
  virtual bool handleEvent(EventBase *event);

  /**
   * @brief Will be generated to run the tick() function defined in
   *  the model and then call _activeState->tick().
   */
  virtual void tick(void);

  /**
   * @brief Returns the timer period for the state.
   */
  virtual double getTimerPeriod(void);

  /**
   * @brief Will be known from the model so will be generated in
   *  derived classes to immediately return the correct initial
   *  state pointer for quickly transitioning to the proper state
   *  during external transition handling.
   * @return Pointer to initial substate
   */
  virtual StateBase *getInitial(void);

  /**
   * @brief Recurses down to the leaf state and calls the exit
   *  actions as it unwinds.
   */
  void exitChildren(void);

  /**
   * @brief Will return _activeState if it exists, otherwise will
   *  return nullptr.
   * @return Pointer to last active substate
   */
  StateBase *getActiveChild(void);

  /**
   * @brief Will return the active leaf state, otherwise will return
   *  nullptr.
   * @return Pointer to last active leaf state.
   */
  StateBase *getActiveLeaf(void);

  /**
   * @brief Make this state the active substate of its parent and
   *  then recurse up through the tree to the root.
   * @note Should only be called on leaf nodes!
   */
  virtual void makeActive(void);

  /**
   * @brief Update the active child state.
   */
  void setActiveChild(StateBase *childState);

  /**
   * @brief Sets the currentlyActive state to the last active state
   *  and re-initializes them.
   */
  void setShallowHistory(void);

  /**
   * @brief Go to the last active leaf of this state. If none
   *  exists, re-initialize.
   */
  void setDeepHistory(void);

  /**
   * @brief Will set the parent state.
   * @param[in] parent Pointer to parent state
   */
  void setParentState(StateBase *parent);

  /**
   * @brief Will return the parent state.
   */
  StateBase *getParentState(void);

  // Pointer to the currently or most recently active substate of this
  // state.
  StateBase *_activeState;

  // Pointer to the parent state of this state.
  StateBase *_parentState;
}; // class StateBase

} // namespace state_machine
} // namespace espp
