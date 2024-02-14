#pragma once

#include "state_base.hpp"

namespace espp::state_machine {

/**
 * @brief Shallow History Pseudostates exist purely to re-implement
 *  the makeActive() function to actually call
 *  _parentState->setShallowHistory()
 */
class ShallowHistoryState : public StateBase {
public:
  ShallowHistoryState()
      : StateBase() {}
  explicit ShallowHistoryState(StateBase *_parent)
      : StateBase(_parent) {}

  /**
   * @brief Calls _parentState->setShallowHistory().
   */
  virtual void makeActive() override {
    if (_parentState) {
      _parentState->setShallowHistory();
    }
  }
};
} // namespace espp::state_machine
