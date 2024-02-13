#pragma once

#include "state_base.hpp"

namespace espp::state_machine {

/**
 * @brief Deep History Pseudostates exist purely to re-implement the
 *  makeActive() function to actually call
 *  _parentState->setDeepHistory()
 */
class DeepHistoryState : public StateBase {
public:
  DeepHistoryState()
      : StateBase() {}
  explicit DeepHistoryState(StateBase *_parent)
      : StateBase(_parent) {}

  /**
   * @brief Calls _parentState->setDeepHistory()
   */
  virtual void makeActive() override {
    if (_parentState) {
      _parentState->setDeepHistory();
    }
  }
};
} // namespace espp::state_machine
