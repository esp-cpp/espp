#pragma once

#include "state_base.hpp"

namespace espp {
namespace state_machine {

/**
 * @brief Deep History Pseudostates exist purely to re-implement the
 *  makeActive() function to actually call
 *  _parentState->setDeepHistory()
 */
class DeepHistoryState : public StateBase {
public:
  /**
   * @brief Construct a new Deep History State object
   */
  DeepHistoryState()
      : StateBase() {}

  /**
   * @brief Construct a new Deep History State object
   * @param _parent The parent state of this state
   */
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
} // namespace state_machine
} // namespace espp
