#include "state_base.hpp"

using namespace espp::state_machine;

StateBase::StateBase()
    : _activeState(this)
    , _parentState(nullptr) {}

StateBase::StateBase(StateBase *parent)
    : _activeState(this)
    , _parentState(parent) {}

StateBase::~StateBase(void) {}

void StateBase::initialize(void){};

void StateBase::entry(void){};

void StateBase::exit(void){};

bool StateBase::handleEvent(EventBase *event) { return false; }

void StateBase::tick(void) {
  if (_activeState != this && _activeState != nullptr)
    _activeState->tick();
};

double StateBase::getTimerPeriod(void) { return 0; }

StateBase *StateBase::getInitial(void) { return this; };

void StateBase::exitChildren(void) {
  if (_activeState != nullptr && _activeState != this) {
    _activeState->exitChildren();
    _activeState->exit();
  }
}

StateBase *StateBase::getActiveChild(void) { return _activeState; }

StateBase *StateBase::getActiveLeaf(void) {
  if (_activeState != nullptr && _activeState != this)
    return _activeState->getActiveLeaf();
  else
    return this;
}

void StateBase::makeActive(void) {
  if (_parentState != nullptr) {
    _parentState->setActiveChild(this);
    _parentState->makeActive();
  }
}

void StateBase::setActiveChild(StateBase *childState) { _activeState = childState; }

void StateBase::setShallowHistory(void) {
  if (_activeState != nullptr && _activeState != this) {
    _activeState->entry();
    _activeState->initialize();
  } else {
    initialize();
  }
}

void StateBase::setDeepHistory(void) {
  if (_activeState != nullptr && _activeState != this) {
    _activeState->entry();
    _activeState->setDeepHistory();
  } else {
    initialize();
  }
}

void StateBase::setParentState(StateBase *parent) { _parentState = parent; }

StateBase *StateBase::getParentState(void) { return _parentState; }
