#include "vionav/estimator_state.hpp"
#include <cassert>

int main() {
  using namespace vionav;
  TrackingStateMachine sm;
  assert(sm.state()==TrackingState::INITIALISING);
  assert(sm.update(false,100,1.0,false,false)==TrackingState::INITIALISING);
  assert(sm.update(true,100,1.0,false,false)==TrackingState::TRACKING);
  assert(sm.update(true,30,5.0,false,false)==TrackingState::DEGRADED);
  assert(sm.update(true,10,5.0,false,false)==TrackingState::LOST);
  sm.begin_reset();
  assert(sm.state()==TrackingState::RESETTING);
  assert(sm.update(true,100,1.0,false,false)==TrackingState::RESETTING);
  sm.reset_complete(false);
  assert(sm.state()==TrackingState::INITIALISING);
  sm.reset_complete(true);
  assert(sm.state()==TrackingState::TRACKING);
}
