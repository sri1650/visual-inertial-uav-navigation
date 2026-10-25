#include "vionav/quality.hpp"
#include <cassert>

int main() {
  using namespace vionav;
  auto good=assess_quality({120,.9,1.0,1.0,2.0});
  assert(good.state==TrackingState::TRACKING);
  assert(good.score==100.0);

  auto lost=assess_quality({120,.9,1.0,40.0,2.0});
  assert(lost.state==TrackingState::LOST);

  RecoverySupervisor supervisor(2,2);
  assert(supervisor.observe(lost)==RecoveryAction::HOLD_LOST);
  assert(supervisor.observe(lost)==RecoveryAction::RESET);
  assert(supervisor.reset_count()==1);
  assert(supervisor.observe(good)==RecoveryAction::RECOVERING);
  assert(supervisor.observe(good)==RecoveryAction::RECOVERED);

  auto innovation_lost=assess_quality({120,.9,1.0,1.0,2.0,30.0});
  assert(innovation_lost.state==TrackingState::LOST);
  assert(innovation_lost.reasons.back()=="innovation_outlier");
}
