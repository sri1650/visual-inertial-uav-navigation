#include "vionav/calibration.hpp"
#include <cassert>
#include <stdexcept>

int main() {
  using namespace vionav;
  SensorCalibration c;
  c.camera = {400,401,320,240,640,480,"radtan",{}};
  c.T_body_camera.translation_m = {0.1,0,0};
  c.validate();

  bool threw=false;
  try {
    auto bad=c;
    bad.camera.fx=0;
    bad.validate();
  } catch (const std::invalid_argument&) { threw=true; }
  assert(threw);

  threw=false;
  try {
    auto bad=c;
    bad.T_body_camera.rotation[1]={{1,1,0}};
    bad.validate();
  } catch (const std::invalid_argument&) { threw=true; }
  assert(threw);
}
