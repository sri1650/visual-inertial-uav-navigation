#include "vionav/imu.hpp"
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

static bool near(double a,double b,double eps=1e-6){ return std::abs(a-b)<eps; }

int main() {
  using namespace vionav;

  ImuBuffer b(10);
  b.push({0,{0,0,0},{0,0,0}});
  b.push({1000000000LL,{0,0,2},{2,0,0}});
  auto mid=b.interpolate(500000000LL);
  assert(near(mid.angular_velocity_rad_s.z,1.0));
  assert(near(mid.linear_acceleration_m_s2.x,1.0));

  std::vector<ImuSample> constant_acc = {
    {0,{0,0,0},{1,0,0}},
    {1000000000LL,{0,0,0},{1,0,0}},
    {2000000000LL,{0,0,0},{1,0,0}},
  };
  auto d=preintegrate(constant_acc);
  assert(near(d.dt_s,2.0));
  assert(near(d.delta_velocity.x,2.0));
  assert(near(d.delta_position.x,2.0));

  std::vector<ImuSample> stationary = {
    {0,{0,0,0},{0,0,9.80665}},
    {1000000,{0,0,0},{0,0,9.80665}},
  };
  auto g=estimate_gravity_from_stationary(stationary);
  assert(near(g.z,-9.80665,1e-5));

  bool threw=false;
  try { b.push({500000000LL,{},{}}); }
  catch(const std::invalid_argument&) { threw=true; }
  assert(threw);
}
