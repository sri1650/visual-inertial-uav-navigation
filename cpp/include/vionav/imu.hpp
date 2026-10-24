#pragma once
#include "vionav/types.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace vionav {

class ImuBuffer {
 public:
  explicit ImuBuffer(std::size_t capacity=4096) : capacity_(capacity) {}
  void push(const ImuSample& sample);
  std::size_t size() const { return samples_.size(); }
  ImuSample interpolate(std::int64_t timestamp_ns) const;
  std::vector<ImuSample> range(std::int64_t start_ns, std::int64_t end_ns) const;

 private:
  std::size_t capacity_;
  std::deque<ImuSample> samples_;
};

struct PreintegratedDelta {
  double dt_s{};
  Quaternion delta_rotation{};
  Vec3 delta_velocity{};
  Vec3 delta_position{};
};

PreintegratedDelta preintegrate(
    const std::vector<ImuSample>& samples,
    const ImuBias& bias = {});

Vec3 estimate_gravity_from_stationary(
    const std::vector<ImuSample>& samples,
    double gravity_m_s2 = 9.80665);

} // namespace vionav
