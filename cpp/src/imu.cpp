#include "vionav/imu.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vionav {

static Vec3 lerp(const Vec3& a, const Vec3& b, double u) {
  return a*(1.0-u) + b*u;
}

void ImuBuffer::push(const ImuSample& s) {
  if (!samples_.empty() && s.timestamp_ns <= samples_.back().timestamp_ns)
    throw std::invalid_argument("IMU timestamps must be strictly increasing");
  samples_.push_back(s);
  while (samples_.size() > capacity_) samples_.pop_front();
}

ImuSample ImuBuffer::interpolate(std::int64_t t) const {
  if (samples_.size() < 2) throw std::out_of_range("insufficient samples");
  if (t < samples_.front().timestamp_ns || t > samples_.back().timestamp_ns)
    throw std::out_of_range("timestamp outside buffer");
  auto it = std::lower_bound(samples_.begin(), samples_.end(), t,
      [](const ImuSample& s, std::int64_t value){ return s.timestamp_ns < value; });
  if (it != samples_.end() && it->timestamp_ns == t) return *it;
  if (it == samples_.begin() || it == samples_.end())
    throw std::out_of_range("cannot interpolate boundary");
  const auto& b = *it;
  const auto& a = *std::prev(it);
  const double u = double(t-a.timestamp_ns)/double(b.timestamp_ns-a.timestamp_ns);
  return {t, lerp(a.angular_velocity_rad_s,b.angular_velocity_rad_s,u),
             lerp(a.linear_acceleration_m_s2,b.linear_acceleration_m_s2,u)};
}

std::vector<ImuSample> ImuBuffer::range(std::int64_t start, std::int64_t end) const {
  if (end < start) throw std::invalid_argument("invalid range");
  std::vector<ImuSample> out;
  for (const auto& s : samples_)
    if (s.timestamp_ns >= start && s.timestamp_ns <= end) out.push_back(s);
  return out;
}

PreintegratedDelta preintegrate(const std::vector<ImuSample>& s, const ImuBias& bias) {
  if (s.size() < 2) throw std::invalid_argument("need at least two IMU samples");
  for (std::size_t i=1;i<s.size();++i)
    if (s[i].timestamp_ns <= s[i-1].timestamp_ns)
      throw std::invalid_argument("IMU timestamps must increase");

  PreintegratedDelta out{};
  Vec3 velocity{};
  Vec3 position{};
  Quaternion q{};

  for (std::size_t i=1;i<s.size();++i) {
    const double dt = double(s[i].timestamp_ns-s[i-1].timestamp_ns)*1e-9;
    const Vec3 w0 = s[i-1].angular_velocity_rad_s - bias.gyro_rad_s;
    const Vec3 w1 = s[i].angular_velocity_rad_s - bias.gyro_rad_s;
    const Vec3 a0 = s[i-1].linear_acceleration_m_s2 - bias.accel_m_s2;
    const Vec3 a1 = s[i].linear_acceleration_m_s2 - bias.accel_m_s2;
    const Vec3 w = (w0+w1)*0.5;
    const Vec3 acc_body = (a0+a1)*0.5;

    const Quaternion dq = Quaternion::from_rotation_vector(w*dt);
    const Vec3 acc_local = q.rotate(acc_body);

    position += velocity*dt + acc_local*(0.5*dt*dt);
    velocity += acc_local*dt;
    q = (q*dq).normalized();
    out.dt_s += dt;
  }
  out.delta_rotation = q;
  out.delta_velocity = velocity;
  out.delta_position = position;
  return out;
}

Vec3 estimate_gravity_from_stationary(const std::vector<ImuSample>& samples, double g) {
  if (samples.empty()) throw std::invalid_argument("no samples");
  Vec3 avg{};
  for (const auto& s : samples) avg += s.linear_acceleration_m_s2;
  avg = avg / double(samples.size());
  const double n = avg.norm();
  if (n < 1e-6) throw std::invalid_argument("acceleration magnitude too small");
  // Assumption: accelerometer at rest measures +g opposite gravity.
  return avg * (-g/n);
}

} // namespace vionav
