#pragma once
#include "vionav/types.hpp"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace vionav {

struct Keyframe {
  std::uint64_t frame_id{};
  std::int64_t timestamp_ns{};
  Vec3 position_m{};
  std::vector<float> descriptor;
};

struct LoopCandidate {
  std::uint64_t query_id{};
  std::uint64_t match_id{};
  double similarity{};
};

class KeyframeDatabase {
 public:
  void add(Keyframe frame);
  std::size_t size() const { return frames_.size(); }
  std::vector<LoopCandidate> query(const Keyframe& query,
                                   std::size_t top_k=5,
                                   std::uint64_t exclude_recent=10,
                                   double min_similarity=.7) const;
 private:
  std::unordered_map<std::uint64_t,Keyframe> frames_;
};

} // namespace vionav
