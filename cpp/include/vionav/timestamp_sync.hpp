#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace vionav {

struct TimePair {
  std::int64_t reference_ns{};
  std::optional<std::int64_t> candidate_ns;
  std::optional<std::int64_t> delta_ns;
};

std::vector<TimePair> pair_nearest(
    const std::vector<std::int64_t>& reference_ns,
    const std::vector<std::int64_t>& candidate_ns,
    std::int64_t max_delta_ns);

}  // namespace vionav
