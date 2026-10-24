#include "vionav/timestamp_sync.hpp"
#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace vionav {

std::vector<TimePair> pair_nearest(
    const std::vector<std::int64_t>& refs,
    const std::vector<std::int64_t>& candidates,
    std::int64_t max_delta_ns) {
  if (max_delta_ns < 0) throw std::invalid_argument("max_delta_ns must be non-negative");
  auto sorted = candidates;
  std::sort(sorted.begin(), sorted.end());
  std::vector<TimePair> out;
  out.reserve(refs.size());

  for (auto r : refs) {
    if (sorted.empty()) {
      out.push_back({r, std::nullopt, std::nullopt});
      continue;
    }
    auto it = std::lower_bound(sorted.begin(), sorted.end(), r);
    std::vector<std::int64_t> choices;
    if (it != sorted.end()) choices.push_back(*it);
    if (it != sorted.begin()) choices.push_back(*std::prev(it));
    auto best = *std::min_element(choices.begin(), choices.end(),
      [r](auto a, auto b) {
        auto da = std::llabs(a-r), db = std::llabs(b-r);
        return da == db ? a < b : da < db;
      });
    auto delta = best-r;
    if (std::llabs(delta) <= max_delta_ns)
      out.push_back({r, best, delta});
    else
      out.push_back({r, std::nullopt, std::nullopt});
  }
  return out;
}

}  // namespace vionav
