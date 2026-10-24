#include "vionav/timestamp_sync.hpp"
#include <cassert>
#include <iostream>

int main() {
  auto x = vionav::pair_nearest({100}, {95,105}, 10);
  assert(x.size() == 1);
  assert(x[0].candidate_ns.has_value());
  assert(*x[0].candidate_ns == 95);
  assert(*x[0].delta_ns == -5);

  auto y = vionav::pair_nearest({100}, {130}, 20);
  assert(!y[0].candidate_ns.has_value());

  std::cout << "timestamp_sync tests passed\n";
  return 0;
}
