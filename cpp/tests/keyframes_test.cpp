#include "vionav/keyframes.hpp"
#include <cassert>

int main() {
  using namespace vionav;
  KeyframeDatabase db;
  db.add({0,0,{0,0,0},{1,0,0}});
  db.add({20,20,{1,0,0},{.98f,.02f,0}});
  db.add({30,30,{2,0,0},{0,1,0}});
  auto c=db.query({40,40,{3,0,0},{1,0,0}},5,5,.8);
  assert(c.size()==2);
  assert(c[0].match_id==0);
}
