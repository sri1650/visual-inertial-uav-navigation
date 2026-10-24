#include "vionav/keyframes.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vionav {

static double norm(const std::vector<float>& v) {
  double s=0; for (auto x:v) s+=double(x)*double(x);
  return std::sqrt(s);
}

void KeyframeDatabase::add(Keyframe frame) {
  if (frames_.contains(frame.frame_id))
    throw std::invalid_argument("duplicate keyframe id");
  if (frame.descriptor.empty()) throw std::invalid_argument("empty descriptor");
  const double n=norm(frame.descriptor);
  if (!(n>0) || !std::isfinite(n)) throw std::invalid_argument("invalid descriptor");
  for (auto& x:frame.descriptor) x=float(double(x)/n);
  frames_.emplace(frame.frame_id,std::move(frame));
}

std::vector<LoopCandidate> KeyframeDatabase::query(
    const Keyframe& query,std::size_t top_k,std::uint64_t exclude_recent,double min_similarity) const {
  if (top_k==0) throw std::invalid_argument("top_k must be positive");
  if (query.descriptor.empty()) return {};
  const double qn=norm(query.descriptor);
  if (!(qn>0)) return {};
  std::vector<LoopCandidate> out;
  for (const auto& [id,frame]:frames_) {
    const auto diff=(query.frame_id>id)?query.frame_id-id:id-query.frame_id;
    if (diff<=exclude_recent || frame.descriptor.size()!=query.descriptor.size()) continue;
    double dot=0;
    for (std::size_t i=0;i<frame.descriptor.size();++i)
      dot += (double(query.descriptor[i])/qn)*double(frame.descriptor[i]);
    if (dot>=min_similarity) out.push_back({query.frame_id,id,dot});
  }
  std::sort(out.begin(),out.end(),[](const auto& a,const auto& b){
    if (a.similarity!=b.similarity) return a.similarity>b.similarity;
    return a.match_id<b.match_id;
  });
  if (out.size()>top_k) out.resize(top_k);
  return out;
}

} // namespace vionav
