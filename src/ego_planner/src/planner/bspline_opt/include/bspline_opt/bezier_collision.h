#ifndef EGO_BEZIER_COLLISION_H
#define EGO_BEZIER_COLLISION_H
#include <Eigen/Core>
#include <array>
#include <vector>
#include <cmath>
#include <cstddef>
#include <chrono>
namespace ego_planner {
using CubicHull = std::array<Eigen::Vector3d, 4>;
// Closed voxel faces and the complete continuous curve are checked. A hull
// intersecting occupancy is subdivided; an unresolved hull fails closed.
template<class Occupied>
bool cubicHullFree(const CubicHull& initial, const Eigen::Vector3d& origin,
                   double resolution, Occupied occupied, std::size_t& nodes,
                   double& hit_fraction, double margin = 0.0) {
  if (!std::isfinite(resolution) || resolution<=0 || !origin.allFinite() ||
      !std::isfinite(margin) || margin<0) return false;
  const auto started=std::chrono::steady_clock::now();
  struct Node { CubicHull hull; int depth; double a,b; };
  std::vector<Node> stack{{initial,0,0.,1.}};
  while (!stack.empty()) {
    Node current=stack.back();stack.pop_back();
    hit_fraction=(current.a+current.b)/2.;
    if (++nodes>10000 || std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>.05) return false;
    Eigen::Vector3d lo=current.hull[0],hi=lo;
    for (const auto& p:current.hull) {
      if (!p.allFinite()) return false;
      lo=lo.cwiseMin(p);hi=hi.cwiseMax(p);
    }
    Eigen::Vector3d lower=(lo-origin-Eigen::Vector3d::Constant(margin))/resolution;
    Eigen::Vector3d upper=(hi-origin+Eigen::Vector3d::Constant(margin))/resolution;
    if ((lower.array().abs()>1000000).any() || (upper.array().abs()>1000000).any()) return false;
    Eigen::Vector3i first,last;
    long long count=1;
    for (int axis=0;axis<3;++axis) {
      first[axis]=static_cast<int>(std::floor(lower[axis]-1e-8));
      last[axis]=static_cast<int>(std::floor(upper[axis]+1e-8));
      count*=static_cast<long long>(last[axis]-first[axis]+1);
      if (count>100000) return false;
    }
    bool hit=false;
    for (int x=first.x();x<=last.x()&&!hit;++x)
      for (int y=first.y();y<=last.y()&&!hit;++y)
        for (int z=first.z();z<=last.z();++z)
          if (occupied(Eigen::Vector3i(x,y,z))) {hit=true;break;}
    if (!hit) continue;
    if (current.depth>=18 || (hi-lo).norm()<resolution/1024.) return false;
    Eigen::Vector3d a=(current.hull[0]+current.hull[1])/2.;
    Eigen::Vector3d b=(current.hull[1]+current.hull[2])/2.;
    Eigen::Vector3d c=(current.hull[2]+current.hull[3])/2.;
    Eigen::Vector3d d=(a+b)/2.,e=(b+c)/2.,middle=(d+e)/2.;
    double mid=(current.a+current.b)/2.;
    stack.push_back({{{middle,e,c,current.hull[3]}},current.depth+1,mid,current.b});
    stack.push_back({{{current.hull[0],a,d,middle}},current.depth+1,current.a,mid});
  }
  return true;
}
}
#endif
