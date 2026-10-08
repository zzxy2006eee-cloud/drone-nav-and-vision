#ifndef EGO_TRAJECTORY_COLLISION_H
#define EGO_TRAJECTORY_COLLISION_H
#include <bspline_opt/bezier_collision.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <chrono>
namespace ego_planner {
inline bool continuousTrajectoryFree(UniformBspline trajectory, const GridMap::Ptr& map,
                                     double& hit_time, double margin=0.0) {
  const auto started=std::chrono::steady_clock::now();
  Eigen::MatrixXd points=trajectory.getControlPoint();
  Eigen::VectorXd knots=trajectory.getKnot();
  if (points.rows()!=3 || points.cols()<4 || points.cols()>256 ||
      knots.size()!=points.cols()+4 || !points.allFinite() || !knots.allFinite()) return false;
  for (int i=1;i<knots.size();++i) if (knots[i]<knots[i-1]) return false;
  double start,end;
  if (!trajectory.getTimeSpan(start,end) || end<=start || end-start>120.) return false;
  UniformBspline velocity=trajectory.getDerivative();
  const Eigen::Vector3d origin=map->getOrigin();
  const double resolution=map->getResolution();
  std::size_t nodes=0;
  auto occupied=[&](const Eigen::Vector3i& id) {
    Eigen::Vector3d center;map->indexToPos(id,center);
    return map->getInflateOccupancy(center)!=0;
  };
  for (int i=3;i<points.cols();++i) {
    const double left=knots[i],right=knots[i+1],duration=right-left;
    if (duration<=0) continue;
    Eigen::Vector3d p0=trajectory.evaluateDeBoor(left),p3=trajectory.evaluateDeBoor(right);
    CubicHull controls{{p0,p0+velocity.evaluateDeBoor(left)*duration/3.,
                         p3-velocity.evaluateDeBoor(right)*duration/3.,p3}};
    double fraction=0.;
    if (!cubicHullFree(controls,origin,resolution,occupied,nodes,fraction,margin)) {
      hit_time=left-start+duration*fraction;return false;
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>.05) {
      hit_time=left-start;return false;
    }
  }
  return true;
}
}
#endif
