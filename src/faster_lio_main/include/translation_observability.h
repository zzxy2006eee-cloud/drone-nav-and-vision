#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace faster_lio {
struct TranslationObservability {
  Eigen::Vector3d strengths = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_inflation = Eigen::Matrix3d::Zero();
};

inline TranslationObservability analyzeTranslation(const Eigen::Matrix3d& information,
                                                   double minimum_support) {
  TranslationObservability result;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(information);
  if (!information.allFinite() || solver.info() != Eigen::Success) {
    result.covariance_inflation = Eigen::Matrix3d::Identity()*1e6;
    return result;
  }
  result.strengths = solver.eigenvalues();
  for (int i=0;i<3;++i) {
    if (minimum_support > 0 && result.strengths(i) < minimum_support) {
      const Eigen::Vector3d direction=solver.eigenvectors().col(i);
      result.covariance_inflation.noalias() += 1e6*direction*direction.transpose();
    }
  }
  return result;
}
}
