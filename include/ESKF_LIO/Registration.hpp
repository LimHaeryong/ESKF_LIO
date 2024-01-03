#ifndef ESKF_LIO_REGISTRATION_HPP_
#define ESKF_LIO_REGISTRATION_HPP_

#include <cmath>
#include <numeric>
#include <tuple>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "ESKF_LIO/LocalMap.hpp"

namespace ESKF_LIO
{
class ICP
{
public:
  using PointVector = typename std::vector<Eigen::Vector3d>;
  using CovarianceVector = typename std::vector<Eigen::Matrix3d>;
  using Correspondence = typename std::tuple<PointVector, CovarianceVector, PointVector,
      CovarianceVector>;

  ICP(const YAML::Node & config)
  : maxIteration_(config["registration"]["max_iteration"].as<int>())
    , translationSquaredThreshold_(config["registration"]["translation_sq_threshold"].as<double>())
    , cosineThreshold_(config["registration"]["cosine_threshold"].as<double>())
  {
  }

  Eigen::Isometry3d align(
    const PointCloud & cloud, const LocalMap & localMap,
    const Eigen::Isometry3d & guess);

private:
  ICP() = delete;

  Eigen::Isometry3d computeTransform(Correspondence & correspondence) const;
  bool convergenceCheck(const Eigen::Isometry3d & transformIter) const;

  std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Vector<double, 6>>
  computeJTJAndJTr(
    const Eigen::Vector3d & srcPoint,
    const Eigen::Vector3d & mapPoint,
    const Eigen::Matrix3d & covariance) const;

  int maxIteration_;
  double translationSquaredThreshold_;
  double cosineThreshold_;

  bool converged_ = false;
};

}  // namespace ESKF_LIO

#endif  // ESKF_LIO_REGISTRATION_HPP_
