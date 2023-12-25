#ifndef ESKF_LIO_REGISTRATION_HPP_
#define ESKF_LIO_REGISTRATION_HPP_

#include <cmath>
#include <numeric>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "ESKF_LIO/LocalMap.hpp"

namespace ESKF_LIO
{
class ICP
{
public:
  using PointVector = typename std::vector<Eigen::Vector3d>;
  using Correspondence = typename std::pair<PointVector, PointVector>;

  ICP(const YAML::Node & config)
  : maxCorrespondenceDistSquared_(
      config["registration"]["max_correspondence_distance_sq"].as<double>())
    , maxIteration_(config["registration"]["max_iteration"].as<int>())
    ,
    relativeMatchingRmseThreshold_(
      config["registration"]["relative_matching_rmse_threshold"].as<double>())
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

  double maxCorrespondenceDistSquared_;
  int maxIteration_;
  double relativeMatchingRmseThreshold_;
  double translationSquaredThreshold_;
  double cosineThreshold_;

  bool converged_ = false;
  double matchingRmse_ = std::numeric_limits<double>::max();
  double matchingRmsePrev_ = std::numeric_limits<double>::max();
  std::pair<PointVector, PointVector> correspondence_;
};

}  // namespace ESKF_LIO

#endif  // ESKF_LIO_REGISTRATION_HPP_
