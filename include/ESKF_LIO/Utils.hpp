#ifndef ESKF_LIO_UTILS_HPP_
#define ESKF_LIO_UTILS_HPP_

#include <Eigen/Dense>

#include "ESKF_LIO/Types.hpp"

namespace ESKF_LIO::Utils
{
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & vec);

void transformPoints(
  std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & transform, size_t start,
  size_t end);

Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d & R);
Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d & r);
Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d & r);
Eigen::Isometry3d se3ToSE3(const Eigen::Vector<double, 6> & se3);

Eigen::Isometry3d interpolateSE3(
  const ESKF_LIO::State & s1, const ESKF_LIO::State & s2,
  const double t);


}  // namespace ESKF_LIO::Utils

#endif  // ESKF_LIO_UTILS_HPP_
