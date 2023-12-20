#ifndef ESKF_LIO_UTILS_HPP_
#define ESKF_LIO_UTILS_HPP_

#include <Eigen/Dense>

namespace ESKF_LIO::Utils
{
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & vec);


void transformPoints(std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & transform);


void transformPoints(
  std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & transform,
  size_t start, size_t size);


}  // namespace ESKF_LIO::Utils

#endif  // ESKF_LIO_UTILS_HPP_
