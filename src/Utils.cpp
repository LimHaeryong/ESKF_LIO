#include "ESKF_LIO/Utils.hpp"

namespace ESKF_LIO::Utils
{
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & vec)
{
  Eigen::Matrix3d skew;
  skew << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;

  return skew;
}

void transformPoints(std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & transform)
{
  Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> pointMatrix(points[0].data(), 3,
    points.size());
  pointMatrix = transform * pointMatrix;
}

void transformPoints(
  std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & transform,
  size_t start, size_t size)
{
  Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> pointMatrix(points[start].data(), 3, size);
  pointMatrix = transform * pointMatrix;
}

}  // namespace ESKF_LIO::Utils
