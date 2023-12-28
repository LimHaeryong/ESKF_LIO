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

void rotateNormals(std::vector<Eigen::Vector3d> & normals, const Eigen::Matrix3d & rotation)
{
  Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> normalMatrix(normals[0].data(), 3,
    normals.size());
  normalMatrix = rotation * normalMatrix;
}

Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d & R)
{
  Eigen::AngleAxisd angleAxis(R);
  return angleAxis.angle() * angleAxis.axis();
}

Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d & r)
{
  Eigen::AngleAxisd angleAxis(r.norm(), r.normalized());
  return angleAxis.toRotationMatrix();
}

Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d & r)
{
  Eigen::AngleAxisd angleAxis(r.norm(), r.normalized());
  return Eigen::Quaterniond(angleAxis);
}

Eigen::Isometry3d se3ToSE3(const Eigen::Vector<double, 6> & se3)
{
  Eigen::Isometry3d SE3;
  SE3.translation() = se3.head<3>();
  SE3.linear() = rotationVectorToMatrix(se3.tail<3>());
  return SE3;
}
Eigen::Vector<double, 6> SE3Tose3(const Eigen::Isometry3d & SE3)
{
  Eigen::Vector<double, 6> se3;
  se3.head<3>() = SE3.translation();
  se3.tail<3>() = rotationMatrixToVector(SE3.linear());
  return se3;
}

Eigen::Isometry3d interpolateSE3(
  const ESKF_LIO::State & s1, const ESKF_LIO::State & s2,
  const double t)
{
  double factor = (t - s1.timestamp) / (s2.timestamp - s1.timestamp + 1e-6);
  Eigen::Isometry3d SE3;
  SE3.linear() = s1.attitude.slerp(factor, s2.attitude).toRotationMatrix();
  SE3.translation() = s1.position + factor * (s2.position - s1.position);

  return SE3;
}

}  // namespace ESKF_LIO::Utils
