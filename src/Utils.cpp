#include "ESKF_LIO/Utils.hpp"

namespace ESKF_LIO::Utils
{
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & vec)
{
  Eigen::Matrix3d skew;
  skew << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;

  return skew;
}

void transformPoints(
  std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & transform,
  size_t start, size_t end)
{
  for (size_t i = start; i < end; ++i) {
    points[i] = transform * points[i];
  }
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

Eigen::Matrix3d computeJ(const Eigen::Vector3d & r)
{
  double angle = r.norm();
  Eigen::Vector3d axis = r.normalized();
  Eigen::Matrix3d J;
  if (angle < 1e-6) {
    J = Eigen::Matrix3d::Identity();
  } else {
    double factor1 = std::sin(angle) / angle;
    double factor2 = (1.0 - std::cos(angle)) / angle;
    J = factor1 * Eigen::Matrix3d::Identity() + (1.0 - factor1) * axis * axis.transpose() +
      factor2 * skewSymmetric(axis);
  }
  return J;
}

Eigen::Isometry3d se3ToSE3(const Eigen::Vector<double, 6> & se3)
{
  Eigen::Isometry3d SE3;
  Eigen::Matrix3d J = computeJ(se3.tail<3>());
  SE3.translation() = J * se3.head<3>();
  SE3.linear() = rotationVectorToMatrix(se3.tail<3>());
  return SE3;
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
