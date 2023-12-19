#ifndef ESKF_LIO_TYPES_HPP_
#define ESKF_LIO_TYPES_HPP_

#include <memory>

#include <Eigen/Dense>
#include <open3d/Open3D.h>

namespace ESKF_LIO
{
using PointCloud = typename open3d::geometry::PointCloud;
using PointCloudPtr = typename std::shared_ptr<PointCloud>;

struct ImuMeasurement
{
  double timestamp;
  Eigen::Vector3d angularVelocity;
  Eigen::Vector3d acceleration;
};
using ImuMeasurementPtr = typename std::shared_ptr<ImuMeasurement>;

struct LidarMeasurement
{
  PointCloudPtr cloud;
  std::vector<double> pointTime;
  double startTime;
  double endTime;
};
using LidarMeasurementPtr = typename std::shared_ptr<LidarMeasurement>;

struct SE3
{
  Eigen::Quaterniond quat;
  Eigen::Vector3d trans;

  SE3()
  : quat(Eigen::Quaterniond::Identity()), trans(Eigen::Vector3d::Zero()) {}

  SE3(const Eigen::Matrix4d & mat)
  : quat(mat.block<3, 3>(0, 0)), trans(mat.block<3, 1>(0, 3)) {}

  SE3(const Eigen::Quaterniond & quat, const Eigen::Vector3d & trans)
  : quat(quat), trans(trans) {}

  SE3(Eigen::Quaterniond && quat, Eigen::Vector3d && trans)
  : quat(std::move(quat)), trans(std::move(trans)) {}

  Eigen::Vector3d operator*(const Eigen::Vector3d & point) const
  {
    return quat * point + trans;
  }

  SE3 operator*(const SE3 & other)
  {
    return SE3(quat * other.quat, trans + quat * other.trans);
  }

  void operator*=(const SE3 & other)
  {
    trans += quat * other.trans;
    quat *= other.quat;
  }

  SE3 inverse() const
  {
    return SE3{quat.conjugate(), -1.0 * (quat.conjugate() * trans)};
  }

  Eigen::Matrix4d getMatrix() const
  {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = quat.toRotationMatrix();
    mat.block<3, 1>(0, 3) = trans;

    return mat;
  }
};

}  // namespace ESKF_LIO

#endif // ESKF_LIO_TYPES_HPP_
