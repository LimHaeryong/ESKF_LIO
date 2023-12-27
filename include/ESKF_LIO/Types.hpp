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

struct State
{
  double timestamp;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d biasAccel = Eigen::Vector3d::Zero();
  Eigen::Vector3d biasGyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 18, 18> P = 1e-3 * Eigen::Matrix<double, 18, 18>::Identity();

  void printState() const
  {
    std::cout << "State at " << timestamp << "\n";
    std::cout << "pose = " << position.transpose() << "\n";
    std::cout << "vel = " << velocity.transpose() << "\n";
    std::cout << "attitude = " << attitude.coeffs().transpose() << "\n";
    std::cout << "bias_a = " << biasAccel.transpose() << "\n";
    std::cout << "bias_g = " << biasGyro.transpose() << "\n";
    std::cout << "gravity = " << gravity.transpose() << "\n";
  }
};

}  // namespace ESKF_LIO

#endif  // ESKF_LIO_TYPES_HPP_
