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

}  // namespace ESKF_LIO

#endif // ESKF_LIO_TYPES_HPP_
