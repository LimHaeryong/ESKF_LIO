#ifndef ESKF_LIO_ODOMETRY_HPP_
#define ESKF_LIO_ODOMETRY_HPP_

#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include <yaml-cpp/yaml.h>

#include "ESKF_LIO/SynchronizedQueue.hpp"
#include "ESKF_LIO/Types.hpp"
#include "ESKF_LIO/ErrorStateKF.hpp"
#include "ESKF_LIO/LocalMap.hpp"

namespace ESKF_LIO
{
class Odometry
{
public:
  using ImuBuffer = typename std::shared_ptr<SynchronizedQueue<ImuMeasurementPtr>>;
  using CloudBuffer = typename std::shared_ptr<SynchronizedQueue<LidarMeasurementPtr>>;
  Odometry(const YAML::Node & config, ImuBuffer imuBuffer, CloudBuffer cloudBuffer);

  void run();

private:
  Odometry() = delete;
  bool initialized_ = false;
  ImuBuffer imuBuffer_;
  CloudBuffer cloudBuffer_;

  std::shared_ptr<ErrorStateKF> kalmanFilter_;
  std::shared_ptr<LocalMap> localMap_;
  SE3 imuToLidar_;
  LidarMeasurementPtr lidarMeas_ = nullptr;
  std::deque<ImuMeasurementPtr> imuMeas_;

  double lastUpdatedTime_;
};
}  // namespace ESKF_LIO

#endif // ESKF_LIO_ODOMETRY_HPP_
