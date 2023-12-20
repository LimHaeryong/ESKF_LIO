#ifndef ESKF_LIO_ODOMETRY_HPP_
#define ESKF_LIO_ODOMETRY_HPP_

#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include <yaml-cpp/yaml.h>

#include "ESKF_LIO/SynchronizedQueue.hpp"
#include "ESKF_LIO/Types.hpp"
#include "ESKF_LIO/ErrorStateKF.hpp"
#include "ESKF_LIO/LocalMap.hpp"
#include "ESKF_LIO/CloudPreprocessor.hpp"

namespace ESKF_LIO
{
class Odometry
{
public:
  using ImuBuffer = typename std::shared_ptr<SynchronizedQueue<ImuMeasurementPtr>>;
  using CloudBuffer = typename std::shared_ptr<SynchronizedQueue<LidarMeasurementPtr>>;
  Odometry(const YAML::Node & config, ImuBuffer imuBuffer, CloudBuffer cloudBuffer)
  : imuBuffer_(imuBuffer)
    , cloudBuffer_(cloudBuffer)
    , kalmanFilter_(std::make_shared<ErrorStateKF>(config))
    , localMap_(std::make_shared<LocalMap>(config))
    , cloudPreprocessor_(std::make_shared<CloudPreprocessor>(config))
  {
  }

  void run();

private:
  Odometry() = delete;
  bool initialized_ = false;
  ImuBuffer imuBuffer_;
  CloudBuffer cloudBuffer_;

  std::shared_ptr<ErrorStateKF> kalmanFilter_;
  std::shared_ptr<LocalMap> localMap_;
  std::shared_ptr<CloudPreprocessor> cloudPreprocessor_;
  LidarMeasurementPtr lidarMeas_ = nullptr;
};
}  // namespace ESKF_LIO

#endif  // ESKF_LIO_ODOMETRY_HPP_
