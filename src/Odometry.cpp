#include "ESKF_LIO/Odometry.hpp"

namespace ESKF_LIO
{
Odometry::Odometry(const YAML::Node & config, ImuBuffer imuBuffer, CloudBuffer cloudBuffer)
: imuBuffer_(imuBuffer), cloudBuffer_(cloudBuffer),
  kalmanFilter_(std::make_shared<ErrorStateKF>(config)),
  localMap_(std::make_shared<LocalMap>(config))
{
  auto imu = config["sensors"]["imu"];
  auto lidar = config["sensors"]["lidar"];

  auto lidar_quat = lidar["extrinsics"]["quaternion"].as<std::vector<double>>();
  auto lidar_trans = lidar["extrinsics"]["translation"].as<std::vector<double>>();

  imuToLidar_.quat = Eigen::Map<Eigen::Quaterniond>(lidar_quat.data());
  imuToLidar_.trans = Eigen::Map<Eigen::Vector3d>(lidar_trans.data());
}

void Odometry::run()
{
  while (true) {
    // get imu measurement
    auto imuMeas = imuBuffer_->popAll();
    if (!imuMeas.empty()) {
      for (auto & imu : imuMeas) {
        imuMeas_.push_back(imu);
      }
    }

    // kalmanfilter process
    if (!imuMeas.empty() && initialized_) {
      for (auto & imu : imuMeas) {
        kalmanFilter_->process(std::move(imu));
      }
    }

    // get lidar measurement
    if (lidarMeas_ == nullptr) {
      auto lidarMeas = cloudBuffer_->pop();
      if (lidarMeas.has_value()) {
        lidarMeas_ = lidarMeas.value();
      }
    }

    if (lidarMeas_ == nullptr) {
      continue;
    }

    if (!initialized_) {
      initialized_ = true;

      lastUpdatedTime_ = lidarMeas_->endTime;
      kalmanFilter_->initState(lastUpdatedTime_);
      while (imuMeas_[0]->timestamp < lastUpdatedTime_) {
        imuMeas_.pop_front();
      }
      continue;
    }


  }
}
}  // namespace ESKF_LIO
