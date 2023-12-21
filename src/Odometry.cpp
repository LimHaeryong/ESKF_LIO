#include "ESKF_LIO/Odometry.hpp"

namespace ESKF_LIO
{

void Odometry::run()
{
  while (true) {
    // get imu measurement
    auto imuMeas = imuBuffer_->popAll();

    // kalmanfilter process
    if (!imuMeas.empty() && initialized_) {
      while (!imuMeas.empty()) {
        kalmanFilter_->process(imuMeas.front());
        kalmanFilter_->feedImu(imuMeas.front());
        imuMeas.pop_front();
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

    auto lidarEndTime = lidarMeas_->endTime;
    if (!initialized_) {
      initialized_ = true;
      kalmanFilter_->initState(lidarMeas_->endTime);
      while (!imuMeas.empty()) {
        kalmanFilter_->process(imuMeas.front());
        kalmanFilter_->feedImu(imuMeas.front());
        imuMeas.pop_front();
      }
      continue;
    }

    auto lastUpdatedTime = kalmanFilter_->getLastStateTime();
    // wait for next imu
    if (lastUpdatedTime < lidarEndTime) {
      continue;
    }

    const auto & states = kalmanFilter_->getStates();
    cloudPreprocessor_->process(states, lidarMeas_);

    kalmanFilter_->update(lidarMeas_);
    auto lidarMeas = lidarMeas_;
    lidarMeas_ = nullptr;
  }
}
}  // namespace ESKF_LIO
