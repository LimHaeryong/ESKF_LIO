#include "ESKF_LIO/Odometry.hpp"

namespace ESKF_LIO
{

void Odometry::run()
{
  while (!exitFlag_)
  {
    if (visualize_)
    {
      if (localMap_->visualizeLocalMap() == false)
        setExit();
    }
    
    // get imu measurement
    auto imuMeas = imuBuffer_->popAll();

    // kalmanfilter process
    if (!imuMeas.empty())
    {
      if (initialized_)
      {
        while (!imuMeas.empty())
        {
          kalmanFilter_->process(imuMeas.front());
          kalmanFilter_->feedImu(imuMeas.front());
          imuMeas.pop_front();
        }
      }
      else
      {
        while (!imuMeas.empty())
        {
          kalmanFilter_->feedImu(imuMeas.front());
          imuMeas.pop_front();
        }
      }
    }

    // get lidar measurement
    if (lidarMeas_ == nullptr)
    {
      auto lidarMeas = cloudBuffer_->pop();
      if (lidarMeas.has_value())
      {
        lidarMeas_ = lidarMeas.value();
      }
    }

    if (lidarMeas_ == nullptr)
    {
      continue;
    }

    auto lidarEndTime = lidarMeas_->endTime;
    if (!initialized_)
    {
      initialized_ = true;
      kalmanFilter_->initialize(lidarMeas_->endTime);
      auto lidarMeasCopy = lidarMeas_;
      lidarMeas_ = nullptr;
      cloudPreprocessor_->voxelDownsample(lidarMeasCopy->cloud->points_);
      localMap_->updateLocalMap(std::move(lidarMeasCopy->cloud), Eigen::Isometry3d::Identity());
      continue;
    }

    auto lastUpdatedTime = kalmanFilter_->getLastStateTime();
    // wait for next imu
    if (lastUpdatedTime < lidarEndTime)
    {
      continue;
    }

    const auto& states = kalmanFilter_->getStates();
    cloudPreprocessor_->process(states, lidarMeas_);

    // kalman filter update
    auto transform = kalmanFilter_->update(*lidarMeas_, *localMap_);

    // local map update
    auto lidarMeasCopy = lidarMeas_;
    lidarMeas_ = nullptr;
    localMap_->updateLocalMap(std::move(lidarMeasCopy->cloud), transform);
  }
}
}  // namespace ESKF_LIO
