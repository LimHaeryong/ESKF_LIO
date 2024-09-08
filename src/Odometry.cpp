#include "ESKF_LIO/Odometry.hpp"

#include <algorithm>
#include <omp.h>

namespace ESKF_LIO
{

void Odometry::run()
{
  double cloudPreprocessorElapsedTime = 0.0, filterUpdateElapsedTime = 0.0,
    mapUpdateElapsedTime = 0.0;
  double cloudPreprocessorMaxElapsedTime = 0.0, filterUpdateMaxElapsedTime = 0.0,
    mapUpdateMaxElapsedTime = 0.0;
  int numFrames = 0;
  while (!exitFlag_) {
    if (visualize_) {
      if (localMap_->visualizeLocalMap() == false) {
        setExit();
      }
    }

    // get imu measurement
    auto imuMeas = imuBuffer_->popAll();

    // kalmanfilter process
    if (!imuMeas.empty()) {
      if (initialized_) {
        while (!imuMeas.empty()) {
          kalmanFilter_->process(imuMeas.front());
          kalmanFilter_->feedImu(imuMeas.front());
          imuMeas.pop_front();
        }
      } else {
        while (!imuMeas.empty()) {
          kalmanFilter_->feedImu(imuMeas.front());
          imuMeas.pop_front();
        }
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
      kalmanFilter_->initialize(lidarMeas_->endTime);
      auto lidarMeasCopy = lidarMeas_;
      lidarMeas_ = nullptr;
      cloudPreprocessor_->process({}, lidarMeasCopy);
      localMap_->updateLocalMap(std::move(lidarMeasCopy->cloud), Eigen::Isometry3d::Identity());
      continue;
    }

    auto lastUpdatedTime = kalmanFilter_->getLastStateTime();
    // wait for next imu
    if (lastUpdatedTime < lidarEndTime) {
      continue;
    }

    const auto & states = kalmanFilter_->getStates();

    auto cloudPreprocessorStart = omp_get_wtime();
    cloudPreprocessor_->process(states, lidarMeas_);
    auto cloudPreprocessorEnd = omp_get_wtime();

    auto filterUpdateStart = omp_get_wtime();
    // kalman filter update
    auto transform = kalmanFilter_->update(*lidarMeas_, *localMap_);
    auto filterUpdateEnd = omp_get_wtime();

    auto mapUpdateStart = omp_get_wtime();
    // local map update
    auto lidarMeasCopy = lidarMeas_;
    lidarMeas_ = nullptr;
    localMap_->updateLocalMap(std::move(lidarMeasCopy->cloud), transform);
    auto mapUpdateEnd = omp_get_wtime();

    ++numFrames;
    cloudPreprocessorElapsedTime += cloudPreprocessorEnd - cloudPreprocessorStart;
    filterUpdateElapsedTime += filterUpdateEnd - filterUpdateStart;
    mapUpdateElapsedTime += mapUpdateEnd - mapUpdateStart;
    cloudPreprocessorMaxElapsedTime = std::max(
      cloudPreprocessorMaxElapsedTime, cloudPreprocessorEnd - cloudPreprocessorStart);
    filterUpdateMaxElapsedTime = std::max(filterUpdateMaxElapsedTime, filterUpdateEnd - filterUpdateStart);
    mapUpdateMaxElapsedTime = std::max(mapUpdateMaxElapsedTime, mapUpdateEnd - mapUpdateStart);
  }

  std::cout << "cloud preprocessor average elapsed time = " << std::fixed <<
    cloudPreprocessorElapsedTime / numFrames * 1000 << " ms\n";
  std::cout << "cloud preprocessor max elapsed time = " << std::fixed <<
    cloudPreprocessorMaxElapsedTime * 1000 << " ms\n";
  std::cout << "filter update average elapsed time = " << std::fixed <<
    filterUpdateElapsedTime / numFrames * 1000 << " ms\n";
  std::cout << "filter update max elapsed time = " << std::fixed <<
    filterUpdateMaxElapsedTime * 1000 << " ms\n";
  std::cout << "map update average elapsed time = " << std::fixed <<
    mapUpdateElapsedTime / numFrames * 1000 << " ms\n";
  std::cout << "map update max elapsed time = " << std::fixed << mapUpdateMaxElapsedTime * 1000 << " ms\n";
}
}  // namespace ESKF_LIO
