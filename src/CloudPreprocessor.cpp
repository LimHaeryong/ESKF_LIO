#include <unordered_map>

#include "ESKF_LIO/CloudPreprocessor.hpp"
#include "ESKF_LIO/Utils.hpp"

namespace ESKF_LIO
{
void CloudPreprocessor::process(
  const std::deque<ErrorStateKF::State> & states,
  LidarMeasurementPtr lidarMeas) const
{
  auto & cloud_points = lidarMeas->cloud->points_;
  // transform lidar points to imu coordinate
  Utils::transformPoints(cloud_points, T_il_);

  deskew(states, lidarMeas->pointTime, cloud_points);
  lidarMeas->pointTime.clear();
  lidarMeas->pointTime.shrink_to_fit();

  voxelDownsample(cloud_points);
}

void CloudPreprocessor::deskew(
  const std::deque<ErrorStateKF::State> & states, const std::vector<double> & pointTime,
  std::vector<Eigen::Vector3d> & points) const
{
  size_t numPoints = points.size();
  size_t pointStartIndex = 0;
  size_t pointEndIndex = 0;

  double lidarEndTime = pointTime.back();

  auto lastState = states.crbegin();
  for (; lastState != states.crend(); ++lastState) {
    if (lastState->timestamp > lidarEndTime) {
      continue;
    }

    break;
  }

  Eigen::Isometry3d lastPoseInv;
  lastPoseInv.linear() = lastState->attitude.toRotationMatrix();
  lastPoseInv.translation() = lastState->position;
  lastPoseInv = lastPoseInv.inverse();

  for (auto state = states.cbegin(); state != lastState.base(); ++state) {
    pointStartIndex = pointEndIndex;
    size_t i = pointStartIndex;
    while (i < numPoints) {
      if (pointTime[i] < state->timestamp) {
        ++i;
      } else {
        pointEndIndex = i;
        break;
      }
    }

    if (pointStartIndex == pointEndIndex) {
      continue;
    }

    Eigen::Isometry3d transform;
    transform.linear() = state->attitude.toRotationMatrix();
    transform.translation() = state->position;
    transform = lastPoseInv * transform;

    Utils::transformPoints(points, transform, pointStartIndex, pointEndIndex - pointStartIndex);
  }
}

void CloudPreprocessor::voxelDownsample(std::vector<Eigen::Vector3d> & points) const
{
  std::vector<Eigen::Vector3d> output;
  std::unordered_map<Eigen::Vector3i, int, open3d::utility::hash_eigen<Eigen::Vector3i>> voxelGrid;

  for (size_t i = 0; i < points.size(); ++i) {
    auto voxelIndex = getVoxelIndex(points[i]);
    if (voxelGrid.find(voxelIndex) == voxelGrid.end()) {
      voxelGrid[voxelIndex] = i;
    }
  }

  output.reserve(voxelGrid.size());
  for (const auto & [_, i] : voxelGrid) {
    output.push_back(points[i]);
  }
  std::swap(points, output);
}

Eigen::Vector3i CloudPreprocessor::getVoxelIndex(const Eigen::Vector3d & point) const
{
  Eigen::Vector3i voxelIndex = (point / voxelSize_).array().floor().cast<int>();
  return voxelIndex;
}

}   // namespace ESKF_LIO
