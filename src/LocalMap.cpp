#include <numeric>
#include <omp.h>

#include "ESKF_LIO/LocalMap.hpp"
#include "ESKF_LIO/Utils.hpp"

namespace ESKF_LIO
{

void LocalMap::updateLocalMap(PointCloudPtr cloud, const Eigen::Isometry3d & transform, bool initialize)
{

  cloud->Transform(transform.matrix());
  open3d::camera::PinholeCameraParameters parameter;
  parameter.extrinsic_ = transform.matrix();
  trajectory_.parameters_.push_back(parameter);

  if (visualize_) {
    auto coord =
      open3d::geometry::TriangleMesh::CreateCoordinateFrame(3.0, transform.translation());
    coord->Rotate(transform.linear(), transform.translation());
    visualizer_->ClearGeometries();
    visualizer_->AddGeometry(cloud);
    visualizer_->AddGeometry(coord);
    visualizer_->GetViewControl().ConvertFromPinholeCameraParameters(visualizerConfig_);
  }

  if(initialize == false && needsMapUpdate(transform) == false)
  {
    prevTransform_ = transform;
    return;
  }

  auto & points = cloud->points_;
  auto & covariances = cloud->covariances_;
  
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    const auto & covariance = covariances[i];
    auto voxelIndex = getVoxelIndex(point);
    auto found = voxelGrid_.find(voxelIndex);

    if (found == voxelGrid_.cend()) {
      voxelGrid_.emplace(voxelIndex, Voxel(maxNumPointsPerVoxel_, point, covariance));
    } else {
      found->second.addPoint(point, covariance);
    }
  }

  prevTransform_ = transform;
  return;
}

LocalMap::Correspondence LocalMap::correspondenceMatching(
  const PointVector & points, const CovarianceVector & covariances) const
{
  Correspondence correspondence;
  auto & [srcPoints, srcCovs, mapPoints, mapCovs] = correspondence;
  srcPoints.reserve(points.size());
  srcCovs.reserve(points.size());
  mapPoints.reserve(points.size());
  mapCovs.reserve(points.size());

#pragma omp parallel
  {
    PointVector srcPointsPrivate, mapPointsPrivate;
    CovarianceVector srcCovsPrivate, mapCovsPrivate;
#pragma omp for nowait
    for (size_t i = 0; i < points.size(); ++i) {
      VoxelGrid::const_iterator found = voxelGrid_.find(getVoxelIndex(points[i]));
      if(found != voxelGrid_.cend())
      {
        srcPointsPrivate.push_back(points[i]);
        srcCovsPrivate.push_back(covariances[i]);
        mapPointsPrivate.push_back(found->second.mean);
        mapCovsPrivate.push_back(found->second.covariance);
      }
    }
#pragma omp critical
    {
      srcPoints.insert(srcPoints.end(), srcPointsPrivate.begin(), srcPointsPrivate.end());
      srcCovs.insert(srcCovs.end(), srcCovsPrivate.begin(), srcCovsPrivate.end());
      mapPoints.insert(mapPoints.end(), mapPointsPrivate.begin(), mapPointsPrivate.end());
      mapCovs.insert(mapCovs.end(), mapCovsPrivate.begin(), mapCovsPrivate.end());
    }
  }

  return correspondence;
}

Eigen::Vector3i LocalMap::getVoxelIndex(const Eigen::Vector3d & point) const
{
  Eigen::Vector3i voxelIndex = (point / voxelSize_).array().floor().cast<int>();
  return voxelIndex;
}

bool LocalMap::visualizeLocalMap() const
{
  if (!visualizer_->PollEvents()) {
    return false;  // exit sign
  }

  if (visualizer_->HasGeometry()) {
    visualizer_->UpdateRender();
  }
  return true;
}

bool LocalMap::needsMapUpdate(const Eigen::Isometry3d & transform) const
{
  Eigen::Isometry3d moved = prevTransform_.inverse() * transform;

  double cosine = 0.5 * (moved.linear().trace() - 1.0);
  if (cosine < cosineThreshold_) {
    return true;
  }

  double translationSq = moved.translation().squaredNorm();
  if (translationSq > translationSquaredThreshold_) {
    return true;
  }

  return false;
}

void LocalMap::save(const std::string & cloud_path, const std::string & trajectory_path) const
{
  auto cloud = std::make_shared<PointCloud>();
  cloud->points_.reserve(voxelGrid_.size() * maxNumPointsPerVoxel_);

  for (const auto & [_, voxel] : voxelGrid_) {
    cloud->points_.insert(cloud->points_.end(), voxel.points.begin(), voxel.points.end());
  }

  open3d::io::WritePointCloud(cloud_path, *cloud);
  open3d::io::WritePinholeCameraTrajectory(trajectory_path, trajectory_);
}

}  // namespace ESKF_LIO
