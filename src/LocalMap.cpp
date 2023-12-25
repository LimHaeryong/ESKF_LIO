#include <numeric>
#include <omp.h>

#include "ESKF_LIO/LocalMap.hpp"
#include "ESKF_LIO/Utils.hpp"

namespace ESKF_LIO
{
std::optional<std::pair<Eigen::Vector3d, double>> LocalMap::Voxel::nearestSearchInVoxel(const Eigen::Vector3d& point,
                                                                                        const double maxDistSq) const
{
  Eigen::Vector3d nearestPoint = Eigen::Vector3d::Zero();
  double nearestDistSq = maxDistSq;

  for (const auto& voxelPoint : points)
  {
    double distSq = (point - voxelPoint).squaredNorm();
    if (distSq < nearestDistSq)
    {
      nearestDistSq = distSq;
      nearestPoint = voxelPoint;
    }
  }

  if (nearestDistSq < maxDistSq)
  {
    return std::make_pair(nearestPoint, nearestDistSq);
  }
  else
  {
    return std::nullopt;
  }
}

void LocalMap::updateLocalMap(PointCloudPtr cloud, const Eigen::Isometry3d& transform)
{
  auto& points = cloud->points_;
  Utils::transformPoints(points, transform);
  open3d::camera::PinholeCameraParameters parameter;
  parameter.extrinsic_ = transform.matrix();
  trajectory_.parameters_.push_back(parameter);

  if (visualize_)
  {
    auto coord = open3d::geometry::TriangleMesh::CreateCoordinateFrame(3.0, transform.translation());
    coord->Rotate(transform.linear(), transform.translation());

    std::unique_lock<std::mutex> visualizerLock(visualizerMutex_);
    visualizer_->ClearGeometries();
    visualizer_->AddGeometry(cloud);
    visualizer_->AddGeometry(coord);
    visualizer_->GetViewControl().ConvertFromPinholeCameraParameters(visualizerConfig_);
  }

  std::unique_lock<std::shared_mutex> voxelGridLock(voxelGridMutex_);
  for (const auto& point : points)
  {
    auto voxelIndex = getVoxelIndex(point);
    auto found = voxelGrid_.find(voxelIndex);

    if (found == voxelGrid_.cend())
    {
      voxelGrid_.emplace(voxelIndex, Voxel(maxNumPointsPerVoxel_, point));
    }
    else
    {
      found->second.addPoint(point);
    }
  }
}

LocalMap::Correspondence LocalMap::correspondenceMatching(const PointVector& points, const double maxDistSq,
                                                          double& matchingRmse) const
{
  std::shared_lock<std::shared_mutex> voxelGridLock(voxelGridMutex_);
  Correspondence correspondence;
  auto& [P, Q] = correspondence;
  P.reserve(points.size());
  Q.reserve(points.size());

  matchingRmse = 0.0;

#pragma omp parallel
  {
    double distanceSquaredSumPrivate = 0.0;
    PointVector P_private, Q_private;
#pragma omp for nowait
    for (size_t i = 0; i < points.size(); ++i)
    {
      auto searchResult = nearestSearch(points[i], maxDistSq);
      if (searchResult.has_value())
      {
        P_private.push_back(points[i]);
        Q_private.push_back(searchResult.value().first);
        distanceSquaredSumPrivate += searchResult.value().second;
      }
    }
#pragma omp critical
    {
      matchingRmse += distanceSquaredSumPrivate;
      for (size_t i = 0; i < P_private.size(); ++i)
      {
        P.push_back(P_private[i]);
        Q.push_back(Q_private[i]);
      }
    }
  }

  if (P.size() > 0)
  {
    matchingRmse = std::sqrt(matchingRmse / static_cast<double>(P.size()));
  }

  return correspondence;
}

std::optional<std::pair<Eigen::Vector3d, double>> LocalMap::nearestSearch(const Eigen::Vector3d& point,
                                                                          const double maxDistSq) const
{
  Eigen::Vector3d nearestPoint = Eigen::Vector3d::Zero();
  double nearestDistSq = maxDistSq;
  auto voxelIndex = getVoxelIndex(point);

  // Add neighbor voxels
  std::vector<std::reference_wrapper<const Voxel>> voxels;
  voxels.reserve(27);
  for (const auto& offset : voxelOffsets_)
  {
    Eigen::Vector3i curVoxelIndex = voxelIndex + offset;
    VoxelGrid::const_iterator found = voxelGrid_.find(curVoxelIndex);
    if (found != voxelGrid_.cend())
    {
      voxels.push_back(found->second);
    }
  }

  for (const auto& voxel : voxels)
  {
    auto searchResult = voxel.get().nearestSearchInVoxel(point, nearestDistSq);
    if (searchResult.has_value())
    {
      nearestPoint = searchResult.value().first;
      nearestDistSq = searchResult.value().second;
    }
  }

  if (nearestDistSq < maxDistSq)
  {
    return std::make_pair(nearestPoint, nearestDistSq);
  }
  else
  {
    return std::nullopt;
  }
}

Eigen::Vector3i LocalMap::getVoxelIndex(const Eigen::Vector3d& point) const
{
  Eigen::Vector3i voxelIndex = (point / voxelSize_).array().floor().cast<int>();
  return voxelIndex;
}

std::vector<Eigen::Vector3i> LocalMap::initVoxelOffsets(size_t numVoxels)
{
  std::vector<Eigen::Vector3i> voxelOffsets;
  voxelOffsets.reserve(numVoxels);
  if (numVoxels >= 1)
  {
    voxelOffsets.emplace_back(0, 0, 0);
  }

  if (numVoxels >= 7)
  {
    voxelOffsets.emplace_back(1, 0, 0);
    voxelOffsets.emplace_back(-1, 0, 0);
    voxelOffsets.emplace_back(0, 1, 0);
    voxelOffsets.emplace_back(0, -1, 0);
    voxelOffsets.emplace_back(0, 0, 1);
    voxelOffsets.emplace_back(0, 0, -1);
  }

  if (numVoxels >= 19)
  {
    voxelOffsets.emplace_back(1, 1, 0);
    voxelOffsets.emplace_back(1, -1, 0);
    voxelOffsets.emplace_back(-1, 1, 0);
    voxelOffsets.emplace_back(-1, -1, 0);
    voxelOffsets.emplace_back(1, 0, 1);
    voxelOffsets.emplace_back(1, 0, -1);
    voxelOffsets.emplace_back(-1, 0, 1);
    voxelOffsets.emplace_back(-1, 0, -1);
    voxelOffsets.emplace_back(0, 1, 1);
    voxelOffsets.emplace_back(0, 1, -1);
    voxelOffsets.emplace_back(0, -1, 1);
    voxelOffsets.emplace_back(0, -1, -1);
  }

  if (numVoxels >= 27)
  {
    voxelOffsets.emplace_back(1, 1, 1);
    voxelOffsets.emplace_back(1, 1, -1);
    voxelOffsets.emplace_back(1, -1, 1);
    voxelOffsets.emplace_back(1, -1, -1);
    voxelOffsets.emplace_back(-1, 1, 1);
    voxelOffsets.emplace_back(-1, 1, -1);
    voxelOffsets.emplace_back(-1, -1, 1);
    voxelOffsets.emplace_back(-1, -1, -1);
  }

  return voxelOffsets;
}

bool LocalMap::visualizeLocalMap() const
{
  std::unique_lock<std::mutex> visualizerLock(visualizerMutex_);
  if (!visualizer_->PollEvents())
  {
    return false;  // exit sign
  }

  if (visualizer_->HasGeometry())
  {
    visualizer_->UpdateRender();
  }
  return true;
}

void LocalMap::save(const std::string& cloud_path, const std::string& trajectory_path) const
{
  auto cloud = std::make_shared<PointCloud>();
  cloud->points_.reserve( voxelGrid_.size() * maxNumPointsPerVoxel_ );

  for(const auto& [_, voxel] : voxelGrid_)
  {
    cloud->points_.insert(cloud->points_.end(), voxel.points.begin(), voxel.points.end());
  }

  open3d::io::WritePointCloud(cloud_path, *cloud);
  open3d::io::WritePinholeCameraTrajectory(trajectory_path, trajectory_);
}

}  // namespace ESKF_LIO
