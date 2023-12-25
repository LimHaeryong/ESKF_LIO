#ifndef ESKF_LIO_LOCAL_MAP_HPP_
#define ESKF_LIO_LOCAL_MAP_HPP_

#include <array>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "ESKF_LIO/Types.hpp"

namespace ESKF_LIO
{
class LocalMap
{
public:
  struct Voxel;

  using PointVector = typename std::vector<Eigen::Vector3d>;
  using Correspondence = typename std::pair<PointVector, PointVector>;
  using VoxelHash = typename open3d::utility::hash_eigen<Eigen::Vector3i>;
  using VoxelGrid = typename std::unordered_map<Eigen::Vector3i, Voxel, VoxelHash>;

  LocalMap(const YAML::Node& config, const open3d::camera::PinholeCameraParameters& visualizerConfig,
           bool visualize = true)
    : voxelSize_(config["local_map"]["voxel_size"].as<double>())
    , maxNumPointsPerVoxel_(config["local_map"]["max_num_points_per_voxel"].as<size_t>())
    , voxelOffsets_(initVoxelOffsets(config["local_map"]["num_voxel_offsets"].as<size_t>()))
    , visualizerConfig_(visualizerConfig)
    , visualize_(visualize)
  {
    if (visualize)
    {
      visualizer_ = std::make_shared<open3d::visualization::Visualizer>();
      visualizer_->CreateVisualizerWindow("Local Map", 1600, 900);
      visualizer_->GetRenderOption().SetPointSize(1.0);
      visualizer_->GetRenderOption().background_color_ = { 0, 0, 0 };
      visualizer_->GetViewControl().ConvertFromPinholeCameraParameters(visualizerConfig);
    }
  }

  LocalMap(double voxelSize, size_t maxNumPointsPerVoxel, size_t numVoxelOffsets, bool visualize = false)
    : voxelSize_(voxelSize)
    , maxNumPointsPerVoxel_(maxNumPointsPerVoxel)
    , voxelOffsets_(initVoxelOffsets(numVoxelOffsets))
    , visualize_(visualize)
  {
  }

  struct Voxel
  {
    size_t maxNumPoints;
    PointVector points;

    Voxel(size_t maxNumPoints, const Eigen::Vector3d& point) : maxNumPoints(maxNumPoints)
    {
      points.reserve(maxNumPoints);
      points.push_back(point);
    }

    void addPoint(const Eigen::Vector3d& point)
    {
      if (points.size() < maxNumPoints)
      {
        points.push_back(point);
      }
    }

    std::optional<std::pair<Eigen::Vector3d, double>> nearestSearchInVoxel(const Eigen::Vector3d& point,
                                                                           const double maxDistSq) const;
  };

  void updateLocalMap(PointCloudPtr cloud, const Eigen::Isometry3d& transform);
  Correspondence correspondenceMatching(const PointVector& points, const double maxDistSq, double& matchingRmse) const;
  std::optional<std::pair<Eigen::Vector3d, double>> nearestSearch(const Eigen::Vector3d& point,
                                                                  const double maxDistSq) const;

  bool visualizeLocalMap() const;
  void save(const std::string& cloud_path, const std::string& trajectory_path) const;

private:
  Eigen::Vector3i getVoxelIndex(const Eigen::Vector3d& point) const;

  double voxelSize_;
  size_t maxNumPointsPerVoxel_;
  VoxelGrid voxelGrid_;

  mutable std::mutex visualizerMutex_;
  mutable std::shared_mutex voxelGridMutex_;

  std::vector<Eigen::Vector3i> initVoxelOffsets(size_t numVoxels);
  const std::vector<Eigen::Vector3i> voxelOffsets_;

  open3d::camera::PinholeCameraParameters visualizerConfig_;

  bool visualize_;
  std::shared_ptr<open3d::visualization::Visualizer> visualizer_;

  open3d::camera::PinholeCameraTrajectory trajectory_;
};
}  // namespace ESKF_LIO

#endif  // ESKF_LIO_LOCAL_MAP_HPP_
