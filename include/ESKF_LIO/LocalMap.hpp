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
  using VoxelHash = typename open3d::utility::hash_eigen<Eigen::Vector3i>;
  using VoxelGrid = typename std::unordered_map<Eigen::Vector3i, Voxel, VoxelHash>;

  LocalMap(const YAML::Node & config)
  : voxelSize_(config["local_map"]["voxel_size"].as<double>()),
    maxNumPointsPerVoxel_(config["local_map"]["max_num_points_per_voxel"].as<size_t>()),
    voxelOffsets_(initVoxelOffsets(config["local_map"]["num_voxel_offsets"].as<size_t>())) {}

  LocalMap(double voxelSize, size_t maxNumPointsPerVoxel, size_t numVoxelOffsets)
  : voxelSize_(voxelSize), maxNumPointsPerVoxel_(maxNumPointsPerVoxel), voxelOffsets_(initVoxelOffsets(
        numVoxelOffsets)) {}

  struct Voxel
  {
    size_t maxNumPoints;
    PointVector points;

    Voxel(size_t maxNumPoints, const Eigen::Vector3d & point)
    : maxNumPoints(maxNumPoints)
    {
      points.reserve(maxNumPoints);
      points.push_back(point);
    }

    void addPoint(const Eigen::Vector3d & point)
    {
      if (points.size() < maxNumPoints) {
        points.push_back(point);
      }
    }

    std::optional<std::pair<Eigen::Vector3d, double>> nearestSearchInVoxel(
      const Eigen::Vector3d & point, double maxDistSq) const;
  };

  void updateLocalMap(const PointVector & points);
  std::optional<std::pair<Eigen::Vector3d, double>> nearestSearch(
    const Eigen::Vector3d & point,
    double maxDistSq) const;

private:
  Eigen::Vector3i getVoxelIndex(const Eigen::Vector3d & point) const;

  double voxelSize_;
  size_t maxNumPointsPerVoxel_;
  VoxelGrid voxelGrid_;

  mutable std::shared_mutex mutex_;

  std::vector<Eigen::Vector3i> initVoxelOffsets(size_t numVoxels);
  const std::vector<Eigen::Vector3i> voxelOffsets_;
};
}  // namesapce ESKF_LIO

#endif // ESKF_LIO_LOCAL_MAP_HPP_
