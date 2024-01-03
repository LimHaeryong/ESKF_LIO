#ifndef ESKF_LIO_LOCAL_MAP_HPP_
#define ESKF_LIO_LOCAL_MAP_HPP_

#include <array>
#include <optional>
#include <tuple>
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
  using CovarianceVector = typename std::vector<Eigen::Matrix3d>;
  using Correspondence = typename std::tuple<PointVector, CovarianceVector, PointVector, CovarianceVector>;
  using VoxelHash = typename open3d::utility::hash_eigen<Eigen::Vector3i>;
  using VoxelGrid = typename std::unordered_map<Eigen::Vector3i, Voxel, VoxelHash>;

  LocalMap(
    const YAML::Node & config, const open3d::camera::PinholeCameraParameters & visualizerConfig,
    bool visualize = true)
  : voxelSize_(config["local_map"]["voxel_size"].as<double>())
    , maxNumPointsPerVoxel_(config["local_map"]["max_num_points_per_voxel"].as<size_t>())
    , translationSquaredThreshold_(config["local_map"]["update"]["translation_sq_threshold"].as<double>())
    , cosineThreshold_(config["local_map"]["update"]["cosine_threshold"].as<double>())
    , visualizerConfig_(visualizerConfig)
    , visualize_(visualize)
  {
    if (visualize) {
      visualizer_ = std::make_shared<open3d::visualization::Visualizer>();
      visualizer_->CreateVisualizerWindow("Local Map", 1600, 900);
      visualizer_->GetRenderOption().SetPointSize(1.0);
      visualizer_->GetRenderOption().background_color_ = {0, 0, 0};
      visualizer_->GetViewControl().ConvertFromPinholeCameraParameters(visualizerConfig);
    }
  }

  LocalMap(
    double voxelSize, size_t maxNumPointsPerVoxel, 
    bool visualize = false)
  : voxelSize_(voxelSize)
    , maxNumPointsPerVoxel_(maxNumPointsPerVoxel)
    , visualize_(visualize)
  {
  }

  struct Voxel
  {
    size_t maxNumPoints;
    size_t numPoints;
    PointVector points;
    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;


    Voxel(size_t maxNumPoints, const Eigen::Vector3d & point, const Eigen::Matrix3d & covariance)
    : maxNumPoints(maxNumPoints), numPoints(1), mean(point), covariance(covariance)
    {
      points.reserve(maxNumPoints);
      points.push_back(point);
    }

    void addPoint(const Eigen::Vector3d & point, const Eigen::Matrix3d & covariance)
    {
      if (numPoints < maxNumPoints) {
        points.push_back(point);
        mean = (numPoints * mean + point) / (numPoints + 1);
        this->covariance = (numPoints * this->covariance + covariance) / (numPoints + 1);
        ++numPoints;
      }
    }

  };

  void updateLocalMap(PointCloudPtr cloud, const Eigen::Isometry3d & transform, bool initialize = false);
  Correspondence correspondenceMatching(
    const PointVector & points, const CovarianceVector & covariances) const;

  bool visualizeLocalMap() const;
  void save(const std::string & cloud_path, const std::string & trajectory_path) const;

private:
  Eigen::Vector3i getVoxelIndex(const Eigen::Vector3d & point) const;
  bool needsMapUpdate(const Eigen::Isometry3d & transform) const;

  double voxelSize_;
  size_t maxNumPointsPerVoxel_;
  double translationSquaredThreshold_;
  double cosineThreshold_;
  Eigen::Isometry3d prevTransform_;

  VoxelGrid voxelGrid_;

  open3d::camera::PinholeCameraParameters visualizerConfig_;

  bool visualize_;
  std::shared_ptr<open3d::visualization::Visualizer> visualizer_;

  open3d::camera::PinholeCameraTrajectory trajectory_;
};
}  // namespace ESKF_LIO

#endif  // ESKF_LIO_LOCAL_MAP_HPP_
