#ifndef ESKF_LIO_CLOUD_PREPROCESSOR_HPP_
#define ESKF_LIO_CLOUD_PREPROCESSOR_HPP_

#include <open3d/Open3D.h>

#include <yaml-cpp/yaml.h>

#include "ESKF_LIO/Types.hpp"
#include "ESKF_LIO/ErrorStateKF.hpp"

namespace ESKF_LIO
{
class CloudPreprocessor
{
public:
  CloudPreprocessor(const YAML::Node & config)
  {
    voxelSize_ = config["cloud_preprocessor"]["voxel_size"].as<double>();

    auto lidar = config["sensors"]["lidar"];
    auto lidar_quat = lidar["extrinsics"]["quaternion"].as<std::vector<double>>();
    auto lidar_trans = lidar["extrinsics"]["translation"].as<std::vector<double>>();

    Eigen::Quaterniond quat = Eigen::Map<Eigen::Quaterniond>(lidar_quat.data());
    Eigen::Vector3d trans = Eigen::Map<Eigen::Vector3d>(lidar_trans.data());

    T_il_.linear() = quat.toRotationMatrix();
    T_il_.translation() = trans;
  }

  void process(const std::deque<State> & states, LidarMeasurementPtr lidarMeas) const;
  void voxelDownsample(
    std::vector<Eigen::Vector3d> & points,
    std::vector<Eigen::Vector3d> & normals) const;

private:
  CloudPreprocessor() = delete;

  void deskew(
    const std::deque<State> & states, const std::vector<double> & pointTime,
    std::vector<Eigen::Vector3d> & points) const;


  Eigen::Vector3i getVoxelIndex(const Eigen::Vector3d & point) const;

  double voxelSize_;
  Eigen::Isometry3d T_il_;
};
}        // namespace ESKF_LIO

#endif  // ESKF_LIO_CLOUD_PREPROCESSOR_HPP_
