#include <iostream>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "ESKF_LIO/Odometry.hpp"
#include "ESKF_LIO/Subscriber.hpp"
#include "ESKF_LIO/Types.hpp"

void visualizeMapcloud(std::shared_ptr<open3d::geometry::PointCloud> cloud,
                         const open3d::camera::PinholeCameraParameters& parameters,
                         const open3d::camera::PinholeCameraTrajectory& trajectory)
{
  auto visualizer = std::make_unique<open3d::visualization::Visualizer>();
  visualizer->CreateVisualizerWindow("Map cloud", 1600, 900);
  visualizer->GetRenderOption().SetPointSize(1.0);
  visualizer->GetRenderOption().background_color_ = { 0, 0, 0 };
  visualizer->GetRenderOption().point_color_option_ =
      open3d::visualization::RenderOption::PointColorOption::ZCoordinate;
  auto& viewControl = visualizer->GetViewControl();
  visualizer->AddGeometry(cloud);
  for(const auto& param : trajectory.parameters_)
  {
    Eigen::Matrix3d R = param.extrinsic_.block<3, 3>(0, 0);
    Eigen::Vector3d t = param.extrinsic_.block<3, 1>(0, 3);
    auto coord = open3d::geometry::TriangleMesh::CreateCoordinateFrame(3.0, t);
    coord->Rotate(R, t);
    visualizer->AddGeometry(coord);
  }
  viewControl.ConvertFromPinholeCameraParameters(parameters);
  visualizer->Run();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto config = YAML::LoadFile(CONFIG_PATH);
  auto imuTopic = config["sensors"]["imu"]["topic_name"].as<std::string>();
  auto lidarTopic = config["sensors"]["lidar"]["topic_name"].as<std::string>();

  open3d::camera::PinholeCameraParameters visualizerConfig;
  open3d::io::ReadIJsonConvertible(VISUALIZER_CONFIG_PATH, visualizerConfig);

  auto imuSubscriber = std::make_shared<ImuSubscriber>(imuTopic);
  auto lidarSubscriber = std::make_shared<LidarSubscriber>(lidarTopic);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(imuSubscriber);
  executor->add_node(lidarSubscriber);

  auto imuBuffer = imuSubscriber->getBuffer();
  auto cloudBuffer = lidarSubscriber->getBuffer();

  auto odom = std::make_shared<ESKF_LIO::Odometry>(config, imuBuffer, cloudBuffer, visualizerConfig);

  std::thread executorThread([&executor]() { executor->spin(); });

  odom->run();
  odom->saveMapcloud(CLOUD_SAVE_PATH, TRAJECTORY_SAVE_PATH);

  // load saved map cloud
  auto mapCloud = open3d::io::CreatePointCloudFromFile(CLOUD_SAVE_PATH);
  auto trajectory = open3d::io::CreatePinholeCameraTrajectoryFromFile(TRAJECTORY_SAVE_PATH);
  visualizeMapcloud(mapCloud, visualizerConfig, *trajectory);

  executor->cancel();
  executorThread.join();
  rclcpp::shutdown();

  return 0;
}
