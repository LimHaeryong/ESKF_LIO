#include <open3d/Open3D.h>

void visualizeMapcloud(
  std::shared_ptr<open3d::geometry::PointCloud> cloud,
  const open3d::camera::PinholeCameraParameters & parameters,
  const open3d::camera::PinholeCameraTrajectory & trajectory)
{
  auto visualizer = std::make_unique<open3d::visualization::Visualizer>();
  visualizer->CreateVisualizerWindow("Map cloud", 1600, 900);
  visualizer->GetRenderOption().SetPointSize(1.0);
  visualizer->GetRenderOption().background_color_ = {0, 0, 0};
  visualizer->GetRenderOption().point_color_option_ =
    open3d::visualization::RenderOption::PointColorOption::ZCoordinate;
  auto & viewControl = visualizer->GetViewControl();
  visualizer->AddGeometry(cloud);

  auto lineSet = std::make_shared<open3d::geometry::LineSet>();
  for (size_t i = 0; i < trajectory.parameters_.size(); ++i) {
    lineSet->points_.push_back(trajectory.parameters_[i].extrinsic_.block<3, 1>(0, 3));
    if (i > 0) {
      lineSet->lines_.emplace_back(i - 1, i);
    }
  }
  lineSet->PaintUniformColor({0, 1, 0});
  visualizer->AddGeometry(lineSet);

  for (size_t i = 0; i < trajectory.parameters_.size(); i += 50) {
    Eigen::Matrix3d R = trajectory.parameters_[i].extrinsic_.block<3, 3>(0, 0);
    Eigen::Vector3d t = trajectory.parameters_[i].extrinsic_.block<3, 1>(0, 3);
    auto coord = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0, t);
    coord->Rotate(R, t);
    visualizer->AddGeometry(coord);
  }

  viewControl.ConvertFromPinholeCameraParameters(parameters);
  visualizer->Run();
}

int main()
{
  open3d::camera::PinholeCameraParameters visualizerConfig;
  open3d::io::ReadIJsonConvertible(VISUALIZER_CONFIG_PATH, visualizerConfig);
  auto mapCloud = open3d::io::CreatePointCloudFromFile(CLOUD_SAVE_PATH);
  auto trajectory = open3d::io::CreatePinholeCameraTrajectoryFromFile(TRAJECTORY_SAVE_PATH);
  visualizeMapcloud(mapCloud, visualizerConfig, *trajectory);
  return 0;
}
