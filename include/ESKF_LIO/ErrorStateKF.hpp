#ifndef ESKF_LIO_ERROR_STATE_KALMAN_FILTER_HPP_
#define ESKF_LIO_ERROR_STATE_KALMAN_FILTER_HPP_

#include <deque>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "ESKF_LIO/Types.hpp"

namespace ESKF_LIO
{
class ErrorStateKF
{
public:
  using Mat18d = typename Eigen::Matrix<double, 18, 18>;
  using Vec18d = typename Eigen::Vector<double, 18>;
  using Mat12d = typename Eigen::Matrix<double, 12, 12>;

  struct State
  {
    double timestamp;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    Mat18d P = Mat18d::Identity();
  };

  ErrorStateKF(const YAML::Node & config);
  const std::deque<State> & getStates() const {return states_;}
  double getLastStateTime() const {return states_.back().timestamp;}

  void feedImu(ImuMeasurementPtr imu) {ImuMeasurements_.push_back(std::move(imu));}
  void initState(double lidarEndTime) {states_[0].timestamp = lidarEndTime;}
  void process(ImuMeasurementPtr imu);
  Eigen::Isometry3d update(LidarMeasurementPtr lidar);

private:
  ErrorStateKF() = delete;

  // states
  std::deque<State> states_;
  std::deque<ImuMeasurementPtr> ImuMeasurements_;

  // variables for process update
  Mat18d F_x_ = Mat18d::Identity();
  Mat12d Q_;
  Eigen::Matrix<double, 18, 12> F_i_;

  // variables for measurement update
};

}  // namespace ESKF_LIO

#endif // ESKF_LIO_ERROR_STATE_KALMAN_FILTER_HPP_
