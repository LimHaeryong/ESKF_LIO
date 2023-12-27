#ifndef ESKF_LIO_ERROR_STATE_KALMAN_FILTER_HPP_
#define ESKF_LIO_ERROR_STATE_KALMAN_FILTER_HPP_

#include <deque>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "ESKF_LIO/Types.hpp"
#include "ESKF_LIO/LocalMap.hpp"
#include "ESKF_LIO/Registration.hpp"

namespace ESKF_LIO
{
class ErrorStateKF
{
public:
  using Mat18d = typename Eigen::Matrix<double, 18, 18>;
  using Vec18d = typename Eigen::Vector<double, 18>;
  using Mat12d = typename Eigen::Matrix<double, 12, 12>;
  using Mat6d = typename Eigen::Matrix<double, 6, 6>;


  ErrorStateKF(const YAML::Node & config);
  const std::deque<State> & getStates() const {return states_;}
  double getLastStateTime() const {return states_.back().timestamp;}

  void feedImu(ImuMeasurementPtr imu) {ImuMeasurements_.push_back(std::move(imu));}
  void initialize(double lidarEndTime);
  void process(ImuMeasurementPtr imu);
  Eigen::Isometry3d update(const LidarMeasurement & lidar, const LocalMap & localMap);

private:
  ErrorStateKF() = delete;
  void injectError(State & state, const Vec18d & errorState) const;
  void reset(State & state, const Vec18d & errorState);

  std::shared_ptr<ICP> icp_;

  // states
  std::deque<State> states_;
  std::deque<ImuMeasurementPtr> ImuMeasurements_;

  // variables for process update
  Mat18d F_x_ = Mat18d::Identity();
  Mat12d Q_;
  Eigen::Matrix<double, 18, 12> F_i_;

  // variables for measurement update
  Eigen::Matrix<double, 6, 18> H_;
  Mat6d V_;
  Mat18d G_;
};

}  // namespace ESKF_LIO

#endif // ESKF_LIO_ERROR_STATE_KALMAN_FILTER_HPP_
