#include "ESKF_LIO/ErrorStateKF.hpp"
#include "ESKF_LIO/Utils.hpp"

#include <cmath>

namespace ESKF_LIO
{
ErrorStateKF::ErrorStateKF(const YAML::Node & config)
{
  constexpr double gravity_magnitude = 9.81;

  auto imu_update_rate = config["sensors"]["imu"]["update_rate"].as<double>();
  auto imu_params = config["sensors"]["imu"]["intrinsics"]["parameters"];
  auto bias_a = imu_params["bias_a"].as<std::vector<double>>();
  auto bias_g = imu_params["bias_g"].as<std::vector<double>>();
  auto gravity = imu_params["gravity"].as<std::vector<double>>();

  State initState;
  initState.bias_accel = Eigen::Map<Eigen::Vector3d>(bias_a.data());
  initState.bias_gyro = Eigen::Map<Eigen::Vector3d>(bias_g.data());
  initState.gravity = Eigen::Map<Eigen::Vector3d>(gravity.data());
  states_.push_back(std::move(initState));

  auto accel_noise_density = imu_params["accel_noise_density"].as<std::vector<double>>();
  auto accel_zero_g_offset = imu_params["accel_zero_g_offset"].as<double>();
  auto gyro_noise_density = imu_params["gyro_noise_density"].as<double>();
  auto gyro_zero_rate_offset = imu_params["gyro_zero_rate_offset"].as<double>();

  Eigen::Vector3d sigma_accel_noise =
    Eigen::Map<Eigen::Vector3d>(accel_noise_density.data()) * gravity_magnitude * std::sqrt(
    imu_update_rate);
  double sigma_gyro_noise = gyro_noise_density * std::sqrt(imu_update_rate) * M_PI / 180.0;
  double sigma_accel_work = accel_zero_g_offset * std::sqrt(imu_update_rate) * 1e-3 *
    gravity_magnitude;
  double sigma_gyro_work = gyro_zero_rate_offset * std::sqrt(imu_update_rate) * M_PI / 180.0;

  sigma_accel_noise = sigma_accel_noise.array().square();
  Q_.block<3, 3>(0, 0) = sigma_accel_noise.asDiagonal();
  Q_.block<3, 3>(3, 3) = std::pow(sigma_gyro_noise, 2.0) * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(6, 6) = std::pow(sigma_accel_work, 2.0) * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(9, 9) = std::pow(sigma_gyro_work, 2.0) * Eigen::Matrix3d::Identity();

  F_i_.setZero();
  F_i_.block<12, 12>(3, 0) = Mat12d::Identity();
}

void ErrorStateKF::process(ImuMeasurementPtr imu)
{
  const auto & prevState = states_.back();
  double dt = imu->timestamp - prevState.timestamp;
  if (dt < 0.0) {
    return;
  }
  State newState = prevState;
  newState.timestamp = imu->timestamp;
  Eigen::Matrix3d R = prevState.attitude.toRotationMatrix();
  Eigen::Vector3d acceleration = imu->acceleration - prevState.bias_accel;
  Eigen::Vector3d angular_velocity = imu->angularVelocity - prevState.bias_gyro;
  Eigen::Quaterniond angle_diff(Eigen::AngleAxisd(
      angular_velocity.norm() * dt,
      angular_velocity.normalized()));

  double dt2 = dt * dt;
  newState.position = prevState.position + prevState.velocity * dt + 0.5 *
    (R * acceleration + prevState.gravity) * dt2;
  newState.velocity = prevState.velocity + (R * acceleration + prevState.gravity) * dt;
  newState.attitude = prevState.attitude * angle_diff;

  Mat12d Q_i = Q_;
  Q_i.block<6, 6>(0, 0) *= dt2;
  Q_i.block<6, 6>(6, 6) *= dt;

  F_x_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3, 3>(3, 6) = -R * Utils::skewSymmetric(acceleration) * dt;
  F_x_.block<3, 3>(3, 9) = -R * dt;
  F_x_.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3, 3>(6, 6) = angle_diff.toRotationMatrix().transpose();
  F_x_.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  newState.P = F_x_ * prevState.P * F_x_.transpose() + F_i_ * Q_i * F_i_.transpose();

  states_.push_back(std::move(newState));

  // std::cout << "pose after process = " << states_.back().position.transpose() << "\n";
}

Eigen::Isometry3d ErrorStateKF::update(LidarMeasurementPtr lidar)
{
  double lidarEndTime = lidar->endTime;

  // Rolls back the state based on the lidarEndTime
  while (!states_.empty() && states_.back().timestamp > lidarEndTime) {
    states_.pop_back();
  }

  // TODO : ICP based Registration and State update
  State state = states_.back();


  // std::cout << "pose after update = " << state.position.transpose() << "\n";
  states_.push_back(state);
  // Discards unnecessary imuMeasurements
  while (!ImuMeasurements_.empty() && ImuMeasurements_.front()->timestamp < lidarEndTime) {
    ImuMeasurements_.pop_front();
  }

  for (auto & imu : ImuMeasurements_) {
    process(imu);
  }

  Eigen::Isometry3d transform;
  transform.linear() = state.attitude.toRotationMatrix();
  transform.translation() = state.position;

  return transform;
}

}  // namespace ESKF_LIO
