#include "ESKF_LIO/ErrorStateKF.hpp"
#include "ESKF_LIO/Utils.hpp"

#include <cmath>

namespace ESKF_LIO
{
ErrorStateKF::ErrorStateKF(const YAML::Node& config) : icp_(std::make_shared<ICP>(config))
{
  constexpr double GRAVITY_MAGNITUDE = 9.81;

  auto imuUpdateRate = config["sensors"]["imu"]["update_rate"].as<double>();
  auto imuParams = config["sensors"]["imu"]["intrinsics"]["parameters"];
  auto biasAccel = imuParams["bias_a"].as<std::vector<double>>();
  auto biasGyro = imuParams["bias_g"].as<std::vector<double>>();
  auto gravity = imuParams["gravity"].as<std::vector<double>>();

  State initState;
  initState.biasAccel = Eigen::Map<Eigen::Vector3d>(biasAccel.data());
  initState.biasGyro = Eigen::Map<Eigen::Vector3d>(biasGyro.data());
  initState.gravity = Eigen::Map<Eigen::Vector3d>(gravity.data());
  states_.push_back(std::move(initState));

  auto accelNoiseDensity = imuParams["accel_noise_density"].as<std::vector<double>>();
  auto accelZeroGOffset = imuParams["accel_zero_g_offset"].as<double>();
  auto gyroNoiseDensity = imuParams["gyro_noise_density"].as<double>();
  auto gyroZeroRateOffset = imuParams["gyro_zero_rate_offset"].as<double>();

  Eigen::Vector3d sigmaAccelNoise =
      Eigen::Map<Eigen::Vector3d>(accelNoiseDensity.data()) * GRAVITY_MAGNITUDE * std::sqrt(imuUpdateRate);
  double sigmaGyroNoise = gyroNoiseDensity * std::sqrt(imuUpdateRate) * M_PI / 180.0;
  double sigmaAccelWork = accelZeroGOffset * std::sqrt(imuUpdateRate) * 1e-3 * GRAVITY_MAGNITUDE;
  double sigmaGyroWork = gyroZeroRateOffset * std::sqrt(imuUpdateRate) * M_PI / 180.0;

  sigmaAccelNoise = sigmaAccelNoise.array().square();
  Q_.block<3, 3>(0, 0) = sigmaAccelNoise.asDiagonal();
  Q_.block<3, 3>(3, 3) = std::pow(sigmaGyroNoise, 2.0) * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(6, 6) = std::pow(sigmaAccelWork, 2.0) * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(9, 9) = std::pow(sigmaGyroWork, 2.0) * Eigen::Matrix3d::Identity();

  F_i_.setZero();
  F_i_.block<12, 12>(3, 0) = Mat12d::Identity();

  double translationNoise = config["kalman_filter"]["update"]["translation_noise"].as<double>();
  double rotationNoise = config["kalman_filter"]["update"]["rotation_noise"].as<double>();

  V_.setZero();
  V_.block<3, 3>(0, 0) = translationNoise * Eigen::Matrix3d::Identity();
  V_.block<3, 3>(3, 3) = rotationNoise * Eigen::Matrix3d::Identity();

  H_.setZero();
  H_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  G_ = Mat18d::Identity();
}

void ErrorStateKF::initialize(double lidarEndTime)
{
  auto& initState = states_[0];
  initState.timestamp = lidarEndTime;

  // Discards unnecessary imuMeasurements
  while (!ImuMeasurements_.empty() && ImuMeasurements_.front()->timestamp < lidarEndTime)
  {
    ImuMeasurements_.pop_front();
  }

  // process remaining imuMeasurements
  for (auto& imu : ImuMeasurements_)
  {
    process(imu);
  }
}

void ErrorStateKF::process(ImuMeasurementPtr imu)
{
  const auto& prevState = states_.back();
  double dt = imu->timestamp - prevState.timestamp;
  if (dt < 0.0)
  {
    return;
  }
  State newState = prevState;
  newState.timestamp = imu->timestamp;
  Eigen::Matrix3d R = prevState.attitude.toRotationMatrix();
  Eigen::Vector3d acceleration = imu->acceleration - prevState.biasAccel;
  Eigen::Vector3d angularVelocity = imu->angularVelocity - prevState.biasGyro;
  Eigen::Quaterniond angleDiff(Eigen::AngleAxisd(angularVelocity.norm() * dt, angularVelocity.normalized()));

  double dt2 = dt * dt;
  newState.position = prevState.position + prevState.velocity * dt + 0.5 * (R * acceleration + prevState.gravity) * dt2;
  newState.velocity = prevState.velocity + (R * acceleration + prevState.gravity) * dt;
  newState.attitude = prevState.attitude * angleDiff;

  Mat12d Q_i = Q_;
  Q_i.block<6, 6>(0, 0) *= dt2;
  Q_i.block<6, 6>(6, 6) *= dt;

  F_x_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3, 3>(3, 6) = -R * Utils::skewSymmetric(acceleration) * dt;
  F_x_.block<3, 3>(3, 9) = -R * dt;
  F_x_.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3, 3>(6, 6) = angleDiff.conjugate().toRotationMatrix();
  F_x_.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  newState.P = F_x_ * prevState.P * F_x_.transpose() + F_i_ * Q_i * F_i_.transpose();

  states_.push_back(std::move(newState));

  // newState.printState();
}

Eigen::Isometry3d ErrorStateKF::update(const LidarMeasurement& lidar, const LocalMap& localMap)
{
  const double lidarEndTime = lidar.endTime;

  // Rolls back the state before the lidarEndTime
  while (!states_.empty() && states_.back().timestamp > lidarEndTime)
  {
    states_.pop_back();
  }

  // TODO : ICP based Registration and State update
  State newState = states_.back();
  newState.timestamp = lidarEndTime;
  const State& prevState = states_.back();
  Eigen::Isometry3d guess;
  guess.linear() = prevState.attitude.toRotationMatrix();
  guess.translation() = prevState.position;
  auto observation = icp_->align(*lidar.cloud, localMap, guess);

  Eigen::Vector<double, 6> residual;
  residual.head<3>() = observation.translation() - guess.translation();
  residual.tail<3>() = Utils::rotationMatrixToVector(guess.linear().transpose() * observation.linear());
  Eigen::Matrix<double, 18, 6> K = prevState.P * H_.transpose() * (H_ * prevState.P * H_.transpose() + V_).inverse();
  Eigen::Vector<double, 18> errorState = K * residual;

  Mat18d I_KH = Mat18d::Identity() - K * H_;
  //newState.P = I_KH * prevState.P * I_KH.transpose() + K * V_ * K.transpose();
  newState.P = I_KH * prevState.P;
  injectError(newState, errorState);
  reset(newState, errorState);

  states_.push_back(newState);
  // Discards imuMeasurements before the lidarEndTime
  while (!ImuMeasurements_.empty() && ImuMeasurements_.front()->timestamp < lidarEndTime)
  {
    ImuMeasurements_.pop_front();
  }

  // process remaining imuMeasurements
  for (auto& imu : ImuMeasurements_)
  {
    process(imu);
  }

  Eigen::Isometry3d transform;
  transform.linear() = newState.attitude.toRotationMatrix();
  transform.translation() = newState.position;

  return transform;
}

void ErrorStateKF::injectError(State& state, const Vec18d& errorState) const
{
  state.position += errorState.block<3, 1>(0, 0);
  state.velocity += errorState.block<3, 1>(3, 0);
  state.attitude *= Utils::rotationVectorToQuaternion(errorState.block<3, 1>(6, 0));
  state.biasAccel += errorState.block<3, 1>(9, 0);
  state.biasGyro += errorState.block<3, 1>(12, 0);
  state.gravity += errorState.block<3, 1>(15, 0);
}

void ErrorStateKF::reset(State& state, const Vec18d& errorState)
{
  Eigen::Vector3d rotationError = errorState.block<3, 1>(6, 0);
  Eigen::Matrix3d skew = Utils::skewSymmetric(rotationError);
  G_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * skew;
  state.P = G_ * state.P * G_.transpose();
}

}  // namespace ESKF_LIO
