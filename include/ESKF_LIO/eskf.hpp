#ifndef ESKF_LIO_ESKF_HPP_
#define ESKF_LIO_ESKF_HPP_

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
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
            Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
            Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
            Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
        };

        ErrorStateKF(const YAML::Node &config);

        State static predictState(const State &state, double dt, const ImuMeasurement &imu);
        void process(double dt, const ImuMeasurement &imu);

    private:
        ErrorStateKF() = delete;

        // states
        State nominalState_;
        Mat18d P_ = Mat18d::Identity();
        Vec18d errorState_ = Vec18d::Zero();

        // variables for process update
        Mat18d F_x_ = Mat18d::Identity();
        Mat12d Q_;
        Eigen::Matrix<double, 18, 12> F_i_;

        // variables for measurement update
    };

}; // namespace ESKF_LIO

#endif // ESKF_LIO_ESKF_HPP_