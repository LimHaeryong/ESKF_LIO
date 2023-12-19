#ifndef ESKF_LIO_UTILS_HPP_
#define ESKF_LIO_UTILS_HPP_

#include <Eigen/Dense>

namespace ESKF_LIO::Utils
{
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & vec)
{

  Eigen::Matrix3d skew;
  skew << 0.0, -vec(2), vec(1),
    vec(2), 0.0, -vec(0),
    -vec(1), vec(0), 0.0;

  return skew;
}

}

#endif // ESKF_LIO_UTILS_HPP_
