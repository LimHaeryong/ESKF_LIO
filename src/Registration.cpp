#include "ESKF_LIO/Registration.hpp"
#include "ESKF_LIO/Utils.hpp"

namespace ESKF_LIO
{

Eigen::Isometry3d ICP::align(
  const PointCloud & cloud, const LocalMap & localMap,
  const Eigen::Isometry3d & guess)
{
  PointCloud tmpCloud = cloud;
  Eigen::Isometry3d totalTransform = guess;
  tmpCloud.Transform(totalTransform.matrix());

  for (int i = 0; i < maxIteration_; ++i) {
    auto correspondence =
      localMap.correspondenceMatching(
      tmpCloud.points_, tmpCloud.covariances_);
    auto transformIter = computeTransform(correspondence);
    totalTransform = transformIter * totalTransform;

    if (convergenceCheck(transformIter)) {
      converged_ = true;
      break;
    }

    tmpCloud.Transform(transformIter.matrix());
  }

  if (!converged_) {
    std::cout << "ICP not converged!\n";
  }

  return totalTransform;
}

bool ICP::convergenceCheck(const Eigen::Isometry3d & transformIter) const
{
  double cosine = 0.5 * (transformIter.linear().trace() - 1.0);
  if (cosine < cosineThreshold_) {
    return false;
  }

  double translationSq = transformIter.translation().squaredNorm();
  if (translationSq > translationSquaredThreshold_) {
    return false;
  }

  return true;
}

Eigen::Isometry3d ICP::computeTransform(Correspondence & correspondence) const
{
  auto & [srcPoints, srcCovs, mapPoints, mapCovs] = correspondence;

  Eigen::Matrix<double, 6, 6> JTJ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Vector<double, 6> JTr = Eigen::Vector<double, 6>::Zero();

  const size_t numCorr = srcPoints.size();
#pragma omp parallel
  {
    Eigen::Matrix<double, 6, 6> JTJPrivate = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Vector<double, 6> JTrPrivate = Eigen::Vector<double, 6>::Zero();
#pragma omp for nowait
    for (size_t i = 0; i < numCorr; ++i) {
      auto [JTJi,
        JTri] = computeJTJAndJTr(srcPoints[i], mapPoints[i], srcCovs[i] + mapCovs[i]);
      JTJPrivate += JTJi;
      JTrPrivate += JTri;
    }
#pragma omp critical
    {
      JTJ += JTJPrivate;
      JTr += JTrPrivate;
    }
  }

  Eigen::Vector<double, 6> se3 = JTJ.ldlt().solve(-JTr);
  Eigen::Isometry3d transform = Utils::se3ToSE3(se3);
  return transform;
}

std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Vector<double, 6>>
ICP::computeJTJAndJTr(
  const Eigen::Vector3d & srcPoint,
  const Eigen::Vector3d & mapPoint,
  const Eigen::Matrix3d & covariance) const
{
  Eigen::Matrix<double, 6, 6> JTJ;
  Eigen::Vector<double, 6> JTr;

  Eigen::Matrix<double, 3, 6> se3Jacobian;
  se3Jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  se3Jacobian.block<3, 3>(0, 3) = -Utils::skewSymmetric(srcPoint);
  Eigen::Matrix3d covarianceInv = covariance.inverse();
  Eigen::Matrix<double, 6, 3> JT = se3Jacobian.transpose() * covarianceInv;
  Eigen::Vector3d r = srcPoint - mapPoint;
  // double error = std::sqrt((r.transpose() * covarianceInv * r).coeff(0)) + 1e-6;
  JTJ = JT * se3Jacobian;
  JTr = JT * r;
  return std::make_pair(JTJ, JTr);
}

}  // namespace ESKF_LIO
