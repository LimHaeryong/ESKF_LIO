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
  Utils::transformPoints(tmpCloud.points_, totalTransform);

  for (int i = 0; i < maxIteration_; ++i) {
    matchingRmsePrev_ = matchingRmse_;
    auto correspondence =
      localMap.correspondenceMatching(
      tmpCloud.points_, maxCorrespondenceDistSquared_,
      matchingRmse_);
    auto transformIter = computeTransform(correspondence);
    totalTransform = transformIter * totalTransform;

    if (convergenceCheck(transformIter)) {
      converged_ = true;
      break;
    }
    Utils::transformPoints(tmpCloud.points_, transformIter);
  }

  if (!converged_) {
    std::cout << "ICP not converged!\n";
  }

  return totalTransform;
}

bool ICP::convergenceCheck(const Eigen::Isometry3d & transformIter) const
{
  double relativeMatchingRmse = std::abs(matchingRmse_ - matchingRmsePrev_);
  if (relativeMatchingRmse > relativeMatchingRmseThreshold_) {
    return false;
  }

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
  auto & [srcPoints, dstPoints] = correspondence;

  Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> srcMatrix(srcPoints[0].data(), 3,
    srcPoints.size());
  Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> dstMatrix(dstPoints[0].data(), 3,
    dstPoints.size());

  Eigen::Vector3d srcMean = srcMatrix.rowwise().mean();
  Eigen::Vector3d dstMean = dstMatrix.rowwise().mean();

  srcMatrix.colwise() -= srcMean;
  dstMatrix.colwise() -= dstMean;

  Eigen::Matrix3d S = srcMatrix * dstMatrix.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  D(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();

  Eigen::Matrix3d R = svd.matrixV() * D * svd.matrixU().transpose();
  Eigen::Vector3d t = dstMean - R * srcMean;

  Eigen::Isometry3d transform;
  transform.linear() = R;
  transform.translation() = t;

  return transform;
}

}  // namespace ESKF_LIO
