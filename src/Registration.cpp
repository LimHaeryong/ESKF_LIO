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
  Utils::rotateNormals(tmpCloud.normals_, totalTransform.linear());

  for (int i = 0; i < maxIteration_; ++i) {
    matchingRmsePrev_ = matchingRmse_;
    auto correspondence =
      localMap.correspondenceMatching(
      tmpCloud.points_, tmpCloud.normals_, maxCorrespondenceDistSquared_,
      matchingRmse_);
    auto transformIter = computeTransform(correspondence);
    totalTransform = transformIter * totalTransform;

    if (convergenceCheck(transformIter)) {
      converged_ = true;
      break;
    }
    Utils::transformPoints(tmpCloud.points_, transformIter);
    Utils::rotateNormals(tmpCloud.normals_, transformIter.linear());
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
  auto & [srcPoints, srcNormals, mapPoints, mapNormals] = correspondence;

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
        JTri] = computeJTJAndJTr(srcPoints[i], mapPoints[i], srcNormals[i] + mapNormals[i]);
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
  const Eigen::Vector3d & normal) const
{
  Eigen::Vector<double, 6> JT;
  Eigen::Vector<double, 1> r;
  JT.head(3) = normal;
  JT.tail(3) = srcPoint.cross(normal);
  r = (srcPoint - mapPoint).transpose() * normal;
  return std::make_pair(JT * JT.transpose(), JT * r);
}

}  // namespace ESKF_LIO
