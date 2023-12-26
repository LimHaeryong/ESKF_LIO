#include <memory>
#include <gtest/gtest.h>

#include "ESKF_LIO/LocalMap.hpp"

class LocalMapTest : public ::testing::Test
{
protected:
  std::shared_ptr<ESKF_LIO::LocalMap> localMap;
  double voxelSize;
  size_t maxNumPointsPerVoxel;
  size_t numVoxelOffsets;

  void SetUp() override
  {
    voxelSize = 1.0;
    maxNumPointsPerVoxel = 20;
    numVoxelOffsets = 7;
    localMap =
      std::make_shared<ESKF_LIO::LocalMap>(voxelSize, maxNumPointsPerVoxel, numVoxelOffsets);
  }
};

TEST_F(LocalMapTest, updateLocalMap)
{
  ESKF_LIO::LocalMap::PointVector points;
  Eigen::Vector3d p1(1.2, 0.2, 0.9);
  Eigen::Vector3d p2(2.2, 1.5, 0.3);
  Eigen::Vector3d p3(1.2, 0.2, 0.8);
  points.push_back(p1);
  points.push_back(p2);

  auto cloud = std::make_shared<open3d::geometry::PointCloud>();
  cloud->points_ = points;
  localMap->updateLocalMap(cloud, Eigen::Isometry3d::Identity());
  auto searchResult = localMap->nearestSearch(p1, 100.0);

  ASSERT_TRUE(searchResult.has_value());
  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(searchResult.value().first[i], p1[i], 1e-6);
  }
  ASSERT_NEAR(searchResult.value().second, 0.0, 1e-6);
}

TEST_F(LocalMapTest, NearestSearchInVoxel)
{
  ESKF_LIO::LocalMap::PointVector points;
  Eigen::Vector3d p1(0.5, 0.4, 0.5);
  Eigen::Vector3d p2(0.1, 0.7, 0.2);
  Eigen::Vector3d p3(0.5, 0.45, 0.5);
  Eigen::Vector3d p4(0.5, 0.5, 0.5);
  ESKF_LIO::LocalMap::Voxel voxel(maxNumPointsPerVoxel, p1);
  voxel.addPoint(p2);
  voxel.addPoint(p3);

  auto searchResult = voxel.nearestSearchInVoxel(p4, 100.0);
  ASSERT_TRUE(searchResult.has_value());
  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(searchResult.value().first[i], p3[i], 1e-6);
  }
  ASSERT_NEAR(searchResult.value().second, (p3 - p4).squaredNorm(), 1e-6);
}

TEST_F(LocalMapTest, NearestSearch)
{


  ESKF_LIO::LocalMap::PointVector points;
  Eigen::Vector3d p1(0.1, 0.4, 0.1);
  Eigen::Vector3d p2(-0.001, -0.001, 0.1);
  Eigen::Vector3d p3(0.1, 0.1, -0.1);
  Eigen::Vector3d p4(0.1, 0.1, 0.1);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);

  auto cloud = std::make_shared<open3d::geometry::PointCloud>();
  cloud->points_ = points;
  localMap->updateLocalMap(cloud, Eigen::Isometry3d::Identity());

  auto searchResult = localMap->nearestSearch(p4, 100.0);

  ASSERT_TRUE(searchResult.has_value());
  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(searchResult.value().first[i], p3[i], 1e-6);
  }
  ASSERT_NEAR(searchResult.value().second, (p3 - p4).squaredNorm(), 1e-6);

}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
