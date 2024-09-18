/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include "parameters.h"

// Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
//                                  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,
//                                  Eigen::Matrix4d init_guess)
// {
//   // This is an example of using pcl::IterativeClosestPoint to align two point
//   // clouds In your project, you should implement your own ICP algorithm!!!

//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(src_cloud);
//   icp.setInputTarget(tar_cloud);
//   icp.setMaximumIterations(params::max_iterations); // set maximum iteration
//   icp.setTransformationEpsilon(1e-6);               // set transformation epsilon
//   icp.setEuclideanFitnessEpsilon(1e-6);             // set euclidean distance difference epsilon
//   icp.setRANSACOutlierRejectionThreshold(0.);       // set RANSAC outlier rejection threshold
//   icp.setMaxCorrespondenceDistance(
//       params::max_distance); // set maximum correspondence distance
//   pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
//   icp.align(aligned_cloud, init_guess.cast<float>());

//   Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
//   return transformation;
// }

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess)
{
  Eigen::Matrix4d transformation = init_guess;

  // calculate the centroid of src_cloud and tar_cloud
  Eigen::Vector4f src_centroid;
  pcl::compute3DCentroid(*src_cloud, src_centroid);

  Eigen::Vector4f tar_centroid;
  pcl::compute3DCentroid(*tar_cloud, tar_centroid);

  // Center the clouds by subtracting the centroid
  for (auto &point : *src_cloud)
  {
    point.getVector3fMap() -= src_centroid.head<3>();
  }

  for (auto &point : *tar_cloud)
  {
    point.getVector3fMap() -= tar_centroid.head<3>();
  }

  for (int iter = 0; iter < params::max_iterations; iter++)
  {
    pcl::PointCloud<pcl::PointXYZ> src_cloud_trans;
    for (const auto &point : *src_cloud)
    {
      pcl::PointXYZ transformed_point;
      Eigen::Vector4f homogenous_point(point.x, point.y, point.z, 1.0);
      Eigen::Vector4f transformed_homogenous_point = transformation * homogenous_point;
      transformed_point.x = transformed_homogenous_point.x();
      transformed_point.y = transformed_homogenous_point.y();
      transformed_point.z = transformed_homogenous_point.z();
      src_cloud_trans.push_back(transformed_point);
    }

    // calculate the nearest point to build the corrspondence
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(tar_cloud);

    std::vector<Eigen::Vector3f> src_points;
    std::vector<Eigen::Vector3f> tar_points;

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    for (const auto &src_point : *src_cloud_trans)
    {
      if (kdtree.nearestKSearch(src_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        src_points.push_back(src_point.getVector3fMap());
        tar_points.push_back((*tar_cloud)[pointIdxNKNSearch[0]].getVector3fMap());
      }
    }

    // calculate the error
    double MSE = 0;
    for (size_t i = 0; i < src_points.size(); ++i)
    {
      MSE += (src_points[i] - tar_points[i]).squaredNorm();
    }
    MSE /= src_points.size();

    if (MSE < 1e-6)
    {
    }
  }

  return transformation;
}