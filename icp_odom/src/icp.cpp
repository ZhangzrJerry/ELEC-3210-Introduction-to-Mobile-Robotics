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

  for(int iter=0;iter<params::max_iterations;iter++)
  {
    // Step 1: Update the source point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_trans;
    for(int i=0;i<src_cloud->size();i++)
    {
      Eigen::Vector4d src_point(src_cloud->points[i].x, src_cloud->points[i].y, src_cloud->points[i].z, 1);
      Eigen::Vector4d trans_point = transformation * src_point;
      src_cloud->points[i].x = trans_point(0);
      src_cloud->points[i].y = trans_point(1);
      src_cloud->points[i].z = trans_point(2);
    }

    // Step 2: Find the closest points
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(tar_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<src_cloud_trans->size();i++)
    {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      kdtree.nearestKSearch(src_cloud_trans->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      src_cloud_filtered->push_back(src_cloud_trans->points[i]);
      tar_cloud_filtered->push_back(tar_cloud->points[pointIdxNKNSearch[0]]);
    }

    // Step 3: Calculate the MSE
    double mse = 0;
    for(int i=0;i<src_cloud_filtered->size();i++)
    {
      Eigen::Vector4d src_point(src_cloud_filtered->points[i].x, src_cloud_filtered->points[i].y, src_cloud_filtered->points[i].z, 1);
      Eigen::Vector4d tar_point(tar_cloud_filtered->points[i].x, tar_cloud_filtered->points[i].y, tar_cloud_filtered->points[i].z, 1);
      mse += (src_point - tar_point).squaredNorm();
    }
    mse /= src_cloud_filtered->size();

    // Step 4: Calculate the centroid of the two point clouds
    Eigen::Vector4d src_centroid, tar_centroid;
    pcl::compute3DCentroid(*src_cloud_filtered, src_centroid);
    pcl::compute3DCentroid(*tar_cloud_filtered, tar_centroid);

    // Step 5: Calculate the covariance matrix
    Eigen::Matrix3d src_cov, tar_cov;
    pcl::computeCovarianceMatrixNormalized(*src_cloud_filtered, src_centroid, src_cov);
    pcl::computeCovarianceMatrixNormalized(*tar_cloud_filtered, tar_centroid, tar_cov);

    // Step 6: Calculate the rotation matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(src_cov.transpose() * tar_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    // Step 7: Calculate the translation vector
    Eigen::Vector3d t = tar_centroid.head(3) - R * src_centroid.head(3);

    // Step 8: Update the transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    transformation = T * transformation;
  }

  return transformation;
}