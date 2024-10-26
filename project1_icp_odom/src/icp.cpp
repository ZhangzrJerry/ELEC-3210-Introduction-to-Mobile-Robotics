/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include "parameters.h"

// Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr
// src_cloud,
//                                  pcl::PointCloud<pcl::PointXYZ>::Ptr
//                                  tar_cloud, Eigen::Matrix4d init_guess)
// {
//   // This is an example of using pcl::IterativeClosestPoint to align two
//   point
//   // clouds In your project, you should implement your own ICP algorithm!!!

//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(src_cloud);
//   icp.setInputTarget(tar_cloud);
//   icp.setMaximumIterations(params::max_iterations); // set maximum iteration
//   icp.setTransformationEpsilon(1e-6);               // set transformation
//   epsilon icp.setEuclideanFitnessEpsilon(1e-6);             // set euclidean
//   distance difference epsilon icp.setRANSACOutlierRejectionThreshold(0.); //
//   set RANSAC outlier rejection threshold icp.setMaxCorrespondenceDistance(
//       params::max_distance); // set maximum correspondence distance
//   pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
//   icp.align(aligned_cloud, init_guess.cast<float>());

//   Eigen::Matrix4d transformation =
//   icp.getFinalTransformation().cast<double>(); return transformation;
// }

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,
                                 Eigen::Matrix4d init_guess) {
  Eigen::Matrix4d transformation = init_guess;

  for (int iter = 0; iter < params::max_iterations; iter++) {
    // Step 1: Update the source point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_trans(
        new pcl::PointCloud<pcl::PointXYZ>(*src_cloud));
    for (int i = 0; i < src_cloud->size(); i++) {
      Eigen::Vector4d src_point(src_cloud->points[i].x, src_cloud->points[i].y,
                                src_cloud->points[i].z, 1);
      Eigen::Vector4d trans_point = transformation * src_point;
      src_cloud_trans->points[i].x = trans_point(0);
      src_cloud_trans->points[i].y = trans_point(1);
      src_cloud_trans->points[i].z = trans_point(2);
    }

    // Step 2: Build the correspondences
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints(
        new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints(
        new pcl::PointCloud<pcl::PointWithScale>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());

    sift.setSearchMethod(tree);
    sift.setScales(0.01f, 3, 2);
    sift.setMinimumContrast(0.0);
    sift.setInputCloud(src_cloud_trans);
    sift.compute(*src_keypoints);

    sift.setInputCloud(tar_cloud);
    sift.compute(*tar_keypoints);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*src_keypoints, *src_keypoints_xyz);
    pcl::copyPointCloud(*tar_keypoints, *tar_keypoints_xyz);

    pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(
            src_keypoints_xyz));
    model->setInputTarget(tar_keypoints_xyz);

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(params::max_distance);
    ransac.computeModel();

    Eigen::VectorXf ransac_coefficients;
    ransac.getModelCoefficients(ransac_coefficients);

    // Step 3: Calculate the MSE
    double mse = 0;
    for (int i = 0; i < src_keypoints_xyz->size(); i++) {
      Eigen::Vector3d src_point(src_keypoints_xyz->points[i].x,
                                src_keypoints_xyz->points[i].y,
                                src_keypoints_xyz->points[i].z);
      Eigen::Vector3d tar_point(tar_keypoints_xyz->points[i].x,
                                tar_keypoints_xyz->points[i].y,
                                tar_keypoints_xyz->points[i].z);
      mse += (src_point - tar_point).squaredNorm();
    }
    mse /= src_keypoints_xyz->size();

    if (mse < 1e-6) {
      break;
    }

    // Step 4: Convert RANSAC coefficients to transformation matrix
    Eigen::Matrix4f ransac_transformation = Eigen::Matrix4f::Identity();
    ransac_transformation.block<3, 3>(0, 0) =
        Eigen::Quaternionf(ransac_coefficients[6], ransac_coefficients[3],
                           ransac_coefficients[4], ransac_coefficients[5])
            .toRotationMatrix();
    ransac_transformation.block<3, 1>(0, 3) = ransac_coefficients.head<3>();

    transformation = ransac_transformation * transformation;
  }

  return transformation;
}