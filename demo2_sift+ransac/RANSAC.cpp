#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

cv::RNG rng(12345);
cv::Mat K = (cv::Mat_<float>(3, 3) << 1024, 0.0, 960.0, 0.0, 1024, 720.0, 0.0,
             0.0, 1.0);

void progressBar(std::string msgs, int progress, int total, int barWidth = 50) {
  if (progress == 1) {
    std::cout << msgs << ": start" << std::endl;
  }
  std::cout << '\r' << msgs << ": [";
  int pos = barWidth * progress / total;
  for (int i = 0; i < barWidth; ++i) {
    if (i < pos)
      std::cout << "=";
    else if (i == pos)
      std::cout << ">";
    else
      std::cout << " ";
  }
  std::cout << "] " << int((float)progress / total * 100.0) << " %\r";
  std::cout.flush();
  if (progress == total) {
    std::cout << "\n" << msgs << ": end" << std::endl;
  }
  return;
}

cv::Mat findEssentialMatrixRansac(const std::vector<cv::Point2f>& points1,
                                  const std::vector<cv::Point2f>& points2,
                                  const cv::Mat& K, std::vector<uchar>& inliers,
                                  int RANSAC_ITERATIONS = 10000,
                                  double RANSAC_INLIER_THRESHOLD = 2.0) {
  int N = std::min(points1.size(), points2.size());
  cv::Mat best_E, K_inv = K.inv();
  int best_inliers = 0;

  for (int iter = 0; iter < RANSAC_ITERATIONS; ++iter) {
    progressBar("RANSAC iteration", iter + 1, RANSAC_ITERATIONS);
    // sample 8 points
    std::vector<cv::Point2f> points1_sample, points2_sample;
    for (int i = 0; i < 8; ++i) {
      int idx = rng.uniform(0, N);
      points1_sample.push_back(points1[idx]);
      points2_sample.push_back(points2[idx]);
    }

    // estimate essential matrix
    cv::Mat E = cv::findEssentialMat(points1_sample, points2_sample, K,
                                     cv::FM_8POINT, 0.999, 1.0);
    E.convertTo(E, CV_32F);

    // count inliers
    std::vector<uchar> curr_inliers(N, 0);
    int curr_inliers_count = 0;
    for (int i = 0; i < N; ++i) {
      cv::Mat p1 = (cv::Mat_<float>(3, 1) << points1[i].x, points1[i].y, 1.0);
      cv::Mat p2 = (cv::Mat_<float>(3, 1) << points2[i].x, points2[i].y, 1.0);
      cv::Mat epolar_line = p2.t() * K_inv * E;
      float norm =
          std::sqrt(epolar_line.at<float>(0) * epolar_line.at<float>(0) +
                    epolar_line.at<float>(1) * epolar_line.at<float>(1));
      cv::Mat err = epolar_line * K_inv * p1;
      if (std::abs(err.at<float>(0, 0)) < RANSAC_INLIER_THRESHOLD) {
        curr_inliers[i] = 1;
        curr_inliers_count++;
      }
    }

    // update best model
    if (curr_inliers_count > best_inliers) {
      best_inliers = curr_inliers_count;
      best_E = E.clone();
      inliers = curr_inliers;
    }
  }

  std::cout << "total iterations: " << RANSAC_ITERATIONS << std::endl;
  std::cout << "inlier counts   :" << best_inliers << std::endl;
  std::cout << "total keypoints : " << N << std::endl;

  return best_E;
}

int main(int argc, char** argv) {
  // load images
  cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  cv::resize(img1, img1, cv::Size(1920, 1440));
  cv::resize(img2, img2, cv::Size(1920, 1440));

  // detect keypoints and compute descriptors
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  detector->detect(img1, keypoints1);
  detector->detect(img2, keypoints2);
  cv::Mat descriptors1, descriptors2;
  detector->compute(img1, keypoints1, descriptors1);
  detector->compute(img2, keypoints2, descriptors2);

  // match keypoints via flann
  std::vector<cv::Point2f> points1, points2;
  std::vector<cv::DMatch> matches;
  cv::FlannBasedMatcher matcher;
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance < 0.7 * knn_matches[i][1].distance) {
      // Lowe's ratio test
      matches.push_back(knn_matches[i][0]);
      points1.push_back(cv::Point2f(
          static_cast<float>(keypoints1[knn_matches[i][0].queryIdx].pt.x),
          static_cast<float>(keypoints1[knn_matches[i][0].queryIdx].pt.y)));
      points2.push_back(cv::Point2f(
          static_cast<float>(keypoints2[knn_matches[i][0].trainIdx].pt.x),
          static_cast<float>(keypoints2[knn_matches[i][0].trainIdx].pt.y)));
    }
  }

  // find essential matrix via RANSAC
  std::vector<uchar> inliers;
  cv::Mat E = findEssentialMatrixRansac(points1, points2, K, inliers, 10000);

  // draw inliers
  std::vector<cv::DMatch> inlier_matches;
  for (int i = 0; i < inliers.size(); i += 20) {
    if (inliers[i]) {
      inlier_matches.push_back(matches[i]);
    }
  }
  cv::Mat img1_color = cv::imread(argv[1]);
  cv::Mat img2_color = cv::imread(argv[2]);
  img1.resize(1920, 1440);
  img2.resize(1920, 1440);
  cv::Mat img_matches;
  cv::drawMatches(img1_color, keypoints1, img2_color, keypoints2,
                  inlier_matches, img_matches, cv::Scalar::all(-1),
                  cv::Scalar::all(-1), std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::resize(img_matches, img_matches,
             cv::Size(img_matches.cols / 2, img_matches.rows / 2));
  cv::imshow("inliers", img_matches);
  cv::waitKey(0);
  cv::imwrite(argv[3], img_matches);

  return 0;
}