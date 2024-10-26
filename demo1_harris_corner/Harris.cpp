#include <string.h>

#include <iostream>
#include <opencv2/opencv.hpp>

void progressBar(std::string msgs, int progress, int total, int barWidth = 50) {
  std::cout << '\r' << msgs << '[';
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
}

std::vector<cv::Point> Harris(cv::Mat &img) {
  std::vector<cv::Point> corners;

  cv::Mat dx, dy;
  cv::Sobel(img, dx, CV_32F, 1, 0);
  cv::Sobel(img, dy, CV_32F, 0, 1);

  cv::imshow("dx", dx);
  cv::waitKey(0);
  cv::imshow("dy", dy);
  cv::waitKey(0);

  // Compute Ix^2, Iy^2, IxIy
  cv::Mat Ix2 = dx.mul(dx);
  cv::Mat Iy2 = dy.mul(dy);
  cv::Mat Ixy = dx.mul(dy);

  // Gaussian filter
  cv::GaussianBlur(Ix2, Ix2, cv::Size(5, 5), 1);
  cv::GaussianBlur(Iy2, Iy2, cv::Size(5, 5), 1);
  cv::GaussianBlur(Ixy, Ixy, cv::Size(5, 5), 1);

  //   cv::imshow("Ix2", Ix2);
  //   cv::waitKey(0);
  //   cv::imshow("Iy2", Iy2);
  //   cv::waitKey(0);
  //   cv::imshow("Ixy", Ixy);
  //   cv::waitKey(0);

  // Compute Harris response
  cv::Mat R = cv::Mat::zeros(img.size(), CV_32F);
  float k = 0.04;
  int base_window_size = 3;
  for (int i = base_window_size / 2; i < img.rows - base_window_size / 2; i++) {
    for (int j = base_window_size / 2; j < img.cols - base_window_size / 2;
         j++) {
      progressBar("Calculating Harris: ", i * img.cols + j,
                  img.rows * img.cols);

      // Dynamic Window Size
      int max_window_size = 19;
      if (i < max_window_size / 2) max_window_size = i * 2 - 1;
      if (j < max_window_size / 2) max_window_size = j * 2 - 1;
      if (i >= img.rows - max_window_size / 2)
        max_window_size = (img.rows - i) * 2 - 1;
      if (j >= img.cols - max_window_size / 2)
        max_window_size = (img.cols - j) * 2 - 1;
      if (max_window_size % 2 == 0) max_window_size -= 1;

      int dynamic_window_size = base_window_size;
      while (dynamic_window_size <= max_window_size) {
        dynamic_window_size += 2;
        // Compute Hessian matrix
        cv::Mat H = cv::Mat::zeros(2, 2, CV_32F);
        for (int u = -dynamic_window_size / 2; u <= dynamic_window_size / 2;
             u++) {
          for (int v = -dynamic_window_size / 2; v <= dynamic_window_size / 2;
               v++) {
            H.at<float>(0, 0) += Ix2.at<float>(i + u, j + v);
            H.at<float>(0, 1) += Ixy.at<float>(i + u, j + v);
            H.at<float>(1, 0) += Ixy.at<float>(i + u, j + v);
            H.at<float>(1, 1) += Iy2.at<float>(i + u, j + v);
          }
        }

        // Compute Harris response
        float det = H.at<float>(0, 0) * H.at<float>(1, 1) -
                    H.at<float>(0, 1) * H.at<float>(1, 0);
        float trace = H.at<float>(0, 0) + H.at<float>(1, 1);
        if (det - k * trace * trace > R.at<float>(i, j))
          R.at<float>(i, j) = det - k * trace * trace;
      }
    }
  }

  // Thresholding
  double Rmax = -1;
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (R.at<float>(i, j) > Rmax) {
        Rmax = R.at<float>(i, j);
      }
    }
  }

  double threshold = 0.01 * Rmax;
  for (int i = 1; i < img.rows - 1; i++) {
    for (int j = 1; j < img.cols - 1; j++) {
      // Non-maximum suppression
      if (R.at<float>(i, j) > threshold &&
          R.at<float>(i, j) > R.at<float>(i - 1, j - 1) &&
          R.at<float>(i, j) > R.at<float>(i - 1, j) &&
          R.at<float>(i, j) > R.at<float>(i - 1, j + 1) &&
          R.at<float>(i, j) > R.at<float>(i, j - 1) &&
          R.at<float>(i, j) > R.at<float>(i, j + 1) &&
          R.at<float>(i, j) > R.at<float>(i + 1, j - 1) &&
          R.at<float>(i, j) > R.at<float>(i + 1, j) &&
          R.at<float>(i, j) > R.at<float>(i + 1, j + 1)) {
        corners.push_back(cv::Point(j, i));
      }
    }
  }

  return corners;
}

int main(int argc, char **argv) {
  cv::Mat img = cv::imread(argv[1], 0);
  std::vector<cv::Point> corners = Harris(img);
  img = cv::imread(argv[1]);
  for (auto &corner : corners) {
    cv::circle(img, corner, 4, cv::Scalar(255, 255, 0), -1);
  }
  cv::imwrite(argv[2], img);
  cv::imshow("Harris Corner", img);
  cv::waitKey(0);
  return 0;
}