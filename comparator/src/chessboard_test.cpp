#include <Eigen/Dense>
#include <iostream>
#include <memory.h>
#include "comparator/camera.hpp"
#include "comparator/mount.hpp"
#include "comparator/realsense_camera_impl.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <fstream>

Mount init_mount(const std::string & file_path)
{
  auto mount = load_mount(file_path);
  std::cout << "Loaded Mount: " << std::endl;
  std::cout << "  " << "Mount Id: " << mount.fiducial_id << std::endl;
  std::cout << "  " << "Mount Parameters: " << mount.fiducial_parameters << std::endl;
  std::cout << "  " << "Mount Num Corners: " << mount.fiducial_corners.size() << std::endl;
  return mount;
}

int main()
{
  // load the realsense
  RealSenseCamera camera(1920, 1080, 30);

  // get the camera intrinsics
  cv::Mat K = camera.get_intrinsics();

  auto mount = init_mount("../mount.txt");


  std::vector<double> x, y, z;

  while (true) {
    const auto image = camera.get_frame();
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // find the chessboard
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, cv::Size(10, 7), corners);
    if (!found) {
      continue;
    }

    //refine the corners using subpixels
    auto winSize = cv::Size(5, 5);
    auto zeroZone = cv::Size(-1, -1);
    auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
    cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);

    // create the world points
    std::vector<cv::Point3f> world_points;
    for (const Eigen::Vector3d & corner: mount.fiducial_corners) {
      world_points.push_back(
        cv::Point3d(
          corner.x(),
          corner.y(),
          corner.z()));
    }

    // Find extrinsics using PnP
    cv::Mat rvec, tvec;
    cv::solvePnPRansac(
      world_points, corners, K, cv::noArray(), rvec, tvec,
      false, 350, 8.0, 0.999, cv::noArray(), cv::SOLVEPNP_ITERATIVE);


    // refine PnP solution
    cv::solvePnPRefineLM(world_points, corners, K, cv::noArray(), rvec, tvec);

    // find reprojection error
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(world_points, rvec, tvec, K, cv::noArray(), reprojected_points);

    double error = 0;
    for (size_t i = 0; i < corners.size(); i++) {
      error += cv::norm(reprojected_points[i] - corners[i]);
    }
    error /= corners.size();


    // convert the rvec and tvec to a 4x4 matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d R_eig;
    cv::cv2eigen(R, R_eig);

    Eigen::Vector3d T = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
    T_wc.block<3, 3>(0, 0) = R_eig.transpose();
    T_wc.block<3, 1>(0, 3) = -R_eig.transpose() * T;

    std::cout << "Reprojection error: " << error << " " <<
      T_wc.block<3, 1>(0, 3).transpose() << std::endl;


    x.push_back(T_wc(0, 3));
    y.push_back(T_wc(1, 3));
    z.push_back(T_wc(2, 3));

    // draw the chess board
    cv::drawChessboardCorners(image, cv::Size(10, 7), corners, found);
    cv::imshow("Image", image);
    char c = cv::waitKey(1);
    if (c == 'q') {
      break;
    }
  }

  // write x, y, z to a csv file for analysis
  auto file = std::ofstream("../position.csv");
  for (size_t i = 0; i < x.size(); i++) {
    file << x[i] << "," << y[i] << "," << z[i] << std::endl;
  }


  return 0;
}
