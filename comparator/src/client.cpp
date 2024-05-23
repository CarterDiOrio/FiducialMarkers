#include <DataStreamClient.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory.h>
#include "comparator/tracked_object.hpp"
#include "comparator/vicon.hpp"
#include "comparator/mount.hpp"
#include "comparator/realsense_camera_impl.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace datastream = ViconDataStreamSDK::CPP;

std::optional<datastream::Client> init_vicon()
{
  std::string server{"169.254.100.131:801"};
  auto client_ret = vicon::connect_to_server(server);

  if (!client_ret) {
    std::cerr << "Failed to connect to the server" << std::endl;
    return {};
  }
  auto client = client_ret.value();

  if (!vicon::configure_datastream(client)) {
    return {};
  }

  return client;
}

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
  // connecting to the vicon server
  std::cout << "Connecting to the vicon server..." << std::endl;
  auto maybe_client = init_vicon();
  if (!maybe_client) {
    return 1;
  }
  std::cout << "Connected to the vicon server" << std::endl;
  auto & client = maybe_client.value();

  // load the realsense
  RealSenseCamera camera(1920, 1080, 30);

  // get the camera intrinsics
  cv::Mat K = camera.get_intrinsics();

  // Read the mount and fiducials definition
  auto mount = init_mount("../mount.txt");

  // For now the fiducial on the mount is going to be a 10x7 chessboard for testing purposes.
  auto fiducial_mount_vicon = vicon::TrackedObject{
    .subject_name = "FiducialMount",
    .root_segment_name = "FiducialMount"
  };

  while (client.GetFrame().Result == datastream::Result::Success) {

    auto maybe_pose = vicon::get_object_transform(fiducial_mount_vicon, client);
    if (!maybe_pose) {
      std::cerr << "Failed to get pose" << std::endl;
      continue; // BAD: Temporary, Refactoring later.
    }
    auto pose = maybe_pose.value();

    // get the image from the camera
    auto image = camera.get_frame();

    // find the chessboard corners
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(image, cv::Size(10, 7), corners);

    if (found) {

      // Using the vicon pose find the fiducial corners in the camera frame
      std::vector<cv::Mat> fiducial_corners_world;
      for (const Eigen::Vector3d & corner: mount.fiducial_corners) {
        Eigen::Vector4d corner_world = pose * mount.T_mp * corner.homogeneous();
        cv::Mat corner_w_cv;
        cv::eigen2cv(corner_world, corner_w_cv);
        fiducial_corners_world.push_back(corner_w_cv);
      }

      cv::Mat rvec, tvec;
      cv::solvePnP(fiducial_corners_world, corners, K, cv::Mat(), rvec, tvec);

      // convert the rvec and tvec to a 4x4 matrix
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      Eigen::Matrix3d R_eig;
      cv::cv2eigen(R, R_eig);

      Eigen::Vector3d T = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};

      // The transformation is given as T_cw and we want T_wc
      Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
      T_cw.block<3, 3>(0, 0) = R_eig.transpose();
      T_cw.block<3, 1>(0, 3) = -1 * R_eig.transpose() * T;

      std::cout << "Translation: " << T_cw.block<3, 1>(0, 3).transpose() << std::endl;
    }

  }


  return 0;
}
