#include <DataStreamClient.h>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <memory.h>
#include "comparator/camera.hpp"
#include "comparator/tracked_object.hpp"
#include "comparator/vicon.hpp"
#include "comparator/mount.hpp"
#include "comparator/realsense_camera_impl.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <optional>
#include <ratio>

namespace datastream = ViconDataStreamSDK::CPP;

std::optional<std::unique_ptr<datastream::Client>> init_vicon()
{
  std::string server{"169.254.100.131:801"};
  auto client_ret = vicon::connect_to_server(server);


  if (!client_ret) {
    std::cerr << "Failed to connect to the server" << std::endl;
    return {};
  }
  auto & client = client_ret.value();

  if (!vicon::configure_datastream(*client)) {
    return {};
  }

  return std::move(client_ret.value());
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

/// \brief A measurement of the fiducial for PnP
struct Measurement
{
  /// \brief The image points
  std::vector<cv::Point2d> image_points;

  /// \brief The world points
  std::vector<cv::Point3d> world_points;
};

/// \brief Measures the calibration pattern
/// \param client The vicon client, assumed to be configured in ClientPull mode.
/// \param camera The camera to use for the measurement
/// \param tracked_object The tracked object to measure
/// \param mount The mount the fiducial is on
/// \return The measurement
std::optional<Measurement> measure_calibration_pattern(
  datastream::Client & client,
  const CameraIntf & camera,
  const vicon::TrackedObject & tracked_object,
  const Mount & mount);

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
  RealSenseCamera camera(1280, 720, 30);

  // get the camera intrinsics
  cv::Mat K = camera.get_intrinsics();

  // Read the mount and fiducials definition
  auto mount = init_mount("../mount.txt");

  // For now the fiducial on the mount is going to be a 10x7 chessboard for testing purposes.
  auto fiducial_mount_vicon = vicon::TrackedObject{
    .subject_name = "FiducialMount",
    .root_segment_name = "FiducialMount"
  };


  // The duration of data gathering
  std::chrono::duration<double> duration = std::chrono::seconds(30); // TODO: Make this a parameter

  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start < duration) {

    if (client->GetFrame().Result != datastream::Result::Success) {
      std::cerr << "Failed to get frame" << std::endl;
      return 1;
    }

    auto maybe_pose = vicon::get_object_transform(fiducial_mount_vicon, *client);
    if (!maybe_pose) {
      std::cerr << "Failed to get pose" << std::endl;
      continue; // BAD: Temporary, Refactoring later.
    }

    // The pose of the mount in the world frame
    auto T_wm = maybe_pose.value();

    // get the image from the camera
    auto image = camera.get_frame();

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Using the vicon pose find the fiducial corners in the camera frame
    std::vector<cv::Point3f> fiducial_corners_world;
    for (const Eigen::Vector3d & corner: mount.fiducial_corners) {
      Eigen::Vector4d corner_world = corner.homogeneous();
      fiducial_corners_world.push_back(
        cv::Point3d(
          corner_world.x(),
          corner_world.y(),
          corner_world.z()));
    }


    // find the chessboard corners
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, cv::Size(10, 7), corners);

    if (found) {
      // refine corners subpix
      auto winSize = cv::Size(5, 5);
      auto zeroZone = cv::Size(-1, -1);
      auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
      cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);

      // display detected corners
      cv::drawChessboardCorners(image, cv::Size(10, 7), corners, found);
      cv::imshow("Image", image);

      cv::Mat rvec, tvec;
      cv::solvePnPRansac(
        fiducial_corners_world, corners, K, cv::noArray(), rvec, tvec,
        false, 200, 2.0, 0.99, cv::noArray(), cv::SOLVEPNP_SQPNP);

      // convert the rvec and tvec to a 4x4 matrix
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      Eigen::Matrix3d R_eig;
      cv::cv2eigen(R, R_eig);

      Eigen::Vector3d T = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};

      // The transformation is given as T_cw and we want T_wc
      Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
      T_cw.block<3, 3>(0, 0) = R_eig;
      T_cw.block<3, 1>(0, 3) = T;

      Eigen::Matrix4d T_pc = Eigen::Matrix4d::Identity();
      T_pc.block<3, 3>(0, 0) = R_eig.transpose();
      T_pc.block<3, 1>(0, 3) = -R_eig.transpose() * T;

      std::cout << "T_pc: " << T_pc.block<3, 1>(0, 3).transpose() << std::endl;
    } else {
      cv::imshow("Image", image);
    }

    auto c = cv::waitKey(1);
    if (c == 'q') {
      break;
    }
  }


  return 0;
}

std::optional<Measurement> measure_calibration_pattern(
  datastream::Client & client,
  const CameraIntf & camera,
  const vicon::TrackedObject & tracked_object,
  const Mount & mount)
{

  // Get the pose from the vicon system
  if (client.GetFrame().Result != datastream::Result::Success) {
    std::cerr << "Failed to get frame" << std::endl;
    return {};
  }

  auto maybe_pose = vicon::get_object_transform(tracked_object, client);
  if (!maybe_pose) {
    std::cerr << "Failed to get pose" << std::endl;
    return {};
  }

  // The pose of the mount in the world frame
  auto T_wm = maybe_pose.value();

  const auto image = camera.get_frame();
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);


  // Get the world points of the fiducial
  std::vector<cv::Point3d> fiducial_corners_world;
  for (const Eigen::Vector3d & corner: mount.fiducial_corners) {
    Eigen::Vector4d corner_world = T_wm * mount.T_mp * corner.homogeneous();
    fiducial_corners_world.push_back(
      cv::Point3d(
        corner_world.x(),
        corner_world.y(),
        corner_world.z()));
  }

  // find the chessboard
  std::vector<cv::Point2d> corners;
  bool found = cv::findChessboardCorners(gray, cv::Size(10, 7), corners);
  if (!found) {
    return {};
  }

  // refine the corners using subpixels
  auto winSize = cv::Size(5, 5);
  auto zeroZone = cv::Size(-1, -1);
  auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
  cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);

  return Measurement{
    .image_points = corners,
    .world_points = fiducial_corners_world
  };
}
