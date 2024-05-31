#include <DataStreamClient.h>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <iostream>
#include <memory.h>
#include "comparator/camera.hpp"
#include "comparator/tracked_object.hpp"
#include "comparator/vicon.hpp"
#include "comparator/mount.hpp"
#include "comparator/measurement.hpp"
#include "comparator/realsense_camera_impl.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <optional>
#include <opencv2/core/persistence.hpp>
#include <boost/program_options.hpp>

namespace datastream = ViconDataStreamSDK::CPP;
namespace po = boost::program_options;

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
  std::cout << "  " << "Mount Translation: " <<
    mount.T_mp.block<3, 1>(0, 3).transpose() << std::endl;
  return mount;
}


/// \brief Gathers measurements using the vicon system
/// \return The measurements
std::vector<Measurement> gather_vicon_measurements(const Mount & mount);

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

int main(int argc, char ** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
    "measurements",
    po::value<std::string>()->default_value("")->implicit_value(""),
    "Load measurements from a file to calculate extrinsics")(
    "mount",
    po::value<std::string>()->default_value("../mount.txt")->implicit_value("../mount.txt"),
    "The path to the mount file")
  ;

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (po::error & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  const std::string measurements_file = vm["measurements"].as<std::string>();

  auto mount = init_mount("../mount.txt");

  std::vector<Measurement> measurements;
  if (!measurements_file.empty()) {
    measurements = load_measurements(measurements_file);
    std::cout << "Loaded " << measurements.size() << " measurements" << std::endl;
  } else {
    // Gather measurements
    measurements = gather_vicon_measurements(mount);
  }

  // load the realsense
  RealSenseCamera camera(1920, 1080, 30);
  const auto K = camera.get_intrinsics();

  // Combine all the measurements and use them to estimate the extrinsics
  std::vector<cv::Point3d> world_points;
  std::vector<cv::Point2d> image_points;
  for (const auto & measurement: measurements) {
    world_points.insert(
      world_points.end(),
      measurement.world_points.begin(), measurement.world_points.end());
    image_points.insert(
      image_points.end(),
      measurement.image_points.begin(), measurement.image_points.end());
  }

  std::cout << "total points: " << world_points.size() << std::endl;

  // Find extrinsics using PnP
  cv::Mat rvec, tvec;
  cv::solvePnPRansac(
    world_points, image_points, K, cv::noArray(), rvec, tvec,
    false, 350, 8.0, 0.999, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

  // refine PnP solution
  cv::solvePnPRefineLM(world_points, image_points, K, cv::noArray(), rvec, tvec);

  // calculate the reprojectione error for each of the measurements
  std::cout << "Reprojection Errors: \n";
  std::vector<double> reprojection_errors;
  for (const auto & measurement: measurements) {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
      measurement.world_points, rvec, tvec, K, cv::noArray(), projected_points);


    double error = 0.0;
    for (size_t i = 0; i < measurement.image_points.size(); i++) {
      error += cv::norm(measurement.image_points[i] - projected_points[i]);
    }
    error /= measurement.image_points.size();

    // find the PnP between the camera and the fiducial in local
    cv::Mat rvec2, tvec2;
    cv::solvePnPRansac(
      measurement.fiducial_points, measurement.image_points, K, cv::noArray(), rvec2, tvec2,
      false, 350, 8.0, 0.999, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

    // refine
    cv::solvePnPRefineLM(
      measurement.fiducial_points, measurement.image_points, K,
      cv::noArray(), rvec2, tvec2);

    // project
    std::vector<cv::Point2f> projected_points2;
    cv::projectPoints(
      measurement.fiducial_points, rvec2, tvec2, K, cv::noArray(), projected_points2);

    double error2 = 0.0;
    for (size_t i = 0; i < measurement.image_points.size(); i++) {
      error2 += cv::norm(measurement.image_points[i] - projected_points2[i]);
    }
    error2 /= measurement.image_points.size();

    std::cout << "  World to Camera: " << error << " Camera to Fiducial: " << error2 << std::endl;

    reprojection_errors.push_back(error);
  }

  const double average_reprojection_error =
    std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0) /
    reprojection_errors.size();
  std::cout << "  Average: " << average_reprojection_error << std::endl;

  // convert the rvec and tvec to a 4x4 matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d R_eig;
  cv::cv2eigen(R, R_eig);

  Eigen::Vector3d T = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
  Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
  T_wc.block<3, 3>(0, 0) = R_eig.transpose();
  T_wc.block<3, 1>(0, 3) = -R_eig.transpose() * T;

  std::cout << "Estimated extrinsics: " << std::endl;
  std::cout << T_wc << std::endl;

  // write to json file
  cv::FileStorage fs("extrinsics.json", cv::FileStorage::WRITE); // TODO: Make output file path a parameter.

  cv::Mat extrinsics;
  cv::eigen2cv(T_wc, extrinsics);
  fs << "extrinsics" << extrinsics;
  fs.release();

  //write measurements to file
  fs.open("measurements.json", cv::FileStorage::WRITE);
  fs << "measurements" << "[";
  for (const auto & measurement: measurements) {
    fs << "{";
    fs << "image_points" << cv::Mat(measurement.image_points).reshape(1);
    fs << "world_points" << cv::Mat(measurement.world_points).reshape(1);
    fs << "fiducial_points" << cv::Mat(measurement.fiducial_points).reshape(1);
    fs << "}";
  }
  fs.release();

  return 0;
}

std::vector<Measurement> gather_vicon_measurements(const Mount & mount)
{
  // connecting to the vicon server
  std::cout << "Connecting to the vicon server..." << std::endl;
  auto maybe_client = init_vicon();
  if (!maybe_client) {
    throw std::runtime_error("Failed to connect to the vicon server");
  }

  std::cout << "Connected to the vicon server" << std::endl;
  auto & client = maybe_client.value();

  // load the realsense
  RealSenseCamera camera(1920, 1080, 30);

  // get the camera intrinsics
  cv::Mat K = camera.get_intrinsics();

  // For now the fiducial on the mount is going to be a 10x7 chessboard for testing purposes.
  auto fiducial_mount_vicon = vicon::TrackedObject{
    .subject_name = "FiducialMount",
    .root_segment_name = "FiducialMount"
  };

  std::vector<Measurement> measurements;

  while (true) {

    auto maybe_measurement =
      measure_calibration_pattern(*client, camera, fiducial_mount_vicon, mount);

    auto c = cv::waitKey(16);

    if (c == 'q') {
      break;
    } else if (c == 's') {
      if (maybe_measurement.has_value()) {
        measurements.push_back(maybe_measurement.value());
        std::cout << "Measurement added: " << measurements.size() << std::endl;
      }
    }

  }

  return measurements;
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
  std::vector<cv::Point3f> fiducial_corners_world;
  std::vector<cv::Point3f> fiducial_points;
  for (const Eigen::Vector3d & corner: mount.fiducial_corners) {
    Eigen::Vector4d corner_world = T_wm * mount.T_mp * corner.homogeneous();
    fiducial_corners_world.push_back(
      cv::Point3d(
        corner_world.x(),
        corner_world.y(),
        corner_world.z()));

    fiducial_points.push_back(
      cv::Point3d(
        corner.x(),
        corner.y(),
        corner.z()));
  }

  // find the chessboard
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(gray, cv::Size(10, 7), corners);
  if (!found) {
    cv::imshow("Image", image);
    return {};
  }


  //refine the corners using subpixels
  auto winSize = cv::Size(5, 5);
  auto zeroZone = cv::Size(-1, -1);
  auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
  cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);

  // draw the chess board
  cv::drawChessboardCorners(image, cv::Size(10, 7), corners, found);
  cv::imshow("Image", image);

  return Measurement{
    .image_points = corners,
    .world_points = fiducial_corners_world,
    .fiducial_points = fiducial_points
  };
}
