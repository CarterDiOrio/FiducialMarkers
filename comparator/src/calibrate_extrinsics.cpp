#include <DataStreamClient.h>
#include <Eigen/Dense>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <iostream>
#include <fstream>
#include <memory.h>
#include "comparator/camera.hpp"
#include "comparator/tracked_object.hpp"
#include "comparator/vicon.hpp"
#include "comparator/mount.hpp"
#include "comparator/measurement.hpp"
#include "comparator/realsense_camera_impl.hpp"
#include "comparator/mrcal_reprojected_camera.hpp"
#include "comparator/extrinsic_observation.hpp"
#include "comparator/chessboard.hpp"
#include "comparator/extrinsic_calibration.hpp"
#include "comparator/reprojection_cost_function.hpp"
#include "comparator/hand_error_cost_function.hpp"
#include <memory>
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
#include <sophus/average.hpp>
#include <nlohmann/json.hpp>
#include <sophus/se3.hpp>


namespace datastream = ViconDataStreamSDK::CPP;
namespace po = boost::program_options;

/// \brief Gathers measurements using the vicon system
/// \return The measurements
ExtrinsicObservations gather_vicon_measurements(
  const CameraIntf & camera);

void measure_vicon_variance();

int main(int argc, char ** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
    "measurements",
    po::value<std::string>()->default_value("")->implicit_value(""),
    "Load measurements from a file to calculate extrinsics")(
    "spline_model",
    po::value<std::string>()->default_value(""),
    "The path to the spline model file from mrcal"
  )(
    "pinhole_model",
    po::value<std::string>()->default_value(""),
    "The path to the pinhole model file from mrcal"
  )(
    "vicon_variance",
    po::value<std::string>()->default_value("n")->implicit_value("y"),
    "Measure the variance of the vicon system"
  );

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

  if (vm["vicon_variance"].as<std::string>() == "y") {
    measure_vicon_variance();
    return 0;
  }

  const std::string measurements_file = vm["measurements"].as<std::string>();

  Eigen::Matrix4d fiducial_initial_guess = Eigen::Matrix4d::Identity();

  const auto chessboard = create_chessboard(
    fiducial_initial_guess, //initial guess of the transformation
    10,
    10,
    15);

  ExtrinsicObservations measurements =
    [&vm, &measurements_file]() {
      if (!measurements_file.empty()) {
        std::cout << "Loading measurements from file" << std::endl;
        std::ifstream file(measurements_file);
        json j;
        file >> j;
        return j.get<ExtrinsicObservations>();
      } else {

        // create a MrCal based camera from the realsense
        const auto rs_camera =
          std::make_shared<RealSenseCamera>(1920, 1080, 30);
        std::cout << "Constructed MrCal Camera..." << std::endl;
        const auto camera = MrCalReprojectedCamera::from_files(
          rs_camera,
          vm["spline_model"].as<std::string>(),
          vm["pinhole_model"].as<std::string>());
        std::cout << "Finished";

        // Gather measurements with the camera
        return gather_vicon_measurements(*camera);
      }
    }();


  std::cout << "Finished Gathering Measurements: " <<
    measurements.observations.size() << std::endl;

  // write the measurements to a file
  if (measurements_file.empty()) {
    std::ofstream file("measurements.json");
    json j = measurements;
    file << std::setw(1) << j << std::endl;
  }

  // load lean lens model
  const auto lean_lens_model = load_lens_model(
    vm["pinhole_model"].as<std::string>());

  /// get intrinsics from the lean lens model
  const cv::Mat K = get_intrinsics_from_camera_model(*lean_lens_model);

  Eigen::Matrix4d T_cmount_camera;
  T_cmount_camera << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

  Eigen::Matrix4d T_mount_fiducial;
  T_mount_fiducial << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, -8,
    0, 0, 0, 1;

  auto initial_guess = calibration::ExtrinsicCalibration::Identity();
  initial_guess.T_x_camera = Sophus::SE3d{T_cmount_camera};
  initial_guess.T_mount_fiducial = Sophus::SE3d{T_mount_fiducial};


  // calibrate the extrinsics
  auto extrinsics_data = calibration::calibrate_extrinsics(
    measurements,
    chessboard,
    K,
    initial_guess);

  const auto extrinsics = extrinsics_data.calibration;

  std::cout << "T_x_camera: \n" << extrinsics.T_x_camera.matrix() << std::endl;
  std::cout << "T_mount_fiducial: \n" << extrinsics.T_mount_fiducial.matrix() <<
    std::endl;

  // use the extrinsics to calculate the reprojection error
  std::vector<double> errors;
  std::vector<Eigen::Vector2d> reprojections;
  for (const auto & observation : measurements.observations) {
    for (size_t i = 0; i < chessboard.fiducial_corners.size(); i++) {
      const auto & corner = chessboard.fiducial_corners[i];
      const auto & img_point =
        observation.chessboard_observations.corners[i].img_point;

      Eigen::Vector4d p = observation.T_world_mount *
        extrinsics.T_mount_fiducial.matrix() * corner.homogeneous();
      Eigen::Vector3d p_camera =
        (extrinsics.T_x_camera.inverse().matrix() *
        observation.T_world_cmount.inverse() * p).head<3>();
      Eigen::Vector2d p_image{
        (p_camera.x() / p_camera.z()) * K.at<double>(0, 0) + K.at<double>(0, 2),
        (p_camera.y() / p_camera.z()) * K.at<double>(1, 1) + K.at<double>(1, 2)
      };
      errors.push_back(img_point.x() - p_image.x());
      errors.push_back(img_point.y() - p_image.y());
      reprojections.push_back(p_image);
    }
  }

  // print the mean
  double mean = 0;
  for (const auto & r : errors) {
    mean += std::fabs(r);
  }
  mean /= errors.size();
  std::cout << "Mean: " << mean << std::endl;

  std::ofstream residuals_file("residuals.txt");
  for (size_t i = 0; i < reprojections.size(); i++) {
    residuals_file << reprojections[i].x() << " " << reprojections[i].y()
                   << " " << errors[i * 2] << " " << errors[i * 2 + 1] <<
      std::endl;
  }
  residuals_file.close();


  return 0;
}

ExtrinsicObservations gather_vicon_measurements(
  const CameraIntf & camera)
{
  // connecting to the vicon server
  std::cout << "Connecting to the vicon server..." << std::endl;
  const auto & client = []()
    {
      std::string server{"169.254.100.131:801"};
      auto client = vicon::connect_to_server(server);
      client | vicon::EnableMarkerData | vicon::EnableSegmentData;
      client->SetStreamMode(datastream::StreamMode::ClientPull);
      client->GetFrame();
      return client;
    }();
  std::cout << "Connected to the vicon server" << std::endl;

  // get the camera intrinsics
  cv::Mat K = camera.get_intrinsics();

  // For now the fiducial on the mount is going to be a 10x7 chessboard for testing purposes.
  const auto fiducial_mount_vicon = vicon::TrackedObject{
    .subject_name = "FiducialMount",
    .root_segment_name = "FiducialMount"
  };

  const auto camera_mount = vicon::TrackedObject{
    .subject_name = "realsense",
    .root_segment_name = "realsense"
  };

  ExtrinsicObservations observations;
  while (true) {
    auto c = cv::waitKey(16);

    const auto img = camera.get_frame();
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    /// measure corners
    const auto maybe_observation = mrgingham_find_chessboard(
      gray, 10);

    if (maybe_observation.has_value()) {
      const auto & corners = maybe_observation->corners;
      // draw corners onto frame
      // std::cout << "Drawing Corners: " << corners.size() << std::endl;
      for (const auto & corner : corners) {
        cv::circle(
          img, cv::Point2d{corner.img_point.x(), corner.img_point.y()}, 10,
          cv::Scalar(0, 255, 0), -1);
      }
    }

    if (c == 'q') {
      /// quit taking measurements
      break;
    } else if (c == 's' && maybe_observation.has_value()) {

      // the rotation averaging operation is not well defined, so we rely on the
      // measurements of the vicon being pretty accurate already to get good
      // results.
      // the main reason we do this is because a single measurement that is noisy
      // can bias a whole set of chessboard corners in the same way leading
      // to even less correct iid assumptions even if the noise from the vicon
      // is small.
      std::vector<Sophus::SE3d> mount_measurements;
      std::vector<Sophus::SE3d> camera_measurements;
      while (mount_measurements.size() < 1500 ||
        camera_measurements.size() < 1500)
      {
        if (mount_measurements.size() % 100 == 0) {
          std::cout << "Iteration: " << mount_measurements.size() << std::endl;
        }

        auto success = client->GetFrame();       //Pull vicon data
        std::optional<Sophus::SE3d> maybe_T_wm =
          get_object_transform(fiducial_mount_vicon, *client);

        if (!maybe_T_wm.has_value()) {
          std::cout << "No measurement available" << std::endl;
          continue;
        }
        Sophus::SE3d T_wm = *maybe_T_wm;
        mount_measurements.push_back(T_wm);

        std::optional<Sophus::SE3d> maybe_T_wc =
          get_object_transform(camera_mount, *client);

        if (!maybe_T_wc.has_value()) {
          std::cout << "No measurement available" << std::endl;
          continue;
        }

        Sophus::SE3d T_wc = *maybe_T_wc;
        camera_measurements.push_back(T_wc);
      }

      Sophus::SE3d T_wm_avg = *Sophus::average(mount_measurements);
      Sophus::SE3d T_wc_avg = *Sophus::average(camera_measurements);

      std::cout << "T_wm_translation: " << T_wm_avg.translation().transpose() <<
        " T_wm_rotation: " << T_wm_avg.so3().angleX() << " " <<
        T_wm_avg.so3().angleY() << " " << T_wm_avg.so3().angleZ() << std::endl;

      std::cout << "T_wc_translation: " << T_wc_avg.translation().transpose() <<
        " T_wc_rotation: " << T_wc_avg.so3().angleX() << " " <<
        T_wc_avg.so3().angleY() << " " << T_wc_avg.so3().angleZ() << std::endl;

      std::cout << std::endl;


      observations.observations.emplace_back(
        T_wm_avg.matrix(),
        T_wc_avg.matrix(),
        *maybe_observation
      );
    }
    cv::imshow("frame ", img);
    // cv::imshow("uncorrected ", uimg);

  }

  return observations;
}


void measure_vicon_variance()
{
  std::cout << "Connecting to the vicon server..." << std::endl;
  const auto & client = []()
    {
      std::string server{"169.254.100.131:801"};
      auto client = vicon::connect_to_server(server);
      client | vicon::EnableMarkerData | vicon::EnableSegmentData;
      client->SetStreamMode(datastream::StreamMode::ClientPull);
      client->GetFrame();
      return client;
    }();
  std::cout << "Connected to the vicon server" << std::endl;

  const auto fiducial_mount_vicon = vicon::TrackedObject{
    .subject_name = "FiducialMount",
    .root_segment_name = "FiducialMount"
  };

  const auto camera_mount = vicon::TrackedObject{
    .subject_name = "realsense",
    .root_segment_name = "realsense"
  };

  std::vector<Sophus::SE3d> mount_measurements;
  std::vector<Sophus::SE3d> camera_measurements;


  for (size_t i = 0; i < 10000; i++) {
    if (i % 100 == 0) {
      std::cout << "Iteration: " << i << std::endl;
    }

    auto success = client->GetFrame();       //Pull vicon data

    std::optional<Sophus::SE3d> maybe_T_wm =
      get_object_transform(fiducial_mount_vicon, *client);

    if (!maybe_T_wm.has_value()) {
      std::cout << "No measurement available" << std::endl;
      continue;
    }

    Sophus::SE3d T_wm = *maybe_T_wm;
    mount_measurements.push_back(T_wm);

    std::optional<Sophus::SE3d> maybe_T_wc =
      get_object_transform(camera_mount, *client);

    if (!maybe_T_wc.has_value()) {
      std::cout << "No measurement available" << std::endl;
      continue;
    }

    Sophus::SE3d T_wc = *maybe_T_wc;
    camera_measurements.push_back(T_wc);
  }

  // write mount measurements to a file
  std::ofstream mount_file("mount_measurements.txt");
  for (const auto & m : mount_measurements) {
    mount_file << m.translation().transpose() << " " << m.so3().angleX() << " "
               << m.so3().angleY() << " " << m.so3().angleZ() << std::endl;
  }

  // write camera measurements to a file
  std::ofstream camera_file("camera_measurements.txt");
  for (const auto & m : camera_measurements) {
    camera_file << m.translation().transpose() << " " << m.so3().angleX() << " "
                << m.so3().angleY() << " " << m.so3().angleZ() << std::endl;
  }
}
