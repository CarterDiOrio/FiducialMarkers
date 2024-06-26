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

  const std::string measurements_file = vm["measurements"].as<std::string>();

  Eigen::Matrix4d initial_guess;
  initial_guess << 1, 0, 0, 0,
    0, 1, 0, -5,
    0, 0, 1, -14,
    0, 0, 0, 1;

  const auto chessboard = create_chessboard(
    Eigen::Matrix4d::Identity(), //initial guess of the transformation
    10,
    10,
    15);

  const ExtrinsicObservations measurements =
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
    json j{measurements};
    file << std::setw(1) << j << std::endl;
  }

  // load lean lens model
  const auto lean_lens_model = load_lens_model(
    vm["pinhole_model"].as<std::string>());

  /// get intrinsics from the lean lens model
  const cv::Mat K = get_intrinsics_from_camera_model(*lean_lens_model);

  // calibrate the extrinsics
  const auto extrinsics = calibration::calibrate_extrinsics(
    measurements,
    chessboard,
    K);

  std::cout << "T_x_camera: \n" << extrinsics.T_x_camera.matrix() << std::endl;
  std::cout << "T_mount_fiducial: \n" << extrinsics.T_mount_fiducial.matrix() <<
    std::endl;

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

  ExtrinsicObservations observations;
  while (true) {
    auto c = cv::waitKey(16);

    const auto img = camera.get_frame();
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    /// measure corners
    const auto maybe_corners = mrgingham_find_chessboard(
      gray, 10);

    if (maybe_corners.has_value()) {
      const auto & corners = *maybe_corners;
      // draw corners onto frame
      // std::cout << "Drawing Corners: " << corners.size() << std::endl;
      for (const auto & corner : corners) {
        cv::circle(
          img, cv::Point2d{corner.x(), corner.y()}, 10,
          cv::Scalar(0, 255, 0), -1);
      }
    }

    if (c == 'q') {
      /// quit taking measurements
      break;
    } else if (c == 's' && maybe_corners.has_value()) {
      const auto & corners = *maybe_corners;


      // the rotation averaging operation is not well defined, so we rely on the
      // measurements of the vicon being pretty accurate already to get good
      // results.
      // the main reason we do this is because a single measurement that is noisy
      // can bias a whole set of chessboard corners in the same way leading
      // to even less correct iid assumptions even if the noise from the vicon
      // is small.
      std::vector<Sophus::SO3d> rotation_measurements;
      std::vector<Eigen::Vector3d> translation_measurements;
      while (rotation_measurements.size() < 100) {
        auto success = client->GetFrame();       //Pull vicon data
        std::optional<Sophus::SE3d> maybe_T_wm =
          get_object_transform(fiducial_mount_vicon, *client);

        if (!maybe_T_wm.has_value()) {
          std::cout << "No measurement available" << std::endl;
          continue;
        }
        Sophus::SE3d T_wm = *maybe_T_wm;

        rotation_measurements.push_back(T_wm.so3());
        translation_measurements.push_back(T_wm.translation());
      }

      auto rotation_average = *Sophus::average(rotation_measurements);
      Eigen::Vector3d translation_average = Eigen::Vector3d::Zero();
      for (const auto & t : translation_measurements) {
        translation_average += t;
      }

      translation_average /= translation_measurements.size();
      Sophus::SE3d T_wm_avg{rotation_average, translation_average};

      std::cout << "T_wm_avg: " << T_wm_avg.matrix() << std::endl;

      observations.observations.emplace_back(
        T_wm_avg.matrix(),
        corners
      );
    }
    cv::imshow("frame", img);
    // cv::imshow("uncorrected", uimg);

  }

  return observations;
}
