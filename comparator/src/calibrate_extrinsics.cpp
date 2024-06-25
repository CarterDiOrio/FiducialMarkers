#include <DataStreamClient.h>
#include <Eigen/Dense>
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
#include "comparator/mrcal_reprojected_camera.hpp"
#include "comparator/extrinsic_observation.hpp"
#include "comparator/chessboard.hpp"
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


namespace datastream = ViconDataStreamSDK::CPP;
namespace po = boost::program_options;

/// \brief Gathers measurements using the vicon system
/// \return The measurements
ExtrinsicObservations gather_vicon_measurements(
  const Mount & mount,
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

  // create a MrCal based camera from the realsense
  const auto camera = MrCalReprojectedCamera::from_files(
    std::make_unique<RealSenseCamera>(1920, 1080, 30),
    vm["spline_model"].as<std::string>(),
    vm["pinhole_model"].as<std::string>());


  // const std::string measurements_file = vm["measurements"].as<std::string>();
  const std::string measurements_file = "";

  const auto chessboard = create_chessboard(
    Eigen::Matrix4d::Identity(), //initial guess of the transformation
    10,
    10,
    0.015);

  const auto measurements =
    [&chessboard, &camera, &measurements_file]() {
      if (!measurements_file.empty()) {
        // TODO: Load measurements from file
        return ExtrinsicObservations{};
      } else {
        // Gather measurements
        return gather_vicon_measurements(chessboard, *camera);
      }
    }();


  return 0;
}

ExtrinsicObservations gather_vicon_measurements(
  const Mount & mount,
  const CameraIntf & camera)
{
  // connecting to the vicon server
  std::cout << "Connecting to the vicon server..." << std::endl;
  const auto client = []()
    {
      std::string server{"169.254.100.131:801"};
      auto client = vicon::connect_to_server(server);
      client | vicon::EnableMarkerData | vicon::EnableSegmentData;
      client->SetStreamMode(datastream::StreamMode::ClientPull);
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

  while (true) {
    auto c = cv::waitKey(16);

    if (c == 'q') {
      /// quit taking measurements
      break;
    } else if (c == 's') {

      // the rotation averaging operation is not well defined, so we rely on the
      // measurements of the vicon being pretty accurate already to get good
      // results.
      // the main reason we do this is because a single measurement that is noisy
      // can bias a whole set of chessboard corners in the same way leading
      // to even less correct iid assumptions even if the noise from the vicon
      // is small.
      std::vector<Sophus::SO3d> rotation_measurements;
      std::vector<Eigen::Vector3d> translation_measurements;
      while (rotation_measurements.size() < 10) {
        Sophus::SE3d T_wm =
          get_object_transform(fiducial_mount_vicon, *client).value();
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


    }

  }
  return {};
  // return measurements;
}
