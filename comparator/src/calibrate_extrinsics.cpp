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
#include <mrcal/mrcal.h>

namespace datastream = ViconDataStreamSDK::CPP;
namespace po = boost::program_options;

std::unique_ptr<datastream::Client> init_vicon()
{
  std::string server{"169.254.100.131:801"};
  auto client = vicon::connect_to_server(server);
  client | vicon::EnableMarkerData | vicon::EnableSegmentData;
  client->SetStreamMode(datastream::StreamMode::ClientPull);
  return client;
}

Mount init_mount(const std::string & file_path)
{
  auto mount = load_mount(file_path);
  std::cout << "Loaded Mount: " << std::endl;
  std::cout << "  " << "Mount Id: " << mount.fiducial_id << std::endl;
  std::cout << "  " << "Mount Parameters: " << mount.fiducial_parameters <<
    std::endl;
  std::cout << "  " << "Mount Num Corners: " << mount.fiducial_corners.size() <<
    std::endl;
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
    po::value<std::string>()->default_value("../mount.txt")->implicit_value(
      "../mount.txt"),
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
    std::cout << "Loaded " << measurements.size() << " measurements" <<
      std::endl;
  } else {
    // Gather measurements
    measurements = gather_vicon_measurements(mount);
  }

  // load the realsense
  RealSenseCamera camera(1920, 1080, 30);
  const auto K = camera.get_intrinsics();

  return 0;
}

std::vector<Measurement> gather_vicon_measurements(const Mount & mount)
{
  // connecting to the vicon server
  std::cout << "Connecting to the vicon server..." << std::endl;
  auto client = init_vicon();


  std::cout << "Connected to the vicon server" << std::endl;
  // auto & client = maybe_client.value();

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

    // auto maybe_measurement =
    //   measure_calibration_pattern(*client, camera, fiducial_mount_vicon, mount);

    auto c = cv::waitKey(16);

    if (c == 'q') {
      break;
    } else if (c == 's') {
      // if (maybe_measurement.has_value()) {
      //   measurements.push_back(maybe_measurement.value());
      //   std::cout << "Measurement added: " << measurements.size() << std::endl;
      // }
    }

  }

  return measurements;
}
