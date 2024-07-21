#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pi_eink/client.hpp>
#include "comparator/measurement.hpp"
#include "comparator/scaled_camera.hpp"
#include "comparator/vicon.hpp"
#include "comparator/extrinsic_calibration.hpp"
#include "comparator/measurement_runner.hpp"
#include "comparator/chessboard_algo.hpp"
#include "comparator/aruco_algo.hpp"
#include "comparator/april_algo.hpp"
#include "comparator/realsense_camera_impl.hpp"
#include "comparator/mrcal_reprojected_camera.hpp"

namespace po = boost::program_options;

int main(int argc, char** argv) {

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
    "spline_model",
    po::value<std::string>()->default_value(""),
    "The path to the spline model file from mrcal"
  )(
    "pinhole_model",
    po::value<std::string>()->default_value(""),
    "The path to the pinhole model file from mrcal"
  )(
    "extrinsics",
    po::value<std::string>()->default_value(""),
    "The path to the extrinsic calibration"
  )(
    "eink_address",
    po::value<std::string>()->default_value(""),
    "The address and port of the eink display"
  )(
    "vicon_address",
    po::value<std::string>()->default_value(""),
    "The address and port of the vicon system"
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

  const auto eink_address = vm["eink_address"].as<std::string>();
  const auto spline_model = vm["spline_model"].as<std::string>();
  const auto pinhole_model = vm["pinhole_model"].as<std::string>();
  const auto vicon_address = vm["vicon_address"].as<std::string>();
  const auto extrinsics = vm["extrinsics"].as<std::string>();

  // loading extrinsics
  std::ifstream extrinsics_file(extrinsics);
  if (!extrinsics_file.is_open()) {
    std::cerr << "Could not open extrinsics file: " << extrinsics << std::endl;
    return 1;
  }
  const auto extrinsics_json = nlohmann::json::parse(extrinsics_file);
  const auto extrinsic_calibration = extrinsics_json.get<calibration::ExtrinsicCalibration>();

  // open realsense
  auto rscamera = std::make_shared<RealSenseCamera>(1280, 720, 30);
  auto mrcamera = MrCalReprojectedCamera::from_files(
    rscamera, 
    spline_model, 
    pinhole_model);
  
  const auto camera = std::make_shared<ScaledCamera>(mrcamera, 1.0);

  const auto eink_client = std::make_shared<pi_eink::EinkClient>(
    eink_address
  );

  // auto vicon_client = []()
  //   {
  //     std::cout << "Connecting to vicon server... ";
  //     std::string server{"169.254.100.131:801"};
  //     auto client = vicon::connect_to_server(server);
  //     client | vicon::EnableMarkerData | vicon::EnableSegmentData;
  //     client->SetStreamMode(datastream::StreamMode::ClientPull);
  //     client->GetFrame();
  //     std::cout << "  Connected!" << std::endl;
  //     return client;
  //   }();

  MeasurementRunner runner {
    100, // measurements per algo
    0, // vicon measurements per location
    std::nullopt,
    eink_client,
    vicon::TrackedObject{
      .subject_name = "realsense",
      .root_segment_name = "realsense"
    },
    vicon::TrackedObject{
      .subject_name = "FiducialMount",
      .root_segment_name = "FiducialMount"
    }
  };

  // double max_size = 11 * 14;
  // // testing various numbers of corners
  // for(size_t i = 4; i < 50; i+=2) {

  //   std::cout << "Registering chessboard algo with " << i << " corners" << max_size/ i << std::endl;
  //   runner.register_measurement<ChessboardAlgo>(
  //     camera,
  //     eink_client,
  //     camera->get_intrinsics(),
  //     i,
  //     max_size / (i + 3)
  //   );
  // }

  runner
  .register_measurement<AprilAlgo>(
    camera,
    eink_client,
    "tagStandard16h5",
    100,
    0
  );

  for (double i = 1.0; i > 0.1; i -= 0.02) {
    camera->set_scale(i);
    
    // scale frame by half to fit on screen
    std::cout << "Running scale " << i << " "  << camera->get_resolution() << std::endl;

    for (size_t j = 0; j < 30; j++) {
      const auto frame = camera->get_frame();
      cv::imshow("frame", frame);
      cv::waitKey(16);
    }


    const std::string set_prefix = "scale_" + std::to_string(i);
    runner.run(set_prefix);
  }

  cv::destroyAllWindows();
  
  ofstream out("tag_measurements.json");
  MeasurementFile mf;
  mf.measurement_sets = runner.get_measurement_sets();
  mf.extrinsics = extrinsic_calibration;
  nlohmann::json j = mf;

  out << std::setw(4) << j << std::endl;

  std::cout << "Wrote data to tag_measurements.json" << std::endl;
  out.close();
  
  return 0;
}