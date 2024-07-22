#include <DataStreamClient.h>
#include <Eigen/Dense>
#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <boost/any.hpp>
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
// #include <pi_eink/api.hpp>
#include <pi_eink/client.hpp>


namespace datastream = ViconDataStreamSDK::CPP;
namespace po = boost::program_options;

/// \brief Gathers measurements using the vicon system
/// \return The measurements
ExtrinsicObservations gather_vicon_measurements(
  const CameraIntf & camera);

void measure_vicon_variance();

ExtrinsicObservations load_measurements_from_files(
  const std::vector<std::string> & measurements_files) {
  ExtrinsicObservations total;
  for (const auto & measurements_file : measurements_files) {
    std::cout << "Loading: " << measurements_file << std::endl;
    std::ifstream file(measurements_file);
    json j;
    file >> j;
    auto observations = j.get<ExtrinsicObservations>();
    total.observations.insert(
      total.observations.end(),
      observations.observations.begin(),
      observations.observations.end());
  }
  return total;
}

int main(int argc, char ** argv)
{
  pi_eink::EinkClient client{"169.254.100.135:8080"};

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
    "camera_stationary",
    po::value<std::vector<std::string>>()->multitoken(),
    "Load measurements from a file to calculate extrinsics")(
    "mount_stationary",
    po::value<std::vector<std::string>>()->multitoken(),
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
  )(
    "output",
    po::value<std::string>()->default_value("measurements.json"),
    "The output file for the measurements"
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

  std::vector<std::string> camera_stationary_files;
  try {
    camera_stationary_files =
      vm["camera_stationary"].as<std::vector<std::string>>();
  } catch (boost::bad_any_cast _) {
  }

  std::vector<std::string> mount_stationary_files;
  try {
    mount_stationary_files =
      vm["mount_stationary"].as<std::vector<std::string>>();
  } catch (boost::bad_any_cast _) {
  }


  Eigen::Matrix4d fiducial_initial_guess = Eigen::Matrix4d::Identity();

  const auto chessboard = create_chessboard(
    fiducial_initial_guess, //initial guess of the transformation
    26,
    26,
    5.152);

  if (!camera_stationary_files.empty() || !mount_stationary_files.empty()) {
    std::cout << "Loading Measurements from files...\n";

    std::optional<ExtrinsicObservations> camera_stationary_observations;
    if (!camera_stationary_files.empty()) {
      camera_stationary_observations =
        load_measurements_from_files(camera_stationary_files);
    }

    std::optional<ExtrinsicObservations> mount_stationary_observations;
    if (!mount_stationary_files.empty()) {
      mount_stationary_observations =
        load_measurements_from_files(mount_stationary_files);
    }

    std::cout << "Loaded. Starting Optimization...\n";

    // load lean lens model
    const auto lean_lens_model = load_lens_model(
      vm["pinhole_model"].as<std::string>());

    /// get intrinsics from the lean lens model
    const cv::Mat K = get_intrinsics_from_camera_model(*lean_lens_model);

    Eigen::Matrix4d T_cmount_camera;
    T_cmount_camera << 1, 0, 0, 0,
      0, 0, 1, 0,
      0, -1, 0, 0,
      0, 0, 0, 1;

    Eigen::Matrix4d T_mount_fiducial;
    T_mount_fiducial << 1, 0, 0, 55,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

    auto initial_guess = calibration::ExtrinsicCalibration::Identity();
    initial_guess.T_hand_eye = Sophus::SE3d{T_cmount_camera};
    initial_guess.T_mount_fiducial = Sophus::SE3d{T_mount_fiducial};


    calibration::OptimizationInputs inputs {
      .camera_stationary_observations = camera_stationary_observations,
      .mount_stationary_observations = mount_stationary_observations,
      .initial_guess = initial_guess
    };

    // calibrate the extrinsics
    auto extrinsics_data = calibration::calibrate_extrinsics(
      inputs,
      chessboard,
      K);

    const auto extrinsics = extrinsics_data.calibration;

    std::cout << "T_x_camera: \n" << extrinsics.T_hand_eye.matrix() << std::endl;
    std::cout << "T_mount_fiducial: \n" << extrinsics.T_mount_fiducial.matrix() <<
      std::endl;

    // use the extrinsics to calculate the reprojection error
    std::vector<double> errors;
    std::vector<Eigen::Vector2d> reprojections;

    if (camera_stationary_observations.has_value()) {
      for (const auto & observation : camera_stationary_observations.value().observations) {
        for (size_t i = 0; i < chessboard.fiducial_corners.size(); i++) {
          const auto & corner = chessboard.fiducial_corners[i];
          const auto & img_point =
            observation.chessboard_observations.corners[i].img_point;

          Eigen::Vector4d p = observation.T_world_mount *
            extrinsics.T_mount_fiducial.matrix() * corner.homogeneous();
          Eigen::Vector3d p_camera =
            (extrinsics.T_hand_eye.inverse().matrix() *
            observation.T_world_hand.inverse() * p).head<3>();
          Eigen::Vector2d p_image{
            (p_camera.x() / p_camera.z()) * K.at<double>(0, 0) + K.at<double>(0, 2),
            (p_camera.y() / p_camera.z()) * K.at<double>(1, 1) + K.at<double>(1, 2)
          };
          errors.push_back(img_point.x() - p_image.x());
          errors.push_back(img_point.y() - p_image.y());
          reprojections.push_back(p_image);
        }
      }
    }

    if (mount_stationary_observations.has_value()) {
      for (const auto & observation : mount_stationary_observations.value().observations) {
        for (size_t i = 0; i < chessboard.fiducial_corners.size(); i++) {
          const auto & corner = chessboard.fiducial_corners[i];
          const auto & img_point =
            observation.chessboard_observations.corners[i].img_point;

          Eigen::Vector4d p = observation.T_world_mount *
            extrinsics.T_mount_fiducial.matrix() * corner.homogeneous();
          Eigen::Vector3d p_camera =
            (extrinsics.T_hand_eye.inverse().matrix() *
            observation.T_world_hand.inverse() * p).head<3>();
          Eigen::Vector2d p_image{
            (p_camera.x() / p_camera.z()) * K.at<double>(0, 0) + K.at<double>(0, 2),
            (p_camera.y() / p_camera.z()) * K.at<double>(1, 1) + K.at<double>(1, 2)
          };
          errors.push_back(img_point.x() - p_image.x());
          errors.push_back(img_point.y() - p_image.y());
          reprojections.push_back(p_image);
        }
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

    // write the extrinsics to a file
    using namespace calibration;
    std::ofstream extrinsics_file("extrinsics.json");
    json j = extrinsics;
    extrinsics_file << std::setw(1) << j << std::endl;
    extrinsics_file.close();
  }
  else {
    std::cout << "Starting Data Gathering...\n";
    // create a MrCal based camera from the realsense
    const auto rs_camera =
      std::make_shared<RealSenseCamera>(1920, 1080, 30);
    std::cout << "Constructed MrCal Camera..." << std::endl;
    const auto camera = MrCalReprojectedCamera::from_files(
      rs_camera,
      vm["spline_model"].as<std::string>(),
      vm["pinhole_model"].as<std::string>());

    // Gather measurements with the camera
    const auto measurements = gather_vicon_measurements(*camera);
    std::cout << "Finished Gathering Measurements: " <<
      measurements.observations.size() << std::endl;
    
    std::cout << "Writing to file..." << std::endl;

    // write the measurements to a file
    std::ofstream file(vm["output"].as<std::string>());
    json j = measurements;
    file << std::setw(1) << j << std::endl;

    std::cout << "Done!" << std::endl;
  }

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
      gray, 26);

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

      while (mount_measurements.size() < 100 ||
        camera_measurements.size() < 100)
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

      std::vector<Sophus::SO3d> mount_rotations;
      std::vector<Sophus::SO3d> camera_rotations;

      for (const auto & m : mount_measurements) {
        mount_rotations.push_back(m.so3());
      }

      for (const auto & m : camera_measurements) {
        camera_rotations.push_back(m.so3());
      }

      Sophus::SO3d mount_avg = *Sophus::average(mount_rotations);
      Sophus::SO3d camera_avg = *Sophus::average(camera_rotations);

      std::vector<Eigen::Vector3d> mount_translations;
      std::vector<Eigen::Vector3d> camera_translations;

      for (const auto & m : mount_measurements) {
        mount_translations.push_back(m.translation());
      }

      for (const auto & m : camera_measurements) {
        camera_translations.push_back(m.translation());
      }

      Eigen::Vector3d mount_avg_translation = Eigen::Vector3d::Zero();
      for (const auto & t : mount_translations) {
        mount_avg_translation += t;
      }
      mount_avg_translation /= mount_translations.size();

      Eigen::Vector3d camera_avg_translation = Eigen::Vector3d::Zero();
      for (const auto & t : camera_translations) {
        camera_avg_translation += t;
      }
      camera_avg_translation /= camera_translations.size();


      // create SE3d from the averages
      Eigen::Matrix4d T_wm_avg = Eigen::Matrix4d::Identity();
      T_wm_avg.block<3, 3>(0, 0) = mount_avg.matrix();
      T_wm_avg.block<3, 1>(0, 3) = mount_avg_translation;

      Eigen::Matrix4d T_wc_avg = Eigen::Matrix4d::Identity();
      T_wc_avg.block<3, 3>(0, 0) = camera_avg.matrix();
      T_wc_avg.block<3, 1>(0, 3) = camera_avg_translation;

      Sophus::SE3d T_wm{T_wm_avg};
      Sophus::SE3d T_wc{T_wc_avg};

      std::cout << "T_wm_translation: " << T_wm.translation().transpose() <<
        " T_wm_rotation: " << T_wm.so3().angleX() << " " <<
        T_wm.so3().angleY() << " " << T_wm.so3().angleZ() << std::endl;

      std::cout << "T_wc_translation: " << T_wc.translation().transpose() <<
        " T_wc_rotation: " << T_wc.so3().angleX() << " " <<
        T_wc.so3().angleY() << " " << T_wc.so3().angleZ() << std::endl;

      std::cout << std::endl;


      observations.observations.emplace_back(
        T_wm.matrix(),
        T_wc.matrix(),
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

  for (size_t i = 0; i < 50000; i++) {
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
  for (size_t idx = 0; idx < camera_measurements.size(); idx++) {
    const auto & m = camera_measurements[idx];
    camera_file << m.translation().transpose() << " " << m.so3().angleX() << " "
                << m.so3().angleY() << " " << m.so3().angleZ() << std::endl;
  }
}
