#ifndef INC_GUARD_ARUCO_ALGO_HPP
#define INC_GUARD_ARUCO_ALGO_HPP

#include "comparator/measurement_algo_interface.hpp"
#include "comparator/camera.hpp"
#include <aruco/cameraparameters.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <pi_eink/api.hpp>
#include <pi_eink/client.hpp>
#include <string>
#include <aruco/dictionary.h>
#include <aruco/posetracker.h>
#include <aruco/markerdetector.h>
#include <aruco/marker.h>

class ArucoAlgo : public MeasurementAlgoInterface
{
public:

  ArucoAlgo(
    std::shared_ptr<CameraIntf> camera,
    std::shared_ptr<pi_eink::EinkClient> eink_client,
    aruco::Dictionary::DICT_TYPES type,
    double marker_size,
    int id
  ): camera{camera},
     eink_client{eink_client},
     marker_size{marker_size},
     type{type},
     id{id}
  {
    dictionary = aruco::Dictionary::loadPredefined(type);
    detector.setDictionary(type);
  };

  virtual std::string get_name() override {
    return "aruco_" + dictionary_type + "_" + std::to_string(id) + "_" + std::to_string(actual_marker_size);
  }

  virtual void display() override {
    // display the aruco fiducial
    double cell_size = 0.0;
    size_t num_cells = 0;
    dictionary_type = "";
    switch (type) {
      case aruco::Dictionary::DICT_TYPES::ARUCO_MIP_36h12:
        cell_size = marker_size / 8.0;
        num_cells = 8;
        dictionary_type = "aruco_mip_36h12";
        break;
      case aruco::Dictionary::DICT_TYPES::ARUCO_MIP_25h7:
        cell_size = marker_size / 7.0;
        num_cells = 7;
        dictionary_type = "aruco_mip_25h7";
        break;
      default:
        throw std::runtime_error("Unsupported dictionary type");        
    }

    const auto response = eink_client->draw(pi_eink::ArucoTagRequest{
      .dictionary_type = dictionary_type,
      .id = id,
      .desired_square_size = cell_size
    });

    actual_marker_size = response.response->actual_square_size * num_cells;
  }

  virtual std::optional<Sophus::SE3d> detect() override {
    // get the current intrinsics from the camera
    const auto K = camera->get_intrinsics();

    auto img = camera->get_frame();
    cv::Mat small;
    cv::resize(img, small, cv::Size(img.cols / 2, img.rows / 2));

    aruco::CameraParameters cam_params(
      K, cv::Mat::zeros(cv::Size(1, 4), CV_32F), cv::Size(img.cols, img.rows)
    );

    auto markers = detector.detect(img, cam_params,
      actual_marker_size / 1000.0
    );

    if (markers.size() == 0) {
      return std::nullopt;
    }

    auto& marker = markers.at(0);
    marker.calculateExtrinsics(actual_marker_size / 1000.0, cam_params, false);

    // convert rvec and tvec to SE3
    cv::Mat R;
    cv::Rodrigues(marker.Rvec, R);
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R, R_eigen);

    Eigen::Vector3d translation;
    cv::cv2eigen(marker.Tvec, translation);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_eigen;
    T.block<3, 1>(0, 3) = translation;

    Sophus::SE3d T_camera_marker{Sophus::SE3d::fitToSE3(T)};

    return T_camera_marker;
  }

private:
  std::shared_ptr<CameraIntf> camera;
  std::shared_ptr<pi_eink::EinkClient> eink_client;
  double marker_size;
  double actual_marker_size;
  aruco::Dictionary::DICT_TYPES type;
  int id;
  aruco::Dictionary dictionary;
  aruco::MarkerDetector detector;
  aruco::MarkerPoseTracker tracker;
  std::string dictionary_type;


};

#endif