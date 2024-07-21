#ifndef INC_GUARD_APRIL_ALGO_HPP
#define INC_GUARD_APRIL_ALGO_HPP

#include "comparator/measurement_algo_interface.hpp"
#include "comparator/camera.hpp"
#include <common/image_types.h>
#include <common/zarray.h>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <pi_eink/api.hpp>
#include <pi_eink/client.hpp>
#include <string>
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>

class AprilAlgo : public MeasurementAlgoInterface
{
public:

  AprilAlgo(
    std::shared_ptr<CameraIntf> camera,
    std::shared_ptr<pi_eink::EinkClient> eink_client,
    std::string tag_standard,
    double marker_size,
    int id
  ): camera{camera},
     eink_client{eink_client},
     marker_size{marker_size},
     tag_standard{tag_standard},
     id{id}
  {}

  virtual std::string get_name() override {
    std::string name = "april_" + tag_standard + "_" + std::to_string(id) + "_" + std::to_string(marker_size);
    return name;
  }

  virtual void display() override {
    // display the aruco fiducial
    double cell_size = 0.0;
    size_t num_cells = 0;
    std::string family = "";
    if (tag_standard == "tagStandard36h11") {
      num_cells = 8;
      cell_size = marker_size / 8;
      family = "tag36h11";
    }
    else if (tag_standard == "tagStandard25h9") {
      num_cells = 7;
      cell_size = marker_size / 7;
      family = "tag25h9";
    }
    else if (tag_standard == "tagStandard16h5") {
      num_cells = 6;
      cell_size = marker_size / 6;
      family = "tag16h5";
    }
    else {
      throw std::runtime_error("Invalid tag standard");
    }

    const auto response = eink_client->draw(pi_eink::AprilTagRequest{
      .family = family,
      .id = id,
      .desired_square_size = cell_size
    });

    actual_marker_size = response.response->actual_square_size * num_cells;
  }

  virtual std::optional<Sophus::SE3d> detect() override {
    auto img = camera->get_frame();

    // get the current intrinsics from the camera
    const auto K = camera->get_intrinsics();

    // change image to grayscale
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_family_t* tf;
    
    if (tag_standard == "tagStandard36h11") {
      tf = tag36h11_create();
    }
    else if (tag_standard == "tagStandard25h9") {
      tf = tag25h9_create();
    }
    else if (tag_standard == "tagStandard16h5") {
      tf = tag16h5_create();
    }
    else {
      throw std::runtime_error("Invalid tag standard");
    }
    apriltag_detector_add_family(td, tf);

    image_u8_t april_tag_img {
      .width = img.cols,
      .height = img.rows,
      .stride = img.cols,
      .buf = img.data
    };
    zarray_t * detections = apriltag_detector_detect(td, &april_tag_img);

    if (zarray_size(detections) == 0) {
      return std::nullopt;
    }

    apriltag_detection_t *det;
    zarray_get(detections, 0, &det);

    apriltag_pose_t pose;
    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = actual_marker_size / 1000.0;
    info.fx = K.at<double>(0, 0);
    info.fy = K.at<double>(1, 1);
    info.cx = K.at<double>(0, 2);
    info.cy = K.at<double>(1, 2);

    double err = estimate_tag_pose(&info, &pose);
    
    // convert pose to SE3
    Eigen::Matrix3d R = Eigen::Map<Eigen::Matrix3d>(pose.R->data);
    Eigen::Vector3d T = Eigen::Map<Eigen::Vector3d>(pose.t->data);

    Sophus::SE3d T_camera_marker(
      R, T
    );

    apriltag_detector_destroy(td);
    apriltag_detections_destroy(detections);
    
    if (tag_standard == "tagStandard36h11") {
      tag36h11_destroy(tf);
    }
    else if (tag_standard == "tagStandard25h9") {
      tag25h9_destroy(tf);
    }
    else if (tag_standard == "tagStandard16h5") {
      tag16h5_destroy(tf);
    }

    return T_camera_marker;
  }

private:
  std::shared_ptr<CameraIntf> camera;
  std::shared_ptr<pi_eink::EinkClient> eink_client;
  double marker_size;
  double actual_marker_size;
  std::string tag_standard;
  int id;


};

#endif