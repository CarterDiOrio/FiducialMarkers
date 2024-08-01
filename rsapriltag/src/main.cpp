#include "rsapriltag/tagCustom20h10.h"
#include <algorithm>
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <common/image_types.h>
#include <common/zarray.h>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <nlohmann/detail/macro_scope.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rsapriltag/ncam_pnp.hpp>
#include <sophus/average.hpp>
#include <sophus/se3.hpp>
#include <span>
#include <string>

using ProjectionMatrix = Eigen::Matrix<double, 3, 4>;
using Vec3 = std::tuple<double, double, double>;

static constexpr double tag_size = (17.5 * 7) / 1000.0;

struct Observation {
  std::vector<Vec3> translation;
  std::vector<Vec3> rotation;

  void add_transform(const Sophus::SE3d &T) {
    rotation.push_back({T.angleX(), T.angleY(), T.angleZ()});
    translation.push_back(
        {T.translation().x(), T.translation().y(), T.translation().z()});
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Observation, translation, rotation);

struct Data {
  std::map<std::string, Observation> observations;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Data, observations);

ProjectionMatrix intrinsics_from_rs(const rs2_intrinsics &intrinsics) {
  ProjectionMatrix K = ProjectionMatrix::Zero();
  K(0, 0) = static_cast<double>(intrinsics.fx);
  K(1, 1) = static_cast<double>(intrinsics.fy);
  K(0, 2) = static_cast<double>(intrinsics.ppx);
  K(1, 2) = static_cast<double>(intrinsics.ppy);
  K(2, 2) = 1.0;
  return K;
};

Sophus::SE3d extrinsics_from_rs(const rs2_extrinsics &extrinsics) {
  const Eigen::Matrix3d R =
      Eigen::Map<Eigen::Matrix<float, 3, 3> const>(extrinsics.rotation)
          .cast<double>();
  const Eigen::Vector3d t =
      Eigen::Map<Eigen::Vector3f const>(extrinsics.translation).cast<double>();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R.cast<double>();
  T.block<3, 1>(0, 3) = t.cast<double>();
  return Sophus::SE3d::fitToSE3(T);
};

apriltag_detection_t *get_best_detection(zarray_t *detections) {
  float decision_margin = 0.0;
  apriltag_detection_t *best_detection = nullptr;
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    if (det->decision_margin > decision_margin) {
      decision_margin = det->decision_margin;
      best_detection = det;
    }
  }
  return best_detection;
}

Sophus::SE3d estimate_tag_position(double fx, double fy, double cx, double cy,
                                   double tag_size, apriltag_detection_t *det) {
  apriltag_detection_info_t info;
  info.tagsize = tag_size;
  info.fx = fx;
  info.fy = fy;
  info.cx = cx;
  info.cy = cy;
  info.det = det;

  apriltag_pose_t pose;
  estimate_tag_pose(&info, &pose);

  Sophus::SO3d tagR{Eigen::Map<Eigen::Matrix3d>(pose.R->data)};
  Eigen::Vector3d tagT = Eigen::Map<Eigen::Vector3d>(pose.t->data);

  return Sophus::SE3d{tagR, tagT};
}

void draw_tag(cv::Mat &image, apriltag_detection_t *det) {
  cv::Point2f corners[4];
  for (int i = 0; i < 4; i++) {
    corners[i] = cv::Point2f(det->p[i][0], det->p[i][1]);
  }

  cv::line(image, corners[0], corners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(image, corners[1], corners[2], cv::Scalar(0, 255, 0), 4);
  cv::line(image, corners[2], corners[3], cv::Scalar(0, 255, 0), 4);
  cv::line(image, corners[3], corners[0], cv::Scalar(0, 255, 0), 4);
  cv::circle(image, cv::Point2d{det->c[0], det->c[1]}, 5, cv::Scalar(0, 0, 255),
             -1);

  // label the corners 1-4
  for (int i = 0; i < 4; i++) {
    cv::putText(image, std::to_string(i + 1), corners[i],
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
  }
}

double distance_to_vector(Eigen::Vector2d v1, Eigen::Vector2d v2) {
  Eigen::Vector2d a = (v1.dot(v2)) * v2;
  Eigen::Vector2d b = v1 - a;
  return b.norm();
};

Camera camera_from_detection(const ProjectionMatrix &K,
                             const Sophus::SE3d &T_world_camera,
                             apriltag_detection_t *det, double tag_size,
                             double ppd) {
  std::vector<PointPair> points;
  points.resize(4, PointPair{});

  points.at(0).object = Eigen::Vector3d{-tag_size / 2.0, -tag_size / 2.0, 0.0};
  points.at(1).object = Eigen::Vector3d{tag_size / 2.0, -tag_size / 2.0, 0.0};
  points.at(2).object = Eigen::Vector3d{tag_size / 2.0, tag_size / 2.0, 0.0};
  points.at(3).object = Eigen::Vector3d{-tag_size / 2.0, tag_size / 2.0, 0.0};

  points.at(0).image = {det->p[0][0], det->p[0][1]};
  points.at(1).image = {det->p[1][0], det->p[1][1]};
  points.at(2).image = {det->p[2][0], det->p[2][1]};
  points.at(3).image = {det->p[3][0], det->p[3][1]};

  return Camera{
      .K = K, .T_world_camera = T_world_camera, .points = points, .ppd = ppd};
}

struct Corner {
  cv::Point2d p;
  double xdist;
  double ydist;
};

void extract_chessboard(std::string name, const cv::Mat &img,
                        apriltag_detection_t *det,
                        const Sophus::SE3d &T_camera_tag) {

  // lets start with a naive method of cropping to the chessboard region
  std::span<double[2]> corners{det->p, 4};
  double min_x, max_x, min_y, max_y;
  for (const auto corner : corners) {
    min_x = std::min(corner[0], min_x);
    max_x = std::max(corner[0], max_x);
    min_y = std::min(corner[1], min_y);
    max_y = std::max(corner[1], max_y);
  }

  const cv::Mat internal =
      img(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));

  std::vector<cv::Point2f> chess_corners;
  const auto found =
      cv::findChessboardCorners(internal, cv::Size(4, 4), chess_corners);

  // get  vector from 1-4
  Eigen::Vector2d xaxis{corners[3][0] - corners[0][0],
                        corners[3][1] - corners[0][1]};
  Eigen::Vector2d yaxis{corners[1][0] - corners[0][0],
                        corners[1][1] - corners[0][1]};
  yaxis.normalize();
  xaxis.normalize();

  if (!chess_corners.empty()) {
    cv::Mat gray;
    cv::cvtColor(internal, gray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(
        gray, chess_corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 60,
                         0.1));

    std::vector<Corner> sorted_corners;
    for (const auto &p : chess_corners) {
      Eigen::Vector2d pv{p.x - (corners[0][0] - min_x),
                         p.y - (corners[0][1] - min_y)};

      sorted_corners.push_back(Corner{.p = p,
                                      .xdist = distance_to_vector(pv, yaxis),
                                      .ydist = distance_to_vector(pv, xaxis)});
    }

    std::sort(sorted_corners.begin(), sorted_corners.end(),
              [](const Corner &a, const Corner &b) {
                if (std::abs(a.ydist - b.ydist) < 5.0) {
                  return a.xdist < b.xdist;
                }
                return a.ydist < b.ydist;
              });

    chess_corners.clear();
    for (const auto &corner : sorted_corners) {
      chess_corners.push_back(corner.p);
    }

    cv::drawChessboardCorners(internal, cv::Size(4, 4), chess_corners, found);
    cv::imshow(name, internal);
  }
}

int main(int argc, char **argv) {
  std::cout << "Hello world\n";

  /// ~~~~~~ configure realsense ~~~~~~

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 800, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 800, RS2_FORMAT_Y8, 30);

  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  auto dev = devices.front();
  auto depth_sensor = dev.first<rs2::depth_sensor>();
  depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

  rs2::pipeline pipe;
  pipe.start(cfg);

  auto rgb_stream_profile =
      pipe.get_active_profile().get_stream(RS2_STREAM_COLOR);
  auto irleft_stream_profile =
      pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED, 1);
  auto irright_stream_profile =
      pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED, 2);

  // get the intrinsics from the realsense
  const auto rgb_intrinsics =
      rgb_stream_profile.as<rs2::video_stream_profile>().get_intrinsics();
  const auto irleft_intrinsics =
      irleft_stream_profile.as<rs2::video_stream_profile>().get_intrinsics();
  const auto irright_intrinsics =
      irright_stream_profile.as<rs2::video_stream_profile>().get_intrinsics();

  // get the extrinsics from the realsense
  const auto irleft_extrinsics =
      irleft_stream_profile.get_extrinsics_to(rgb_stream_profile);
  const auto irright_extrinsics =
      irright_stream_profile.get_extrinsics_to(rgb_stream_profile);

  // create the camera models
  ProjectionMatrix colorK = intrinsics_from_rs(rgb_intrinsics);
  ProjectionMatrix irleftK = intrinsics_from_rs(irleft_intrinsics);
  ProjectionMatrix irrightK = intrinsics_from_rs(irright_intrinsics);

  const Sophus::SE3d T_world_color{Eigen::Matrix4d::Identity()};
  const Sophus::SE3d T_world_irleft = extrinsics_from_rs(irleft_extrinsics);
  const Sophus::SE3d T_world_irright = extrinsics_from_rs(irright_extrinsics);

  std::cout << T_world_irleft.matrix() << std::endl;
  std::cout << T_world_irright.matrix() << std::endl;

  /// ~~~~~~ setup apriltag detector ~~~~~~
  apriltag_detector_t *td = apriltag_detector_create();
  td->refine_edges = true;
  // apriltag_family_t *tf = tag16h5_create();
  apriltag_family_t *tf = tagCustom20h10_create();
  apriltag_detector_add_family(td, tf);

  for (size_t i = 0; i < 60; i++) {
    rs2::frameset frames = pipe.wait_for_frames();
  }

  Observation color, irleft, irright, multi, average_obs;

  size_t data_count = 0;
  while (true) {
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame irleft_frame = frames.get_infrared_frame(1);
    rs2::frame irright_frame = frames.get_infrared_frame(2);

    cv::Mat color_mat(cv::Size(1920, 1080), CV_8UC3,
                      const_cast<void *>(color_frame.get_data()));
    cv::Mat irleft_mat(cv::Size(1280, 800), CV_8UC1,
                       const_cast<void *>(irleft_frame.get_data()));
    cv::Mat irright_mat(cv::Size(1280, 800), CV_8UC1,
                        const_cast<void *>(irright_frame.get_data()));

    // create bgr8 versions of the ir frames
    cv::Mat irleft_color;
    cv::cvtColor(irleft_mat, irleft_color, cv::COLOR_GRAY2BGR);
    cv::Mat irright_color;
    cv::cvtColor(irright_mat, irright_color, cv::COLOR_GRAY2BGR);

    // convert color image to grayscale
    cv::Mat gray;
    cv::cvtColor(color_mat, gray, cv::COLOR_BGR2GRAY);

    // detect apriltags
    image_u8_t color_img{.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};

    image_u8_t irleft_img{.width = irleft_mat.cols,
                          .height = irleft_mat.rows,
                          .stride = irleft_mat.cols,
                          .buf = irleft_mat.data};

    image_u8_t irright_img{.width = irright_mat.cols,
                           .height = irright_mat.rows,
                           .stride = irright_mat.cols,
                           .buf = irright_mat.data};

    auto detections_color = apriltag_detector_detect(td, &color_img);
    auto detections_irleft = apriltag_detector_detect(td, &irleft_img);
    auto detections_irright = apriltag_detector_detect(td, &irright_img);

    if (zarray_size(detections_color) > 0 &&
        zarray_size(detections_irleft) > 0 &&
        zarray_size(detections_irright) > 0) {

      const auto best_color = get_best_detection(detections_color);
      const auto best_irleft = get_best_detection(detections_irleft);
      const auto best_irright = get_best_detection(detections_irright);

      // draw best detections
      draw_tag(color_mat, best_color);
      draw_tag(irleft_color, best_irleft);
      draw_tag(irright_color, best_irright);

      // estimate positions from each camera
      Sophus::SE3d color_transform =
          estimate_tag_position(colorK(0, 0), colorK(1, 1), colorK(0, 2),
                                colorK(1, 2), tag_size, best_color);

      Sophus::SE3d irleft_transform =
          estimate_tag_position(irleftK(0, 0), irleftK(1, 1), irleftK(0, 2),
                                irleftK(1, 2), tag_size, best_irleft);

      Sophus::SE3d irright_transform =
          estimate_tag_position(irrightK(0, 0), irrightK(1, 1), irrightK(0, 2),
                                irrightK(1, 2), tag_size, best_irright);

      extract_chessboard("color_chess", color_mat, best_color, color_transform);
      // extract_chessboard("irleft_chess", irleft_mat, best_irleft,
      //                    irleft_transform);
      // extract_chessboard("irright_chess", irright_mat, best_irright,
      //                    irright_transform);

      // stragegy 1: refine using non-linear optimization of reprojection errors
      NCamPnPRefineProblem problem;
      problem |
          camera_from_detection(colorK, T_world_color, best_color, tag_size,
                                1.0) |
          camera_from_detection(irleftK, T_world_irleft, best_irleft, tag_size,
                                0.5) |
          camera_from_detection(irrightK, T_world_irright, best_irright,
                                tag_size, 0.5);
      const Sophus::SE3d multi_transform = problem.solve();

      // stragety 2: average the transforms
      std::vector<Sophus::SE3d> transforms{color_transform,
                                           T_world_irleft * irleft_transform,
                                           T_world_irright * irright_transform};
      std::vector<double> weights{0.5, 0.25, 0.25};

      Eigen::Vector3d average_translation = Eigen::Vector3d::Zero();
      for (size_t i = 0; i < transforms.size(); i++) {
        average_translation += weights.at(i) * transforms.at(i).translation();
      }

      double average_angle_x = 0.0;
      double average_angle_y = 0.0;
      double average_angle_z = 0.0;

      for (size_t i = 0; i < transforms.size(); i++) {
        average_angle_x += weights.at(i) * transforms.at(i).angleX();
        average_angle_y += weights.at(i) * transforms.at(i).angleY();
        average_angle_z += weights.at(i) * transforms.at(i).angleZ();
      }

      // color.add_transform(color_transform);
      // irleft.add_transform(T_world_irleft * irleft_transform);
      // irright.add_transform(T_world_irright * irright_transform);
      // multi.add_transform(multi_transform);
      // average_obs.translation.push_back({average_translation.x(),
      //                                    average_translation.y(),
      //                                    average_translation.z()});
      // average_obs.rotation.push_back(
      //     {average_angle_x, average_angle_y, average_angle_z});

      // std::cout << data_count++ << " "
      //           << multi_transform.translation().transpose() << std::endl;
    }

    cv::imshow("color", color_mat);
    cv::imshow("irleft", irleft_color);
    cv::imshow("irright", irright_color);

    if (cv::waitKey(1) == 'q') {
      break;
    }

    zarray_destroy(detections_color);
    zarray_destroy(detections_irleft);
    zarray_destroy(detections_irright);
  }

  // Data data{.observations = {{"color", color},
  //                            {"irleft", irleft},
  //                            {"irright", irright},
  //                            {"multi", multi},
  //                            {"average", average_obs}}};

  // dump data to file
  // std::fstream data_file("data.json", std::ios::out);
  // nlohmann::json j = data;
  // data_file << std::setw(4) << j << std::endl;
  // data_file.close();
}