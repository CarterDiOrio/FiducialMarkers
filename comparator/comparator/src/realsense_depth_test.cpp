#include <algorithm>
#include <common/zarray.h>
#include <cstdint>
#include <iostream>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <fstream>
#include "comparator/mrcal_reprojected_camera.hpp"

void draw_tag(cv::Mat& image, apriltag_detection_t* det) {
  cv::Point2f corners[4];
  for (int i = 0; i < 4; i++) {
    corners[i] = cv::Point2f(det->p[i][0], det->p[i][1]);
  }

  cv::line(image, corners[0], corners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(image, corners[1], corners[2], cv::Scalar(0, 255, 0), 4);
  cv::line(image, corners[2], corners[3], cv::Scalar(0, 255, 0), 4);
  cv::line(image, corners[3], corners[0], cv::Scalar(0, 255, 0), 4);
  cv::circle(image, cv::Point2d{det->c[0], det->c[1]}, 5, cv::Scalar(0, 0, 255), -1);
}

cv::Mat create_tag_mask(const cv::Mat& image, apriltag_detection_t* det) {
  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U);
  cv::Point2i corners[4];
  for (int i = 0; i < 4; i++) {
    corners[i] = cv::Point2f(det->p[i][0], det->p[i][1]);
  }
  cv::fillConvexPoly(mask, corners, 4, cv::Scalar(255));
  return mask;
}

/// @brief Fits a plane to a set of points
/// @pre assumes that the points are centered around the origin
/// @param points The points to fit the plane to
/// @return The normal of the plane
Eigen::Vector3d fit_plane_to_points(const std::vector<Eigen::Vector3d>& points) {
  Eigen::Matrix3Xd A(3, points.size());
  for (size_t i = 0; i < points.size(); i++) {
    A.block<3, 1>(0, i) = points[i];
  }

  Eigen::JacobiSVD<Eigen::Matrix3Xd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  Eigen::Vector3d normal = svd.matrixU().col(2);
  return normal;
}

Eigen::Vector3d deproject_point(double px, double py, double depth, double fx, double fy, double cx, double cy) {
  Eigen::Vector3d  p3d {
    (px - cx) / fx,
    (py - cy) / fy,
    1.0
  };

  return p3d * depth;
}

int main(int argc, char** argv) {
  std::cout << "Hello, World!" << std::endl;

  // load mrcal models
  const auto spline_model = load_lens_model("spline.cameramodel");
  const auto pinhole_model = load_lens_model("pinhole.cameramodel");
  const auto reprojection_maps = create_reprojection_map(
    *spline_model, 
    *pinhole_model, 
    cv::Size{1920, 1080}
  );

  const double fx = pinhole_model->intrinsics[0];
  const double fy = pinhole_model->intrinsics[1];
  const double cx = pinhole_model->intrinsics[2];
  const double cy = pinhole_model->intrinsics[3];
  
  // setup april tag detector
  apriltag_detector_t* td = apriltag_detector_create();
  td->refine_edges = true;
  apriltag_family_t* tf = tag16h5_create();
  apriltag_detector_add_family(td, tf);

  // setup realsense camera
  rs2::config cfg;
  cfg.enable_stream(
    RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8,
    30);
  cfg.enable_stream(
    RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16,
    30);

  // aligning depth to the color frame
  rs2::align align_to_color(RS2_STREAM_COLOR);

  rs2::pipeline pipeline;
  pipeline.start(cfg);

  // get depth scale
  const auto depth_sensor = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
  const auto depth_scale = depth_sensor.get_depth_scale();

  // get rgb intrinsics
  const auto realsense_intrinsics =
    pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()
    .
    get_intrinsics();

  apriltag_detection_info_t info;
  info.tagsize = 0.090;
  info.fx = fx;
  info.fy = fy;
  info.cx = cx;
  info.cy = cy;
  

  std::fstream file(argv[1], std::ios::out);

  while (true)  {

    // get frames
    rs2::frameset frames = pipeline.wait_for_frames();
    
    // align frames
    const auto aligned_frames = align_to_color.process(frames);

    const auto color_frame = aligned_frames.get_color_frame();
    auto depth_frame = aligned_frames.get_depth_frame();

    // add temporal filter
    rs2::temporal_filter filter;
    depth_frame = filter.process(depth_frame);

    // add spatial filter
    rs2::spatial_filter spatial_filter{};
    depth_frame = spatial_filter.process(depth_frame);

    // hole filling filter
    rs2::hole_filling_filter hole_filling_filter{};
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    depth_frame = hole_filling_filter.process(depth_frame);

    // convert to cv mat
    cv::Mat color_image(
      cv::Size(
        color_frame.as<rs2::video_frame>().get_width(),
        color_frame.as<rs2::video_frame>().get_height()
      ),
      CV_8UC3,
      (void *)color_frame.get_data(),
      cv::Mat::AUTO_STEP
    );

    // convert depth image to mat
    cv::Mat depth_image(
      cv::Size(
        depth_frame.as<rs2::video_frame>().get_width(),
        depth_frame.as<rs2::video_frame>().get_height()
      ),
      CV_16UC1,
      (void *)depth_frame.get_data(),
      cv::Mat::AUTO_STEP
    );

    // reproject color image
    cv::Mat remapped_frame;
    cv::remap(
      color_image,
      remapped_frame,
      reprojection_maps.map_x,
      reprojection_maps.map_y,
      cv::INTER_LINEAR
    );

    //remap depth image
    cv::Mat remapped_depth;
    cv::remap(
      depth_image,
      remapped_depth,
      reprojection_maps.map_x,
      reprojection_maps.map_y,
      cv::INTER_LINEAR
    );

    cv::Mat gray;
    cv::cvtColor(remapped_frame, gray, cv::COLOR_BGR2GRAY);

    // detect april tag
    image_u8_t april_tag_img {
      .width = gray.cols,
      .height = gray.rows,
      .stride = gray.cols,
      .buf = gray.data
    };
    zarray_t* detections = apriltag_detector_detect(td, &april_tag_img);
    
    float decision_margin = 0.0;
    int max_idx = -1;
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      if (det->decision_margin > decision_margin) {
        decision_margin = det->decision_margin;
        max_idx = i;
      }
    }

    if (max_idx >= 0) {
      apriltag_detection_t* det;
      zarray_get(detections, max_idx, &det);
      draw_tag(remapped_frame, det);
      
      info.det = det;
      apriltag_pose_t pose;
      estimate_tag_pose(&info, &pose);

      Sophus::SO3d tagR{Eigen::Map<Eigen::Matrix3d>(pose.R->data)};
      Eigen::Vector3d tagT = Eigen::Map<Eigen::Vector3d>(pose.t->data);

      
      // ~~~~~~~~ To fit a plane, find all valid points in the tag area ~~~~~~~~
      
      // create a mask from the tag
      const auto tag_mask = create_tag_mask(remapped_frame, det);

      // mask out the depth values
      cv::Mat masked_depth;
      remapped_depth.copyTo(masked_depth, tag_mask);

      // get the depth values
      std::vector<cv::Point2i> non_zero_points;
      cv::findNonZero(masked_depth, non_zero_points);
      
      std::vector<Eigen::Vector3d> points;
      double average_depth = 0.0;
      for (const auto& point : non_zero_points) {
        const auto depth = masked_depth.at<uint16_t>(point.y, point.x) * depth_scale;
        Eigen::Vector3d p3d = deproject_point(point.x, point.y, depth, fx, fy, cx, cy);
        average_depth += depth;
        points.push_back(p3d);
      }
      average_depth /= non_zero_points.size();

      Eigen::Vector3d center_point = deproject_point(
        det->c[0], det->c[1], average_depth, fx, fy, cx, cy
      );

      for (auto& point : points) {
        point -= center_point;
      }


      // fit a plane to the points
      Eigen::Vector3d normal = fit_plane_to_points(points);

      // tag always points towards camera
      if (normal(2) > 0) {
        normal *= -1;
      }

      Eigen::Vector3d offset = center_point + normal * 0.1;
      // project offset back to image
      Eigen::Vector3d offset_px = {
        offset(0) * fx / offset(2) + cx,
        offset(1) * fy / offset(2) + cy,
        1.0
      };

      Eigen::Vector3d tag_normal = tagR * Eigen::Vector3d{0, 0, 1};

      Eigen::Vector3d tag_offset = center_point + tag_normal * 0.1;
      Eigen::Vector3d tag_offset_px = {
        tag_offset(0) * fx / tag_offset(2) + cx,
        tag_offset(1) * fy / tag_offset(2) + cy,
        1.0
      };


      // for (const auto& point: points) {
      //   // project point back to image

      //   Eigen::Vector3d p = point + center_point;
      //   Eigen::Vector3d p_px = {
      //     p(0) * fx / p(2) + cx,
      //     p(1) * fy / p(2) + cy,
      //     1.0
      //   };

      //   cv::circle(remapped_frame, cv::Point2d{p_px(0), p_px(1)}, 1, cv::Scalar(0, 0, 255), -1);
      // }

      cv::line(remapped_frame, cv::Point2d{det->c[0], det->c[1]}, cv::Point2d{offset_px(0), offset_px(1)}, cv::Scalar(255, 0, 0), 5);
      cv::line(remapped_frame, cv::Point2d{det->c[0], det->c[1]}, cv::Point2d{tag_offset_px(0), tag_offset_px(1)}, cv::Scalar(0, 255, 0), 5);

      double angleX = -std::atan2(normal(1), -normal(2)) * 180.0 / M_PI;
      double angleY = std::atan2(normal(0), -normal(2)) * 180.0 / M_PI;

      std::cout << "Tag Position: " << tagT.transpose() << std::endl; 
      std::cout << "Tag Normal: " << tag_normal.transpose() << std::endl;
      std::cout << "Tag Angle X: " << tagR.angleX() * 180.0 / M_PI << " Tag Angle Y: " << tagR.angleY() * 180.0 / M_PI << std::endl;
      std::cout << "Center point: " << center_point.transpose() << std::endl;
      std::cout << "Normal: " << normal.transpose() << std::endl;
      std::cout << "AngleX: " << angleX << " AngleY: " << angleY << std::endl;
      std::cout << std::endl;

      file << tagT.transpose() << " " << center_point.transpose() << " " <<
        tagR.angleX() * 180.0 / M_PI << " " << tagR.angleY() * 180.0 / M_PI << " " <<
        angleX << " " << angleY << std::endl;
    }

    // colorize and display depth image
    cv::Mat depth_color;
    cv::Mat depth_scaled;
    cv::normalize(remapped_depth, depth_scaled, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(depth_scaled, depth_color, cv::COLORMAP_JET);
    cv::imshow("depth", depth_color);

    cv::imshow("color", remapped_frame);

    zarray_destroy(detections);


    const auto c =cv::waitKey(16);
    if (c == 'q') {
      break;
    }
  }

  file.close();

  return 0;
}