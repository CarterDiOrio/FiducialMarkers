#ifndef INC_GUARD_REALSENSE_CAMERA_IMPL_HPP
#define INC_GUARD_REALSENSE_CAMERA_IMPL_HPP

#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include "comparator/camera.hpp"

class RealSenseCamera : public CameraIntf
{
public:
  /// \brief Attempts to start the realsense camera with the given parameters
  /// \param width The width of the camera frame in px
  /// \param height The height of the camera frame in px
  /// \param framerate The framerate of the camera in fps
  RealSenseCamera(
    double width,
    double height,
    int framerate
  )
  : CameraIntf()
  {
    cfg.enable_stream(
      RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8,
      framerate);
    pipeline.start(cfg);
  }

  explicit RealSenseCamera(rs2::config cfg)
  {
    pipeline.start(cfg);
  }

  virtual cv::Size get_resolution() const override
  {
    const auto realsense_intrinsics =
      pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()
      .
      get_intrinsics();
    return cv::Size(realsense_intrinsics.width, realsense_intrinsics.height);
  }

  virtual cv::Mat get_frame() const override
  {
    // the frames are held in our control until we release our ownership over them.
    // this means if we hold them for greater than (1 / framerate) seconds, a queue
    // will begin to fill and we will have frame drops.
    rs2::frameset frames = pipeline.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();

    // interpret the frame data as a cv::Mat
    cv::Mat frame(
      cv::Size(
        color_frame.as<rs2::video_frame>().get_width(),
        color_frame.as<rs2::video_frame>().get_height()
      ),
      CV_8UC3,
      (void *)color_frame.get_data(),
      cv::Mat::AUTO_STEP
    );

    // copy the frame data out to return control of the frame to the realsense
    cv::Mat frame_copy;
    frame.copyTo(frame_copy);
    return frame_copy;
  }

  virtual cv::Mat get_intrinsics() const override
  {
    const auto realsense_intrinsics =
      pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()
      .
      get_intrinsics();

    cv::Mat intrinsics = cv::Mat::zeros(3, 3, CV_64F);
    intrinsics.at<double>(0, 0) = realsense_intrinsics.fx;
    intrinsics.at<double>(0, 2) = realsense_intrinsics.ppx;
    intrinsics.at<double>(1, 1) = realsense_intrinsics.fy;
    intrinsics.at<double>(1, 2) = realsense_intrinsics.ppy;
    intrinsics.at<double>(2, 2) = 1.0;
    return intrinsics;
  }

private:
  rs2::config cfg;
  rs2::pipeline pipeline;
};



#endif
