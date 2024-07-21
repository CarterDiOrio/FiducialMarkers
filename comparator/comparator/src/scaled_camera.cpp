#include "comparator/scaled_camera.hpp"
#include "comparator/camera.hpp"

#include <memory>
#include <opencv2/imgproc.hpp>

ScaledCamera::ScaledCamera(
  std::shared_ptr<CameraIntf> camera,
  double scaled_factor
) : camera(camera), scale_factor(scaled_factor)
{}

void ScaledCamera::set_scale(double scaled_factor)
{
  scale_factor = scaled_factor;
}

cv::Size ScaledCamera::get_resolution() const
{
  cv::Size resolution = camera->get_resolution();
  return cv::Size(resolution.width * scale_factor, resolution.height * scale_factor);
}

cv::Mat ScaledCamera::get_frame() const
{
  cv::Mat frame = camera->get_frame();
  cv::Mat scaled_frame;
  cv::resize(frame, scaled_frame, get_resolution(), cv::INTER_AREA);
  return scaled_frame;
}

cv::Mat ScaledCamera::get_intrinsics() const
{
  cv::Mat intrinsics = camera->get_intrinsics();
  intrinsics.at<double>(0, 0) *= scale_factor * 0.66;
  intrinsics.at<double>(1, 1) *= scale_factor * 0.66;
  intrinsics.at<double>(0, 2) *= scale_factor * 0.66;
  intrinsics.at<double>(1, 2) *= scale_factor * 0.66;
  return intrinsics;
}
