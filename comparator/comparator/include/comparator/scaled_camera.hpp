/**
  * @file scaled_camera.hpp
  * @brief This file defines the ScaledCamera class that takes in another
  * camera and scales the resolution of the camera by a factor.
  */

#ifndef INC_GUARD_SCALED_CAMERA_HPP
#define INC_GUARD_SCALED_CAMERA_HPP


#include "comparator/camera.hpp"
#include <memory>

/// @brief A camera that scales the resolution of another camera
class ScaledCamera : public CameraIntf
{
public:
  /// @brief Constructor
  /// @param camera the camera to scale
  /// @param scale_factor the factor to scale the resolution by
  ScaledCamera(
    std::shared_ptr<CameraIntf> camera,
    double scale_factor
  );

  /// @brief Set the scale factor
  /// @param scale_factor the factor to scale the resolution by
  void set_scale(double scale_factor);

  cv::Size get_resolution() const override;

  cv::Mat get_frame() const override;

  cv::Mat get_intrinsics() const override;

private:

  /// @brief The camera to scale
  std::shared_ptr<CameraIntf> camera;

  /// @brief The factor to scale the resolution by
  double scale_factor;
};

#endif