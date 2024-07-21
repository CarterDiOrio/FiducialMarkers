#ifndef INC_GUARD_CAMERA_HPP
#define INC_GUARD_CAMERA_HPP

#include <opencv2/core.hpp>

/// \brief Interface modeling a generic camera.
class CameraIntf
{
public:
  /// \brief Gets the resolution of the Camera in pixels
  /// \return The resolution of the camera {width, height}
  virtual cv::Size get_resolution() const = 0;

  /// \brief Gets a frame from the Camera
  /// \return The frame from the camera
  virtual cv::Mat get_frame() const = 0;

  /// \brief Gets the intrinsics of the Camera
  /// \return The intrinsics of the camera
  virtual cv::Mat get_intrinsics() const = 0;
};

#endif
