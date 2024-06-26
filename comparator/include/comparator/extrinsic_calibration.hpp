#ifndef INC_GUARD_EXTRINSIC_CALIBRATION_HPP
#define INC_GUARD_EXTRINSIC_CALIBRATION_HPP

#include "comparator/extrinsic_observation.hpp"
#include "comparator/mount.hpp"
#include <sophus/se3.hpp>

namespace calibration
{

struct ExtrinsicCalibrationOptions
{
  /// \brief If true the extrinsic calibration performs a camera to hand
  /// and not camera to world calibration.
  /// This is useful when the camera is mounted to an object that provides a
  /// transformation to the world. I.E. robot arm, tracking mount
  bool camera_to_hand {false};
};

struct ExtrinsicCalibration
{
  Sophus::SE3d T_x_camera;
  Sophus::SE3d T_mount_fiducial;
};

/// \brief Calibrates the extrinsics of the system
ExtrinsicCalibration calibrate_extrinsics(
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibrationOptions & options = ExtrinsicCalibrationOptions{}
);

/// \brief Optimizes the extrinsics of the system
ExtrinsicCalibration optimize_extrinsics(
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibration & initial_guess,
  const ExtrinsicCalibrationOptions & options = ExtrinsicCalibrationOptions{}
);

/// \brief Seeds an initial guess for the extrinsic calibration
/// \param observations The observations to seed the calibration with
/// \param mount The mount used in the observations
Sophus::SE3d seed_extrinsic_calibration(
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibrationOptions & options = ExtrinsicCalibrationOptions{}
);

/// \brief Performs PnP using OpenCV's solvePnP
/// \param ExtrinsicObservation The observation to use
/// \param K The camera matrix
Sophus::SE3d solve_pnp(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & K
);

/// \brief Performs PnP using OpenCV's solvePnP
/// \param ExtrinsicObservation The observation to use
/// \param K The camera matrix
Sophus::SE3d solve_pnp(
  const std::vector<Eigen::Vector3d> & object_points,
  const std::vector<Eigen::Vector2d> & image_points,
  const cv::Mat & K
);

}


#endif
