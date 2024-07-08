#ifndef INC_GUARD_EXTRINSIC_CALIBRATION_HPP
#define INC_GUARD_EXTRINSIC_CALIBRATION_HPP

#include "comparator/extrinsic_observation.hpp"
#include "comparator/mount.hpp"
#include <sophus/se3.hpp>
#include <ceres/problem.h>


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
  Sophus::SE3d T_world_object;

  static ExtrinsicCalibration Identity()
  {
    return {
      Sophus::SE3d{Eigen::Matrix4d::Identity()},
      Sophus::SE3d{Eigen::Matrix4d::Identity()},
      Sophus::SE3d{Eigen::Matrix4d::Identity()}
    };
  }
};

struct OptimizationInputs
{
  /// \brief The observations for the extrinsic calibration
  ExtrinsicObservations observations;

  /// \brief the initial guess of T_hand_camera and T_mount_fiducial
  ExtrinsicCalibration initial_guess;

  /// \brief Initial transformation guesses from the calibration object to
  /// the camera
  std::vector<Sophus::SE3d> T_eye_objects;
};

struct ExtrinsicCalibrationData
{
  ExtrinsicCalibration calibration;
  std::vector<double> residuals{};
};

/// \brief Calibrates the extrinsics of the system
ExtrinsicCalibrationData calibrate_extrinsics(
  ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibration & initial_guess = ExtrinsicCalibration::Identity()
);

/// \brief Optimizes the extrinsics of the system
ExtrinsicCalibrationData optimize_extrinsics(
  OptimizationInputs & inputs,
  const Mount & mount,
  const cv::Mat & K
);

/// \brief Adds the hand-eye calibration problem to the ceres problem
void add_hand_eye_problem(
  ceres::Problem & problem,
  Sophus::SE3d & T_hand_eye,
  Sophus::SE3d & T_world_object,
  std::vector<Sophus::SE3d> & T_eye_objects,
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K
);

/// \brief Adds the mount calibration problem to the ceres problem
void add_mount_fiducial_problem(
  ceres::Problem & problem,
  Sophus::SE3d & T_mount_fiducial,
  Sophus::SE3d & T_world_object,
  const ExtrinsicObservations & observations
);

/// \brief Seeds an initial guess for the extrinsic calibration
/// \param observations The observations to seed the calibration with
/// \param mount The mount used in the observations
std::vector<Sophus::SE3d> seed_extrinsic_calibration(
  const ExtrinsicObservations & observations,
  const ExtrinsicCalibration & initial_guess,
  const Mount & mount,
  const cv::Mat & K
);

/// \brief Performs PnP using OpenCV's solvePnP
/// \param object_points The object points
/// \param image_points The image points
/// \param K The camera matrix
/// \return The transformation from the object to the camera's frame, T_co
Sophus::SE3d solve_pnp(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & K
);

/// \brief Performs PnP using OpenCV's solvePnP
/// \param object_points The object points
/// \param image_points The image points
/// \param K The camera matrix
/// \return The transformation from the object to the camera's frame, T_co
Sophus::SE3d solve_pnp(
  const std::vector<Eigen::Vector3d> & object_points,
  const std::vector<Eigen::Vector2d> & image_points,
  const cv::Mat & K
);

double evaluate_observation(
  const ExtrinsicCalibration & extrinsics,
  const ExtrinsicObservation & observation,
  const cv::Mat K,
  const Mount & mount
);

double evaluate_pnp(
  const ExtrinsicObservation & observation,
  const Sophus::SE3d & T_eye_object,
  const cv::Mat & K,
  const Mount & mount
);

double evaluate_hand_eye(
  const ExtrinsicObservation & observation,
  const Sophus::SE3d & T_hand_eye,
  const Sophus::SE3d & T_world_object,
  const cv::Mat & K,
  const Mount & mount
);

}


#endif
