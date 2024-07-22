#ifndef INC_GUARD_EXTRINSIC_CALIBRATION_HPP
#define INC_GUARD_EXTRINSIC_CALIBRATION_HPP

#include "comparator/extrinsic_observation.hpp"
#include "comparator/mount.hpp"
#include "comparator/eigen_json.hpp"
#include <memory>
#include <sophus/se3.hpp>
#include <ceres/problem.h>
#include <nlohmann/json_fwd.hpp>


namespace calibration
{

struct ExtrinsicCalibration
{
  Sophus::SE3d T_hand_eye;
  Sophus::SE3d T_mount_fiducial;

  static ExtrinsicCalibration Identity()
  {
    return {
      Sophus::SE3d{Eigen::Matrix4d::Identity()},
      Sophus::SE3d{Eigen::Matrix4d::Identity()},
    };
  }
};
void to_json(json & j, const ExtrinsicCalibration & cal);
void from_json(const json & j, ExtrinsicCalibration & cal);


struct OptimizationInputs
{
  /// \brief The observations for the extrinsic calibration
  std::optional<ExtrinsicObservations> camera_stationary_observations;

  /// \brief The observations for the extrinsic calibration
  std::optional<ExtrinsicObservations> mount_stationary_observations;

  /// \brief the initial guess of T_hand_camera and T_mount_fiducial
  ExtrinsicCalibration initial_guess;
};

struct ExtrinsicCalibrationData
{
  ExtrinsicCalibration calibration;
  std::vector<double> residuals{};
};

/// \brief Calibrates the extrinsics of the system
ExtrinsicCalibrationData calibrate_extrinsics(
  OptimizationInputs & inputs,
  const Mount & mount,
  const cv::Mat & K
);

/// \brief Optimizes the extrinsics of the system
ExtrinsicCalibrationData optimize_extrinsics(
  OptimizationInputs & inputs,
  const Mount & mount,
  const cv::Mat & K
);

struct ProblemOutput {
  std::vector<Sophus::SE3d> T_eye_objects;
  std::unique_ptr<Sophus::SE3d> transform;
};


ProblemOutput add_camera_stationary(ceres::Problem & problem,
                           Sophus::SE3d & T_hand_eye,
                           Sophus::SE3d & T_mount_object,
                           const Mount & mount,
                           const cv::Mat & K,
                           const ExtrinsicObservations & observations);

ProblemOutput add_mount_stationary(ceres::Problem & problem,
                          Sophus::SE3d & T_hand_eye,
                          Sophus::SE3d & T_mount_object,
                          const Mount & mount,
                          const cv::Mat & K,
                          const ExtrinsicObservations & observations);


/// \brief Seeds an initial guess for the extrinsic calibration
/// \param observations The observations to seed the calibration with
/// \param mount The mount used in the observations
std::vector<Sophus::SE3d> seed_extrinsic_calibration(
  const ExtrinsicObservations & observations,
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
