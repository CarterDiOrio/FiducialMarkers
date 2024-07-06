#include "comparator/extrinsic_calibration.hpp"
#include "comparator/extrinsic_ceres_cost_function.hpp"
#include "comparator/extrinsic_observation.hpp"
#include "comparator/hand_error_cost_function.hpp"
#include "comparator/reprojection_cost_function.hpp"
#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>
#include <sophus/average.hpp>
#include <ceres/ceres.h>
#include <iostream>


namespace calibration
{

ExtrinsicCalibrationData calibrate_extrinsics(
  ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibration & initial_guess,
  const ExtrinsicCalibrationOptions & options
)
{
  ExtrinsicCalibrationData extrinsics;

  bool is_outliers = true;
  while (is_outliers) {
    const auto T_eye_objects = seed_extrinsic_calibration(
      observations,
      initial_guess,
      mount,
      K);

    OptimizationInputs inputs{
      .observations = observations,
      .initial_guess = initial_guess,
      .T_eye_objects = T_eye_objects
    };

    extrinsics = optimize_extrinsics(
      inputs,
      mount,
      K,
      false,
      false,
      options
    );

    // evaluate the residuals
    std::vector<double> residuals;
    for (const auto & observation: observations.observations) {
      residuals.push_back(
        evaluate_observation(
          extrinsics.calibration,
          observation,
          K,
          mount
      ));
    }

    // find std deviation
    double mean = 0;
    for (const auto & r : residuals) {
      mean += std::fabs(r);
    }
    mean /= residuals.size();

    double std_dev = 0;
    for (const auto & r : residuals) {
      std_dev += std::pow(r - mean, 2);
    }
    std_dev = std::sqrt(std_dev / residuals.size());

    // remove outliers
    is_outliers = false;
    std::vector<ExtrinsicObservation> inliers;
    for (size_t i = 0; i < residuals.size(); i++) {
      if (std::fabs(residuals[i] - mean) < 3 * std_dev) {
        inliers.push_back(observations.observations[i]);
      } else {
        is_outliers = true;
      }
    }

    if (is_outliers) {
      std::cout << "Found: " << observations.observations.size() -
        inliers.size() << " outliers. " << "Reoptimizing..." << std::endl;
    }

    observations.observations = inliers;
  }

  std::cout << "Final Size: " << observations.observations.size() << std::endl;

  return extrinsics;
}

ExtrinsicCalibrationData optimize_extrinsics(
  OptimizationInputs & inputs,
  const Mount & mount,
  const cv::Mat & K,
  bool lock_camera,
  bool lock_mount,
  const ExtrinsicCalibrationOptions &
)
{
  const auto & initial_guess = inputs.initial_guess;
  const auto & observations = inputs.observations;

  std::cout << "Setting up extrinsic calibration problem" << std::endl;
  Eigen::Matrix3d K_eig;
  cv::cv2eigen(K, K_eig);
  Sophus::SE3d T_mount_fiducial{initial_guess.T_mount_fiducial};
  Sophus::SE3d T_hand_eye{initial_guess.T_x_camera};

  // create problem
  ceres::Problem problem;

  // the mount to fiducial transform, estimated
  auto parameterization = new Sophus::Manifold<Sophus::SE3>;
  problem.AddParameterBlock(
    T_mount_fiducial.data(),
    Sophus::SE3d::num_parameters,
    parameterization
  );

  // hand eye transform, estimated
  problem.AddParameterBlock(
    T_hand_eye.data(),
    Sophus::SE3d::num_parameters,
    parameterization
  );

  // add each of the eye object transforms to the problem
  for (Sophus::SE3d & T_eye_object:  inputs.T_eye_objects) {
    problem.AddParameterBlock(
      T_eye_object.data(),
      Sophus::SE3d::num_parameters,
      parameterization
    );
  }

  // set mount fiducial to be constant. TODO: make this an exposed option
  if (lock_mount) {
    problem.SetParameterBlockConstant(T_mount_fiducial.data());
  }

  if (lock_camera) {
    problem.SetParameterBlockConstant(T_hand_eye.data());
  }
  auto loss = new ceres::HuberLoss(1.0);

  for (size_t obs_idx = 0; obs_idx < observations.observations.size();
    obs_idx++)
  {
    const auto & observation = observations.observations.at(obs_idx);

    // 1) add the reprojection error cost for each obj point
    for (size_t i = 0; i < mount.fiducial_corners.size(); i++) {
      const auto obj_point = mount.fiducial_corners[i];
      const auto img_point =
        observation.chessboard_observations.corners[i].img_point;

      problem.AddResidualBlock(
        ReprojectionCostFunction::Create(
          K_eig,
          img_point,
          obj_point
        ),
        loss,
        inputs.T_eye_objects.at(obs_idx).data()
      );
    }


    // 2) add hand position error cost
    problem.AddResidualBlock(
      HandErrorCostFunction::Create(
        Sophus::SE3d{observation.T_world_cmount},
        Sophus::SE3d{observation.T_world_mount}
      ),
      loss,
      inputs.T_eye_objects.at(obs_idx).data(),
      T_hand_eye.data(),
      T_mount_fiducial.data()
    );
  }

  // setup the solver
  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.sparse_linear_algebra_library_type =
    ceres::SUITE_SPARSE;
  ceres_solver_options.linear_solver_type = ceres::DENSE_QR;
  ceres_solver_options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options, &problem, &summary);

  // calculate final residuals
  // std::vector<double> residuals;
  // problem.Evaluate(
  //   ceres::Problem::EvaluateOptions{}, nullptr, &residuals,
  //   nullptr, nullptr);

  return {
    {T_hand_eye, T_mount_fiducial},
    {}
  };
}


std::vector<Sophus::SE3d> seed_extrinsic_calibration(
  const ExtrinsicObservations & observations,
  const ExtrinsicCalibration &,
  const Mount & mount,
  const cv::Mat & K)
{
  std::vector<Sophus::SE3d> transforms;
  for (const auto & observation: observations.observations) {
    const auto obj_points = mount.fiducial_corners;

    std::vector<Eigen::Vector2d> image_points;
    for (const auto & corner: observation.chessboard_observations.corners) {
      image_points.push_back(corner.img_point);
    }

    const Sophus::SE3d T_eye_object = solve_pnp(
      obj_points, image_points,
      K);

    transforms.push_back(T_eye_object);
  }
  return transforms;
}

Sophus::SE3d solve_pnp(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & K
)
{
  cv::Mat rvec, tvec;
  cv::solvePnPRansac(
    object_points,
    image_points,
    K,
    cv::noArray(),
    rvec,
    tvec,
    false,
    350,
    8.0,
    0.999,
    cv::noArray(),
    cv::SOLVEPNP_ITERATIVE
  );

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(R, R_eigen);

  Eigen::Vector3d T =
  {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
  // Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
  // T_wc.block<3, 3>(0, 0) = R_eigen.transpose();
  // T_wc.block<3, 1>(0, 3) = -R_eigen.transpose() * T;

  Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
  T_cw.block<3, 3>(0, 0) = R_eigen;
  T_cw.block<3, 1>(0, 3) = T;

  return Sophus::SE3d{T_cw};
}

Sophus::SE3d solve_pnp(
  const std::vector<Eigen::Vector3d> & object_points,
  const std::vector<Eigen::Vector2d> & image_points,
  const cv::Mat & K
)
{
  std::vector<cv::Point3f> object_points_cv;
  std::vector<cv::Point2f> image_points_cv;

  for (const auto & point: object_points) {
    object_points_cv.push_back(cv::Point3f(point.x(), point.y(), point.z()));
  }

  for (const auto & point: image_points) {
    image_points_cv.push_back(cv::Point2f(point.x(), point.y()));
  }

  return solve_pnp(object_points_cv, image_points_cv, K);
}

double evaluate_observation(
  const ExtrinsicCalibration & extrinsics,
  const ExtrinsicObservation & observation,
  const cv::Mat K,
  const Mount & mount)
{
  std::vector<double> errors;
  for (size_t i = 0; i < mount.fiducial_corners.size(); i++) {
    const auto & corner = mount.fiducial_corners[i];
    const auto & img_point =
      observation.chessboard_observations.corners[i].img_point;

    Eigen::Vector4d p = observation.T_world_mount *
      extrinsics.T_mount_fiducial.matrix() * corner.homogeneous();
    Eigen::Vector3d p_camera =
      (extrinsics.T_x_camera.inverse().matrix() *
      observation.T_world_cmount.inverse() * p).head<3>();
    Eigen::Vector2d p_image{
      (p_camera.x() / p_camera.z()) * K.at<double>(0, 0) + K.at<double>(0, 2),
      (p_camera.y() / p_camera.z()) * K.at<double>(1, 1) + K.at<double>(1, 2)
    };
    errors.push_back(img_point.x() - p_image.x());
    errors.push_back(img_point.y() - p_image.y());
  }

  double mean = 0;
  for (const auto & r : errors) {
    mean += std::fabs(r);
  }

  mean /= errors.size();
  return mean;
}

}
