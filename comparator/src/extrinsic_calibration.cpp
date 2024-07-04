#include "comparator/extrinsic_calibration.hpp"
#include "comparator/extrinsic_ceres_cost_function.hpp"
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
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibration & initial_guess,
  const ExtrinsicCalibrationOptions & options
)
{
  // get a decent initial guess
  // Sophus::SE3d T_x_camera = seed_extrinsic_calibration(
  //   observations,
  //   mount,
  //   K,
  //   initial_guess,
  //   options);

  const auto extrinsics = optimize_extrinsics(
    observations,
    mount,
    K,
    initial_guess,
    options
  );

  return extrinsics;
}

ExtrinsicCalibrationData optimize_extrinsics(
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibration & initial_guess,
  const ExtrinsicCalibrationOptions &
)
{
  std::cout << "Setting up extrinsic calibration problem" << std::endl;
  Eigen::Matrix3d K_eig;
  cv::cv2eigen(K, K_eig);
  Sophus::SE3d T_mount_fiducial{initial_guess.T_mount_fiducial};
  Sophus::SE3d T_x_camera{initial_guess.T_x_camera};

  // create problem
  ceres::Problem problem;

  // In this problem the only two parameters are the two defined transforms
  auto parameterization = new Sophus::Manifold<Sophus::SE3>;
  problem.AddParameterBlock(
    T_mount_fiducial.data(),
    Sophus::SE3d::num_parameters,
    parameterization
  );

  std::cout << "T_mount_fiducial: \n" << T_mount_fiducial.matrix() << std::endl;

  problem.AddParameterBlock(
    T_x_camera.data(),
    Sophus::SE3d::num_parameters,
    parameterization
  );

  // set mount fiducial to be constant. TODO: make this an exposed option
  // problem.SetParameterBlockConstant(T_mount_fiducial.data());
  auto loss = new ceres::HuberLoss(5.0);

  // add each observation/fiducial corner to the problem
  std::vector<ceres::ResidualBlockId> ids; //save the ids for outlier rejection
  for (const auto & extrinsic_observation: observations.observations) {

    const auto & corner_observations =
      extrinsic_observation.chessboard_observations.corners;

    for (size_t i = 0; i < corner_observations.size(); i++) {
      const auto & corner_observation = corner_observations[i];
      const auto cost_function = ExtrinsicsCostFunction::Create(
        corner_observation.img_point,
        mount.fiducial_corners[i],
        extrinsic_observation.T_world_mount,
        extrinsic_observation.T_world_cmount,
        K_eig
      );   //ceres takes ownership over this memory itself


      // scaling by the scale factor used to detect the corner. The higher
      // the level on the image pyramid the smaller the corner is in the image
      double scale_factor = 1.0 / (std::pow(2.0, corner_observation.level));

      // ceres::LossFunction * scaled_loss = new ceres::ScaledLoss(
      //   new ceres::HuberLoss(), //the scaled loss function takes ownership
      //   scale_factor,
      //   ceres::TAKE_OWNERSHIP
      // ); //ceres takes ownership of this memory

      const auto id = problem.AddResidualBlock(
        cost_function,
        loss,
        T_mount_fiducial.data(),
        T_x_camera.data()
      );

      ids.push_back(id);
    }
  }

  // setup the solver
  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.sparse_linear_algebra_library_type =
    ceres::SUITE_SPARSE;
  ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres_solver_options.minimizer_progress_to_stdout = true;

  std::cout << "Solving extrinsic calibration problem..." << std::endl;
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options, &problem, &summary);


  std::cout << summary.FullReport() << std::endl;

  // calculate final residuals
  std::vector<double> residuals;
  problem.Evaluate(
    ceres::Problem::EvaluateOptions{}, nullptr, &residuals,
    nullptr, nullptr);

  return {
    {T_x_camera, T_mount_fiducial},
    residuals
  };
}


Sophus::SE3d seed_extrinsic_calibration(
  const ExtrinsicObservations & observations,
  const Mount & mount,
  const cv::Mat & K,
  const ExtrinsicCalibration & initial_guess,
  const ExtrinsicCalibrationOptions & options)
{
  // Not sure if averaging is better than throwing it all into one
  // pnp problem... should test
  std::vector<Sophus::SE3d> T_ws;
  for (const auto & observation: observations.observations) {
    const auto world_points = (observation.T_world_mount * mount.T_mp) *
      mount.fiducial_corners;

    std::vector<Eigen::Vector2d> image_points;
    for (const auto & corner: observation.chessboard_observations.corners) {
      image_points.push_back(corner.img_point);
    }

    const Sophus::SE3d T_world_camera = solve_pnp(
      world_points, image_points,
      K);

    const Sophus::SE3d T_camera_world = T_world_camera.inverse();
    const Sophus::SE3d T_camera_x = T_camera_world *
      Sophus::SE3d{observation.T_world_cmount};
    const Sophus::SE3d T_x_camera = T_camera_x.inverse();

    T_ws.push_back(T_x_camera);
  }
  return *Sophus::average(T_ws);
  // return Sophus::SE3d{};
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
  Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
  T_wc.block<3, 3>(0, 0) = R_eigen.transpose();
  T_wc.block<3, 1>(0, 3) = -R_eigen.transpose() * T;

  return Sophus::SE3d{T_wc};
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

}
