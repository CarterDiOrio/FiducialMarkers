#include "rsapriltag/ncam_pnp.hpp"
#include "rsapriltag/multicam_refine_cost_funcion.hpp"
#include <ceres/loss_function.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

NCamPnPRefineProblem &NCamPnPRefineProblem::operator|(Camera &&camera) {
  cameras.push_back(camera);
  return *this;
}

Sophus::SE3d NCamPnPRefineProblem::solve() {

  // ~~~~~~ lets seed solution using the first camera ~~~~~~
  const auto &cam = cameras[0];
  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  for (const auto &point : cam.points) {
    object_points.push_back(
        {point.object.x(), point.object.y(), point.object.z()});
    image_points.push_back({point.image.x(), point.image.y()});
  }

  cv::Mat K;
  Eigen::Matrix3d K_eigen = cam.K.block<3, 3>(0, 0);
  cv::eigen2cv(K_eigen, K);

  Sophus::SE3d T_world_object = solve_pnp(object_points, image_points, K);

  // ~~~~~~ setup refinement using multiple cameras ~~~~~~
  ceres::Problem problem;

  auto parameterization = new Sophus::Manifold<Sophus::SE3>;
  problem.AddParameterBlock(T_world_object.data(), Sophus::SE3d::num_parameters,
                            parameterization);

  auto loss = new ceres::HuberLoss(1.0);
  for (auto &cam : cameras) {

    for (const auto &[obj_point, image_point] : cam.points) {
      problem.AddResidualBlock(RefinePnP::Create(obj_point, image_point, cam.K,
                                                 cam.T_world_camera, cam.ppd),
                               loss, T_world_object.data());
    }
  }

  //~~~~~~ refine/solve ~~~~~~
  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  ceres_solver_options.linear_solver_type = ceres::DENSE_QR;
  // ceres_solver_options.minimizer_progress_to_stdout = true;
  ceres_solver_options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options, &problem, &summary);

  return T_world_object;
}

Sophus::SE3d solve_pnp(const std::vector<cv::Point3d> &object_points,
                       const std::vector<cv::Point2d> &image_points,
                       const cv::Mat &K) {

  cv::Mat rvec, tvec;
  cv::solvePnP(object_points, image_points, K, cv::noArray(), rvec, tvec, false,
               cv::SOLVEPNP_P3P);

  // cv::solvePnPRefineLM(object_points, image_points, K, cv::noArray(), rvec,
  //                      tvec);

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(R, R_eigen);

  Eigen::Vector3d T = {tvec.at<double>(0), tvec.at<double>(1),
                       tvec.at<double>(2)};

  Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
  T_cw.block<3, 3>(0, 0) = R_eigen;
  T_cw.block<3, 1>(0, 3) = T;

  return Sophus::SE3d{T_cw};
}
