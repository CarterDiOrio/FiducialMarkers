#ifndef INC_GUARD_REPOREJECTION_COST_FUNCTION_HPP
#define INC_GUARD_REPOREJECTION_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

namespace calibration
{

/// \brief A ceres autodiff cost function for reprojecting a point
struct ReprojectionCostFunction
{

  ReprojectionCostFunction(
    const Eigen::Matrix3d & K,
    const Eigen::Vector2d & observed_point,
    const Eigen::Vector3d & point_object
  )
  : K{K},
    observed_point{observed_point},
    point_object{point_object} {}

  /// \brief The operator() function for the cost function
  /// \param T_eo The transformation from the object to the eye/camera
  template<typename T>
  bool operator()(
    const T * T_eo,
    T * residuals) const
  {
    // map the input se3 manifold to a Sophus type
    Sophus::SE3<T> T_eye_object = Eigen::Map<Sophus::SE3<T> const>{T_eo};

    // transform the point to the eye frame
    Eigen::Vector<T,
      4> point_eye = T_eye_object * point_object.cast<T>().homogeneous();

    // project the point to the image plane
    const auto x = K(0, 0) * point_eye.x() / point_eye.z() + K(0, 2);
    const auto y = K(1, 1) * point_eye.y() / point_eye.z() + K(1, 2);

    // calculate the residuals
    residuals[0] = x - T{observed_point.x()};
    residuals[1] = y - T{observed_point.y()};

    return true;
  }

  static ceres::CostFunction * Create(
    const Eigen::Matrix3d & K,
    const Eigen::Vector2d & observed_point,
    const Eigen::Vector3d & point_object)
  {
    return new ceres::AutoDiffCostFunction<ReprojectionCostFunction, 2,
             Sophus::Manifold<Sophus::SE3>::num_parameters>(
      new ReprojectionCostFunction(
        K, observed_point, point_object));
  }

  const Eigen::Matrix3d K;
  const Eigen::Vector2d observed_point;
  const Eigen::Vector3d point_object;
  const Sophus::SE3d T_world_hand;
};

}

#endif
