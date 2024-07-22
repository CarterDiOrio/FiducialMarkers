#ifndef INC_GUARD_MOUNT_ERROR_COST_FUNCTION_HPP
#define INC_GUARD_MOUNT_ERROR_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

namespace calibration
{

struct StationaryHandCostFunction
{
  StationaryHandCostFunction(
    const Sophus::SE3d & T_world_hand_m
  )
  : T_world_hand_m{T_world_hand_m}
  {
  }

  template<typename T>
  bool operator()(
    const T * T_world_eye_param,
    const T * T_hand_eye_param,
    T * residuals_ptr
  ) const
  {
    using SE3 = Sophus::SE3<T>;

    // map the input parameters to Sophus types
    SE3 T_world_eye = Eigen::Map<SE3 const>{T_world_eye_param};
    SE3 T_hand_eye = Eigen::Map<SE3 const>{T_hand_eye_param};

    // calculate the pose graph error for the eye
    const SE3 T_world_hand_est = T_world_eye * T_hand_eye.inverse();
    const SE3 err = T_world_hand_m.inverse() * T_world_hand_est;

    // map the residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals{residuals_ptr};
    residuals = err.log();

    return true;
  }

  static ceres::CostFunction * Create(
    const Sophus::SE3d & T_world_hand_m)
  {
    return new ceres::AutoDiffCostFunction<StationaryHandCostFunction, 6,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters>(
      new StationaryHandCostFunction(T_world_hand_m));
  }

  const Sophus::SE3d T_world_hand_m;
};

struct StationaryMountCostFunction
{
  StationaryMountCostFunction(
    const Sophus::SE3d & T_world_mount_m
  )
  : T_world_mount_m{T_world_mount_m}
  {
  }

  template<typename T>
  bool operator()(
    const T * T_world_object_param,
    const T * T_mount_object_param,
    T * residuals_ptr
  ) const
  {
    using SE3 = Sophus::SE3<T>;

    // map the input parameters to Sophus types
    SE3 T_world_object = Eigen::Map<SE3 const>{T_world_object_param};
    SE3 T_mount_object = Eigen::Map<SE3 const>{T_mount_object_param};

    // calculate the pose graph error for the eye
    const SE3 T_world_mount_est = T_world_object * T_mount_object.inverse();
    const SE3 err = T_world_mount_m.inverse() * T_world_mount_est;

    // map the residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals{residuals_ptr};
    residuals = err.log();

    return true;
  }

  static ceres::CostFunction * Create(
    const Sophus::SE3d & T_world_hand_m)
  {
    return new ceres::AutoDiffCostFunction<StationaryMountCostFunction, 6,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters>(
      new StationaryMountCostFunction(T_world_hand_m));
  }

  const Sophus::SE3d T_world_mount_m;
};

}

#endif
