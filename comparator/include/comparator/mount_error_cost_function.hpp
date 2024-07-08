#ifndef INC_GUARD_MOUNT_ERROR_COST_FUNCTION_HPP
#define INC_GUARD_MOUNT_ERROR_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

namespace calibration
{


struct MountErrorCostFunction
{
  MountErrorCostFunction(
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

    // calculate the pose graph error for the mount
    const SE3 T_world_object_calc =
      (T_world_mount_m * T_mount_object).inverse();

    const SE3 err = T_world_object * T_world_object_calc;

    // map the residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals{residuals_ptr};
    residuals = err.log();

    return true;
  }

  static ceres::CostFunction * Create(
    const Sophus::SE3d & T_world_mount_m)
  {
    return new ceres::AutoDiffCostFunction<MountErrorCostFunction, 6,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters>(
      new MountErrorCostFunction(T_world_mount_m));
  }

  const Sophus::SE3d T_world_mount_m;
};

}

#endif
