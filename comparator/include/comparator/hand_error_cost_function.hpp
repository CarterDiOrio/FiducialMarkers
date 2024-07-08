#ifndef INC_GUARD_HAND_ERROR_COST_FUNCTION_HPP
#define INC_GUARD_HAND_ERROR_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>


namespace calibration
{

/// \brief A ceres autodiff cost function for calculating hand pose error
/// This is a pose graph term
struct HandErrorCostFunction
{

  HandErrorCostFunction(
    const Sophus::SE3d & T_world_hand_m
  )
  : T_world_hand_m{T_world_hand_m} {}

  template<typename T>
  bool operator()(
    const T * T_eye_object_param,
    const T * T_hand_eye_param,
    const T * T_world_object_param,
    T * residuals_ptr) const
  {
    using SE3 = Sophus::SE3<T>;

    // map the input parameters to Sophus types
    SE3 T_eye_object = Eigen::Map<SE3 const>{T_eye_object_param};
    SE3 T_hand_eye = Eigen::Map<SE3 const>{T_hand_eye_param};
    SE3 T_world_object = Eigen::Map<SE3 const>{T_world_object_param};

    // calculate the pose graph error for the hand
    const SE3 err = T_hand_eye *
      (T_eye_object * T_world_object.inverse() * T_world_hand_m);

    //map the residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals{residuals_ptr};
    residuals = err.log();

    return true;
  }

  static ceres::CostFunction * Create(
    const Sophus::SE3d & T_world_hand_m)
  {
    return new ceres::AutoDiffCostFunction<HandErrorCostFunction, 6,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters>(
      new HandErrorCostFunction(T_world_hand_m));
  }

  const Sophus::SE3d T_world_hand_m;
};

}

#endif
