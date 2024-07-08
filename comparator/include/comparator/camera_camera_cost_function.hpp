#ifndef INC_GUARD_CAMERA_CAMERA_COST_FUNCTION_HPP
#define INC_GUARD_CAMERA_CAMERA_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>


namespace calibration
{

/// \brief A ceres autodiff cost function for calculating hand pose error
/// This is a pose graph term
struct CameraCameraCostFucntion
{

  CameraCameraCostFucntion(
    const Sophus::SE3d & T_world_hand1_m,
    const Sophus::SE3d & T_world_hand2_m
  )
  : T_world_hand1_m{T_world_hand1_m}, T_world_hand2_m{T_world_hand2_m} {}

  template<typename T>
  bool operator()(
    const T * T_eye1_object_param,
    const T * T_eye2_object_param,
    const T * T_hand_eye_param,
    T * residuals_ptr) const
  {
    using SE3 = Sophus::SE3<T>;

    // map the input parameters to Sophus types
    SE3 T_eye1_object = Eigen::Map<SE3 const>{T_eye1_object_param};
    SE3 T_eye2_object = Eigen::Map<SE3 const>{T_eye2_object_param};
    SE3 T_hand_eye = Eigen::Map<SE3 const>{T_hand_eye_param};

    SE3 T_eye2_eye1 = T_eye2_object * T_eye1_object.inverse();
    SE3 T_hand2_hand1 = T_world_hand2_m.cast<T>().inverse() *
      T_world_hand1_m.cast<T>();

    // // calculate the pose graph error for the hand
    // const SE3 T_eye2_hand1 = T_eye2_eye1 * T_hand_eye.inverse();
    // const SE3 T_eye2_hand2 = T_eye2_hand1 * T_hand2_hand1.inverse();

    // const SE3 err = T_eye2_hand2 * T_hand_eye;

    const SE3 err = T_eye2_eye1.inverse() *
      (T_hand_eye.inverse() * T_hand2_hand1 * T_hand_eye);

    //map the residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals{residuals_ptr};
    residuals = err.log();

    return true;
  }

  static ceres::CostFunction * Create(
    const Sophus::SE3d & T_world_hand1_m,
    const Sophus::SE3d & T_world_hand2_m)
  {
    return new ceres::AutoDiffCostFunction<CameraCameraCostFucntion, 6,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters>(
      new CameraCameraCostFucntion(T_world_hand1_m, T_world_hand2_m));
  }

  const Sophus::SE3d T_world_hand1_m;
  const Sophus::SE3d T_world_hand2_m;
};

}

#endif
