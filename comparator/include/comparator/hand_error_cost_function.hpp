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
    const Sophus::SE3d & T_world_hand_m,
    const Sophus::SE3d & T_world_mount_m
  )
  : T_world_hand_m{T_world_hand_m}, T_world_mount_m{T_world_mount_m} {}

  template<typename T>
  bool operator()(
    const T * T_eo,
    const T * T_he,
    const T * T_mo,
    T * residuals_ptr) const
  {
    // map the input parameters to Sophus types
    Sophus::SE3<T> T_eye_object = Eigen::Map<Sophus::SE3<T> const>{T_eo};
    Sophus::SE3<T> T_hand_eye = Eigen::Map<Sophus::SE3<T> const>{T_he};
    Sophus::SE3<T> T_mount_object = Eigen::Map<Sophus::SE3<T> const>{T_mo};

    // testing...
    const Sophus::SE3<T> T_eye_hand = T_hand_eye.inverse();
    const Sophus::SE3<T> T_hand_world = T_world_hand_m.cast<T>().inverse();
    const Sophus::SE3<T> T_eye_world = T_eye_hand * T_hand_world;

    const Sophus::SE3<T> T_world_object = T_world_mount_m *
      T_mount_object;

    const Sophus::SE3<T> e = T_eye_object.inverse() *
      (T_eye_world * T_world_object);
    Eigen::Vector<T, 6> e_ww = e.log();

    //map the residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals{residuals_ptr};

    residuals = e_ww;
    return true;
  }

  static ceres::CostFunction * Create(
    const Sophus::SE3d & T_world_hand_m,
    const Sophus::SE3d & T_world_mount_m)
  {
    return new ceres::AutoDiffCostFunction<HandErrorCostFunction, 6,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters,
             Sophus::SE3d::num_parameters>(
      new HandErrorCostFunction(T_world_hand_m, T_world_mount_m));
  }

  const Sophus::SE3d T_world_hand_m;
  const Sophus::SE3d T_world_mount_m;
};

}

#endif
