#ifndef INC_GUARD_EXTRINSIC_CERES_COST_FUNCTION_HPP
#define INC_GUARD_EXTRINSIC_CERES_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

namespace calibration
{

struct ExtrinsicsCostFunction
{
  /// \brief Constructor for the cost function
  /// \param observed_point The observed point in the image
  /// \param fiducial_point The fiducial point in the world
  /// \param T_world_mount The transformation from the world to the mount
  /// \param T_world_x The transformation from x frame to the world.
  /// (x can be) the world, mount, or hand depending on the calibration. If
  /// its the world, set this to be the Identity Matrix. The optimization for
  /// the camera's position happens in the x frame.
  ExtrinsicsCostFunction(
    const Eigen::Vector2d & observed_point,
    const Eigen::Vector3d & fiducial_point,
    const Eigen::Matrix4d & T_world_mount,
    const Eigen::Matrix4d & T_world_x,
    const Eigen::Matrix3d & K)
  : observed_point{observed_point},
    fiducial_point{fiducial_point},
    T_world_mount{T_world_mount},
    T_world_x{T_world_x},
    K{K} {}

  /// \brief The operator() function for the cost function
  /// \param T_mf The transformation from the mount to the fiducial
  /// \param T_xc The transformation from the camera to the x frame (x can be)
  /// the world, mount, or hand depending on the calibration.
  /// \param point_fiducial The fiducial corner point
  /// \param residuals The residuals to be calculated
  template<typename T>
  bool operator()(
    const T * const T_mf,
    const T * const T_xc,
    T * residuals) const
  {
    // map the input parameters to Sophus types
    Sophus::SE3<T> T_mount_fiducial = Eigen::Map<Sophus::SE3<T> const>{T_mf};
    Sophus::SE3<T> T_x_camera = Eigen::Map<Sophus::SE3<T> const>{T_xc};

    // transform the fiducial point to the world
    Eigen::Vector<T,
      4> point_homogenous = fiducial_point.cast<T>().homogeneous();

    Eigen::Vector<T,
      4> p_world = T_world_mount.cast<T>() * T_mount_fiducial.matrix() *
      point_homogenous;

    // transform the point from the world to camera frames
    Eigen::Vector<T,
      4> p_camera = T_x_camera.inverse().matrix() *
      T_world_x.cast<T>().inverse() *
      p_world;

    // std::cout << "p_world:\n" << p_world << std::endl;
    // std::cout << "p_x: \n" << T_world_x.cast<T>().inverse() * p_world <<
    //   std::endl;
    // std::cout << "p_camera: \n" << p_camera << std::endl;

    // project the point onto the image plane
    Eigen::Vector<T, 2> p_image{
      (p_camera.x() / p_camera.z()) * K(0, 0) + K(0, 2),
      (p_camera.y() / p_camera.z()) * K(1, 1) + K(1, 2)
    };

    // calculate the residuals
    residuals[0] = (p_image.x() - T{observed_point.x()});
    residuals[1] = (p_image.y() - T{observed_point.y()});


    return true;
  }

  static ceres::CostFunction * Create(
    const Eigen::Vector2d & observed_point,
    const Eigen::Vector3d & fiducial_point,
    const Eigen::Matrix4d & T_world_mount,
    const Eigen::Matrix4d & T_world_x,
    const Eigen::Matrix3d & K)
  {
    return new ceres::AutoDiffCostFunction<ExtrinsicsCostFunction, 2,
             Sophus::Manifold<Sophus::SE3>::num_parameters,
             Sophus::Manifold<Sophus::SE3>::num_parameters>(
      new ExtrinsicsCostFunction(
        observed_point,
        fiducial_point,
        T_world_mount,
        T_world_x,
        K)
             );
  }

  const Eigen::Vector2d & observed_point;
  const Eigen::Vector3d & fiducial_point;
  const Eigen::Matrix4d & T_world_mount;
  const Eigen::Matrix4d & T_world_x;
  const Eigen::Matrix3d & K;
};

}


#endif
