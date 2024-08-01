#ifndef INC_GUARD_MULTICAM_REFINE_COST_FUNCTION_HPP
#define INC_GUARD_MULTICAM_REFINE_COST_FUNCTION_HPP

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/jet.h>
#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

class RefinePnP {
public:
  RefinePnP(const Eigen::Vector3d &object_point,
            const Eigen::Vector2d &image_point,
            const Eigen::Matrix<double, 3, 4> &K,
            const Sophus::SE3d &T_world_camera, const double ppd)
      : object_point(object_point), image_point(image_point), K{K},
        T_world_camera(T_world_camera), ppd(ppd) {}

  template <typename T>
  bool operator()(const T *T_world_object_param, T *residuals) const {
    using SE3 = Sophus::SE3<T>;

    // map parameters to sophus types
    SE3 T_world_object = Eigen::Map<SE3 const>{T_world_object_param};

    // project
    Eigen::Vector4<T> point_camera = T_world_camera.cast<T>().inverse() *
                                     T_world_object *
                                     object_point.homogeneous().cast<T>();

    Eigen::Vector3<T> projected = K * point_camera;
    projected /= projected(2);

    // // compute residuals
    residuals[0] = (projected(0) - image_point(0)) * ppd;
    residuals[1] = (projected(1) - image_point(1)) * ppd;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &object_point,
                                     const Eigen::Vector2d &image_point,
                                     const Eigen::Matrix<double, 3, 4> &K,
                                     const Sophus::SE3d &T_world_camera,
                                     const double ppd) {
    return new ceres::AutoDiffCostFunction<RefinePnP, 2,
                                           Sophus::SE3d::num_parameters>(
        new RefinePnP(object_point, image_point, K, T_world_camera, ppd));
  }

private:
  Eigen::Vector3d object_point;
  Eigen::Vector2d image_point;
  Eigen::Matrix<double, 3, 4> K;
  Sophus::SE3d T_world_camera;
  double ppd;
};

#endif