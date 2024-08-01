#ifndef INC_GUARD_NCAMPNP_HPP
#define INC_GUARD_NCAMPNP_HPP

#include <iostream>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

/// @brief a 3d point in the world frame and its projection in the image plane
struct PointPair {
  Eigen::Vector3d object;
  Eigen::Vector2d image;
};

struct Camera {
  Eigen::Matrix<double, 3, 4> K;
  Sophus::SE3d T_world_camera;
  std::vector<PointPair> points;
  double ppd;
};

class NCamPnPRefineProblem {
public:
  NCamPnPRefineProblem &operator|(Camera &&camera);
  Sophus::SE3d solve();

private:
  std::vector<Camera> cameras;
};

/// \brief Performs PnP using OpenCV's solvePnP
/// \param object_points The object points
/// \param image_points The image points
/// \param K The camera matrix
/// \return The transformation from the object to the camera's frame, T_co
Sophus::SE3d solve_pnp(const std::vector<cv::Point3d> &object_points,
                       const std::vector<cv::Point2d> &image_points,
                       const cv::Mat &K);

#endif