#include "comparator/mount.hpp"
#include <vector>


Mount create_chessboard(
  const Eigen::Matrix4d & T_mp,
  size_t n_corners_x,
  size_t n_corners_y,
  double corner_spacing)
{
  double half_width = (n_corners_x) * corner_spacing / 2;
  double half_height = (n_corners_y) * corner_spacing / 2;

  std::vector<Eigen::Vector3d> corners;
  for (size_t y = 0; y < n_corners_y; y++) {
    for (size_t x = 0; x < n_corners_x; x++) {
      corners.push_back(
        Eigen::Vector3d(
          -half_width + x * corner_spacing,
          half_height - y * corner_spacing,
          0));
    }
  }

  return Mount{
    T_mp,
    FiducialType::ChessBoard,
    corners
  };
}

std::vector<Eigen::Vector3d> operator*(
  const Eigen::Matrix4d & T,
  const std::vector<Eigen::Vector3d> & points)
{
  std::vector<Eigen::Vector3d> transformed_points;
  for (const Eigen::Vector3d & point: points) {
    transformed_points.push_back(
      (T * point.homogeneous()).head<3>()
    );
  }
  return transformed_points;
}
