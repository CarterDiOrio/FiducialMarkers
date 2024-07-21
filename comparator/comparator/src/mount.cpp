#include "comparator/mount.hpp"
#include "comparator/chessboard.hpp"
#include <vector>
#include <iostream>



Mount create_chessboard(
  const Eigen::Matrix4d & T_mp,
  size_t n_corners_x,
  size_t n_corners_y,
  double corner_spacing)
{
  return Mount{
    T_mp,
    FiducialType::ChessBoard,
    create_chessboard_corners(n_corners_x, n_corners_y, corner_spacing)
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


