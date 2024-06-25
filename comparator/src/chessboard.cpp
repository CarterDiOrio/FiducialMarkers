#include "comparator/chessboard.hpp"
#include <mrgingham/mrgingham.hh>

std::optional<std::vector<Eigen::Vector2d>> mrgingham_find_chessboard(
  const cv::Mat & img,
  int gridn,
  bool refine_corners
)
{
  // setting up corner refinement buffer
  signed char * refinement_level = NULL;

  std::vector<mrgingham::PointDouble> points_out;
  auto found_pyramid_level = mrgingham::find_chessboard_from_image_array(
    points_out,
    refine_corners ? &refinement_level : NULL,
    gridn,
    img);

  if (found_pyramid_level < 0) {
    return {};
  }

  // making sure to free the refinement buffer
  free(refinement_level);

  // convert mrgingham points to eigen vectors
  std::vector<Eigen::Vector2d> corners;
  for (const auto & point : points_out) {
    corners.push_back(Eigen::Vector2d(point.x, point.y));
  }

  return corners;
}
