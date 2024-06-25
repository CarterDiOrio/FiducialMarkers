#ifndef INC_GUARD_CHESSBOARD_DETECTOR_HPP
#define INC_GUARD_CHESSBOARD_DETECTOR_HPP

#include <mrgingham/mrgingham.hh>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <optional>

/// \brief Finds the corners of a chessboard in an image using mrgingham
/// \param img The image to find the chessboard in
/// \param gridn The number of corners in the chessboard
/// \param refine_corners Whether to refine the corners
/// \return The corners of the chessboard if found
std::optional<std::vector<Eigen::Vector2d>> mrgingham_find_chessboard(
  const cv::Mat & img,
  int gridn,
  bool refine_corners = true
);

#endif
