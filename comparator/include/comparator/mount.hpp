#ifndef INC_GUARD_MOUNT_HPP
#define INC_GUARD_MOUNT_HPP

#include <opencv2/core.hpp>
#include <Eigen/Dense>


enum FiducialType
{
  ChessBoard
};

/// \brief Class modeling the fiducial mount
struct Mount
{
  /// \brief The transformation fiducial to the mount
  Eigen::Matrix4d T_mp;

  /// \brief The type of fiducial
  FiducialType fiducial_id;

  /// \brief The corners of the fiducial in the fiducials frame
  std::vector<Eigen::Vector3d> fiducial_corners;
};


/// \brief Creates a chessboard mount
/// \param T_mp The transformation from the fiducial to the mount (or an guess)
/// \param n_corners The number of corners in the chessboard
/// \param corner_spacing The spacing between the corners in meters
/// \return The created mount
Mount create_chessboard(
  const Eigen::Matrix4d & T_mp,
  size_t n_corners_x,
  size_t n_corners_y,
  double corner_spacing
);


#endif
