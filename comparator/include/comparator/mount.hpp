#ifndef INC_GUARD_MOUNT_HPP
#define INC_GUARD_MOUNT_HPP

#include <string>
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

  /// \brief The parameters of the fiducial
  std::string fiducial_parameters;

  /// \brief The corners of the fiducial
  std::vector<Eigen::Vector3d> fiducial_corners;
};

/// \brief Loads a mount from a file
/// \param file_path The path to the file to load the mount from
/// \return The mount
Mount load_mount(const std::string & file_path);

#endif
