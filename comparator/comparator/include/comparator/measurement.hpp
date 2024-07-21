#ifndef INC_GUARD_MEASUREMENT_HPP
#define INC_GUARD_MEASUREMENT_HPP

#include <nlohmann/detail/macro_scope.hpp>
#include <sophus/se3.hpp>
#include <string>
#include "comparator/eigen_json.hpp"
#include "comparator/extrinsic_calibration.hpp"

struct Measurement
{
  /// \brief the name of the algorithm that generated the measurement
  std::string algorithm_name;

  /// \brief the path to the image that was used to generate the measurement
  /// Only saves the first image
  std::string image_path;

  /// \brief the transform between the fiducial and the eye/camera
  std::vector<Sophus::SE3d> eye_fiducial_transforms;
};

/// \brief Contains one set of measurements at a common location
struct MeasurementSet
{
  /// \brief the prefix for the measurements
  /// Could contain additional information about resolution, groupings, etc...
  std::string prefix{""};

  /// \brief the transform from the hand to the world.
  /// Hand is what the camera/eye is attached to
  Sophus::SE3d T_world_hand;

  /// \brief the transform from the mount to the world
  /// The Mount is what the fiducial is attached to
  Sophus::SE3d T_world_mount;

  /// \brief the measurements at this location
  std::vector<Measurement> measurements;
};

/// \brief the file that contains the measurements
struct MeasurementFile
{
  /// \brief the extrinsics of the system
  /// The hand eye and mount fiducial transforms: T_he, T_mf
  calibration::ExtrinsicCalibration extrinsics;

  /// \brief the measurement sets
  std::vector<MeasurementSet> measurement_sets;
};

// Macros for json serialization
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  Measurement, algorithm_name, image_path,
  eye_fiducial_transforms)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  MeasurementSet, prefix, T_world_hand, T_world_mount, measurements)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  MeasurementFile, extrinsics, measurement_sets)


#endif
