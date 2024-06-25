#ifndef INC_GUARD_LINK_HPP
#define INC_GUARD_LINK_HPP

#include <string>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <optional>
#include "DataStreamClient.h"
#include <sophus/se3.hpp>

namespace vicon
{

struct TrackedObject
{
  std::string subject_name;
  std::string root_segment_name;
};

/// \brief Gets the SE3 matrix representing the pose of the object
/// \param obj The object to get the pose of
/// \param client The vicon client
/// \return The pose of the object
std::optional<Sophus::SE3d> get_object_transform(
  const TrackedObject & obj,
  const ViconDataStreamSDK::CPP::Client & client);

}


#endif
