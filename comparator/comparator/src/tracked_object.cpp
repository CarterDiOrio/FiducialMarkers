#include "comparator/tracked_object.hpp"
#include <sophus/se3.hpp>
#include <iostream>

namespace vicon
{

std::optional<Sophus::SE3d> get_object_transform(
  const TrackedObject & obj,
  const ViconDataStreamSDK::CPP::Client & client)
{
  auto segment_global_translation = client.GetSegmentGlobalTranslation(
    obj.subject_name,
    obj.root_segment_name);

  if (segment_global_translation.Result !=
    ViconDataStreamSDK::CPP::Result::Success)
  {
    return {};
  }

  auto segment_global_rotation = client.GetSegmentGlobalRotationMatrix(
    obj.subject_name,
    obj.root_segment_name);

  if (segment_global_rotation.Result !=
    ViconDataStreamSDK::CPP::Result::Success)
  {
    return {};
  }

  // check if rotation is 0
  if (std::fabs(segment_global_rotation.Rotation[0]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[1]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[2]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[3]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[4]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[5]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[6]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[7]) < 1e-6 &&
    std::fabs(segment_global_rotation.Rotation[8]) < 1e-6)
  {
    return {};
  }

  // Eigen::Map<Eigen::Matrix3d> rotation(segment_global_rotation.Rotation);

  // map rotation array to rotation matrix
  Eigen::Matrix3d rotation = Eigen::Map<Eigen::Matrix<double, 3, 3>>(
    segment_global_rotation.Rotation);

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = rotation.transpose();
  pose.block<3, 1>(0, 3) = Eigen::Map<Eigen::Vector3d>(
    segment_global_translation.Translation);

  return Sophus::SE3d{pose};
}


}
