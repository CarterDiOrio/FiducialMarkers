#include "comparator/tracked_object.hpp"

namespace vicon
{

std::optional<Eigen::Matrix4d> get_object_transform(
  const TrackedObject & obj,
  const ViconDataStreamSDK::CPP::Client & client)
{
  auto segment_global_translation = client.GetSegmentGlobalTranslation(
    obj.subject_name,
    obj.root_segment_name);

  if (segment_global_translation.Result != ViconDataStreamSDK::CPP::Result::Success) {
    return {};
  }

  auto segment_global_rotation = client.GetSegmentGlobalRotationMatrix(
    obj.subject_name,
    obj.root_segment_name);

  if (segment_global_rotation.Result != ViconDataStreamSDK::CPP::Result::Success) {
    return {};
  }

  Eigen::Map<Eigen::Matrix3d> rotation(segment_global_rotation.Rotation);

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = rotation;
  pose.block<3, 1>(0, 3) = Eigen::Map<Eigen::Vector3d>(
    segment_global_translation.Translation);

  return pose;
}


}
