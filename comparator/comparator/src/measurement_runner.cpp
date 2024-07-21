#include "comparator/measurement_runner.hpp"
#include "comparator/measurement.hpp"
#include "comparator/tracked_object.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <future>
#include <sophus/average.hpp>

MeasurementRunner::MeasurementRunner(
  size_t measurements_per_algo,
  size_t num_vicon_pose_samples,
  std::optional<std::unique_ptr<datastream::Client>> vicon_client,
  std::shared_ptr<pi_eink::EinkClient> eink_client,
  const vicon::TrackedObject & hand,
  const vicon::TrackedObject & mount
)
: measurements_per_algo{measurements_per_algo},
  num_vicon_pose_samples{num_vicon_pose_samples},
  vicon_client{std::move(vicon_client)},
  eink_client{eink_client},
  hand{hand},
  mount{mount}
{}

void MeasurementRunner::run(std::string set_prefix)
{
  // two tasks are needing to be done simulatenously to save time:
  // 1) taking the requested number of vicon poses for the hand and mount
  // 2) Taking the measurements for each algorithm

  auto measurement_fut = std::async(
    std::launch::async, [this]()
    {
      return this->take_measurements();
    });

  Sophus::SE3d T_world_hand{Eigen::Matrix4d::Identity()};
  Sophus::SE3d T_world_mount{Eigen::Matrix4d::Identity()};

  std::future<std::pair<Sophus::SE3d, Sophus::SE3d>> vicon_fut;

  if (vicon_client.has_value()) {
    vicon_fut = std::async(
      std::launch::async, [this]()
      {
        return this->gather_vicon_poses();
      });
  }

  const auto measurements = measurement_fut.get();
  
  if (vicon_client.has_value()) {
    const auto & [T_wh, T_wm] = vicon_fut.get();
    T_world_hand = T_wh;
    T_world_mount = T_wm;
  }

  measurement_sets.push_back(
    MeasurementSet{
    set_prefix,
    T_world_hand,
    T_world_mount,
    measurements
  }
  );

}

std::vector<Measurement> MeasurementRunner::take_measurements()
{
  std::vector<Measurement> measurements;

  for (const auto & algo: algos) {
    // display fiducial for the algorithm
    eink_client->clear();
    algo->display();
    // eink_client->sleep();
    std::cout << "Displaying fiducial for " << algo->get_name() << std::endl;

    // take the requested number of measurements
    std::vector<Sophus::SE3d> poses;
    for (size_t i = 0; i < measurements_per_algo; ++i) {

      const auto maybe_pose = algo->detect();
      if (!maybe_pose.has_value()) {
        std::cout << "Failed to detect pose" << std::endl;
        continue;
      }

      const auto pose = maybe_pose.value();

      if (i % 100 == 0){
        std::cout << "Pose: " << pose.translation().transpose() << std::endl;
      }

      poses.push_back(pose);
    }

    measurements.emplace_back(
      algo->get_name(),
      "", //TODO: add way to save image
      poses
    );
  }

  return measurements;
}

std::pair<Sophus::SE3d, Sophus::SE3d> MeasurementRunner::gather_vicon_poses()
{
  std::vector<Sophus::SE3d> hand_poses;
  std::vector<Sophus::SE3d> mount_poses;

  for (size_t i = 0; i < num_vicon_pose_samples; ++i) {
    if (i % 100 == 0) {
      std::cout << "Taking vicon pose " << i << "/" << num_vicon_pose_samples << std::endl;
    }
    
    vicon_client.value()->GetFrame();

    // TODO: add error handling
    const auto T_hw_maybe = vicon::get_object_transform(hand, *(vicon_client.value()));
    const auto T_mw_maybe = vicon::get_object_transform(mount, *(vicon_client.value()));

    if (!T_hw_maybe.has_value() || !T_mw_maybe.has_value()) {
      std::cout << "Failed to get vicon pose" << std::endl;
      continue;
    }

    const auto T_hw = T_hw_maybe.value();
    const auto T_mw = T_mw_maybe.value();

    hand_poses.push_back(T_hw);
    mount_poses.push_back(T_mw);
  }

  // TODO: add error handling
  Sophus::SE3d average_hand_pose = *Sophus::average(hand_poses);
  Sophus::SE3d average_mount_pose = *Sophus::average(mount_poses);

  return {average_hand_pose, average_mount_pose};
}
