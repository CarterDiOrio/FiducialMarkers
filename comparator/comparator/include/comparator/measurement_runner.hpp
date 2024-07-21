#ifndef INC_GUARD_MEASUREMENT_RUNNER_HPP
#define INC_GUARD_MEASUREMENT_RUNNER_HPP

#include "comparator/measurement.hpp"
#include "comparator/tracked_object.hpp"
#include "comparator/vicon.hpp"
#include "comparator/measurement_algo_interface.hpp"
#include <DataStreamClient.h>
#include <concepts>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <pi_eink/client.hpp>
#include <string>
#include <unordered_map>




/// \brief Handles the registering and running of measurements
class MeasurementRunner
{
public:
  /// \brief Constructor
  /// \param measurements_per_algo the number of measurements to take per algo
  /// \param num_vicon_pose_samples the number of vicon poses to average
  MeasurementRunner(
    size_t measurements_per_algo,
    size_t num_vicon_pose_samples,
    std::optional<std::unique_ptr<datastream::Client>> vicon_client,
    std::shared_ptr<pi_eink::EinkClient> eink_client,
    const vicon::TrackedObject & hand,
    const vicon::TrackedObject & mount
  );

  template<typename T>
  requires std::derived_from<T, MeasurementAlgoInterface>
  MeasurementRunner & register_measurement()
  {
    register_measurement(T{});
    return *this;
  }

  MeasurementRunner & register_measurement(
    std::unique_ptr<MeasurementAlgoInterface> algo)
  {
    algos.push_back(std::move(algo));
    return *this;
  }

  template<typename T, typename ... Args>
  requires std::derived_from<T, MeasurementAlgoInterface>
  MeasurementRunner & register_measurement(Args &&... args)
  {
    algos.push_back(std::make_unique<T>(std::forward<Args>(args)...));
    return *this;
  }

  /// \brief Runs the measurements
  /// \param set_prefix the prefix for the group of measurements
  void run(std::string set_prefix="");

  inline const std::vector<MeasurementSet> & get_measurement_sets() const
  {
    return measurement_sets;
  }

private:
  /// \brief the number of measurements to take with each algorithm and fiducial
  size_t measurements_per_algo;

  /// \brief the number of vicon pose to average to arrive at a more
  /// accurate world poses
  size_t num_vicon_pose_samples;

  std::optional<std::unique_ptr<datastream::Client>> vicon_client;

  std::shared_ptr<pi_eink::EinkClient> eink_client;

  /// \brief the tracked object for the hand
  vicon::TrackedObject hand;

  /// \brief the tracked object for the mount
  vicon::TrackedObject mount;

  /// \brief the algorithms to run
  std::vector<std::unique_ptr<MeasurementAlgoInterface>> algos;

  /// \brief the sets of measurements taken
  std::vector<MeasurementSet> measurement_sets;

  /// \brief Takes the measurements for each algorithm
  std::vector<Measurement> take_measurements();

  /// \brief Takes the vicon poses for the hand and mount
  std::pair<Sophus::SE3d, Sophus::SE3d> gather_vicon_poses();

};


#endif
