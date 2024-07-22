#ifndef INC_GUARD_EXTRINSIC_OBSERVATION_HPP
#define INC_GUARD_EXTRINSIC_OBSERVATION_HPP

#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>
#include "comparator/mount.hpp"
#include "comparator/chessboard.hpp"

using json = nlohmann::json;


/// \brief Struct containing a single observation under the vicon system.
struct ExtrinsicObservation
{
  /// \brief The transform from the mount to the world coordinates
  Eigen::Matrix4d T_world_mount;

  /// \brief The transform from the camera mount to the world coordinates
  Eigen::Matrix4d T_world_hand;

  /// \brief the observation of the chessboard in the image
  ChessboardObservation chessboard_observations;

  ExtrinsicObservation() = default;

  ExtrinsicObservation(
    const Eigen::Matrix4d & T_world_mount,
    const Eigen::Matrix4d & T_world_cmount,
    const ChessboardObservation & chessboard_observation
  )
  : T_world_mount(T_world_mount),
    T_world_hand(T_world_cmount),
    chessboard_observations(chessboard_observation) {}
};

struct ExtrinsicObservations
{
  /// \brief contains all the individual observations
  std::vector<ExtrinsicObservation> observations;
};

/// \brief Converts ExtrinsicObservation to json
/// \param j The json object to convert to
/// \param p The ExtrinsicObservation to convert
void to_json(json & j, const ExtrinsicObservation & p);

/// \brief Parses ExtrinsicObservation from json
/// \param j The json object to parse from
/// \param p The ExtrinsicObservation to parse into
void from_json(const json & j, ExtrinsicObservation & p);

/// \brief Converts ExtrinsicObservations to json
/// \param j The json object to convert to
/// \param p The ExtrinsicObservations to convert
void to_json(json & j, const ExtrinsicObservations & p);

/// \brief Parses ExtrinsicObservations from json
/// \param j The json object to parse from
/// \param p The ExtrinsicObservations to parse into
void from_json(const json & j, ExtrinsicObservations & p);

#endif
