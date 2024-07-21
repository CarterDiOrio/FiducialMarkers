#include "comparator/extrinsic_observation.hpp"
#include <nlohmann/json.hpp>
#include "comparator/eigen_json.hpp"
#include <iostream>

void to_json(json & j, const ExtrinsicObservation & value)
{
  j = json{
    {"T_world_mount", value.T_world_mount},
    {"T_world_cmount", value.T_world_cmount},
    {"chessboard_observation", value.chessboard_observations},
  };
}

void from_json(const json & j, ExtrinsicObservation & p)
{
  j.at("T_world_mount").get_to(p.T_world_mount);
  j.at("T_world_cmount").get_to(p.T_world_cmount);
  j.at("chessboard_observation").get_to(p.chessboard_observations);
}

void to_json(json & j, const ExtrinsicObservations & value)
{
  j = json{
    {"observations", value.observations}
  };
}

void from_json(const json & j, ExtrinsicObservations & p)
{
  j.at("observations").get_to(p.observations);
}
