/**
  * @file pi_eink/api.hpp
  * @brief Contains the API types for the HTTP protocol
  * All of the types here are deserializeable and serializeable to JSON
  */

#ifndef INC_GUARD_API_HPP
#define INC_GUARD_API_HPP

#include <nlohmann/json.hpp>

namespace pi_eink
{

/// @brief the request to draw a chessboard
struct ChessboardRequest
{
  /// @brief the desired square size in mm
  double desired_square_size;

  /// @brief the number of squares in the x direction
  size_t num_corners;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  ChessboardRequest, desired_square_size, num_corners);

/// @brief the response to a chessboard request
struct ChessboardResponse
{
  /// @brief the actual square size in mm
  double actual_square_size;

  /// @brief the locations of the corners in mm
  std::vector<std::pair<double, double>> corners;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  ChessboardResponse, actual_square_size, corners);

/// @brief the request to draw an april tag
struct AprilTagRequest
{
  /// @brief the family of the april tag
  std::string family;

  /// @brief the id of the april tag
  int id;

  /// @brief the desired square size in mm
  double desired_square_size;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  AprilTagRequest, family, id,
  desired_square_size);

/// @brief the response to an april tag request
struct AprilTagResponse
{
  /// @brief the actual square size in mm
  double actual_square_size;

  /// @brief the x of the april tag in mm relative to the center of the display
  double x;

  /// @brief the y of the april tag in mm relative to the center of the display
  double y;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AprilTagResponse, actual_square_size, x, y);

/// @brief the request to draw an aruco tag
struct ArucoTagRequest
{

  /// @brief the type of dictionary to use
  std::string dictionary_type;

  /// @brief the id of the aruco tag
  int id;

  /// @brief the desired square size in mm
  double desired_square_size;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  ArucoTagRequest, dictionary_type, id,
  desired_square_size);

struct ArucoTagResponse
{
  /// @brief the actual square size in mm
  double actual_square_size;

  /// @brief the x of the aruco tag in mm relative to the center of the display
  double x;

  /// @brief the y of the aruco tag in mm relative to the center of the display
  double y;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  ArucoTagResponse, actual_square_size, x, y);

}

#endif
