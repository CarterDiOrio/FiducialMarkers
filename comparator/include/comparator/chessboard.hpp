#ifndef INC_GUARD_CHESSBOARD_DETECTOR_HPP
#define INC_GUARD_CHESSBOARD_DETECTOR_HPP

#include <mrgingham/mrgingham.hh>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <optional>
#include <nlohmann/json_fwd.hpp>

struct CornerObservation
{
  Eigen::Vector2d img_point;
  int level;
};

struct ChessboardObservation
{
  std::vector<CornerObservation> corners;
};

/// \brief Converts a CornerObservation to json
/// \param j The json object to convert to
/// \param obs The CornerObservation to convert
void to_json(nlohmann::json & j, const CornerObservation & obs);

/// \brief Parses a CornerObservation from json
/// \param j The json object to parse from
/// \param obs The CornerObservation to parse into
void from_json(const nlohmann::json & j, CornerObservation & obs);

/// \brief Converts a ChessboardObservation to json
/// \param j The json object to convert to
/// \param obs The ChessboardObservation to convert
void to_json(nlohmann::json & j, const ChessboardObservation & obs);

/// \brief Parses a ChessboardObservation from json
/// \param j The json object to parse from
/// \param obs The ChessboardObservation to parse into
void from_json(const nlohmann::json & j, ChessboardObservation & obs);


/// \brief Finds the corners of a chessboard in an image using mrgingham
/// \param img The image to find the chessboard in
/// \param gridn The number of corners in the chessboard
/// \param refine_corners Whether to refine the corners
/// \return The corners of the chessboard if found
std::optional<ChessboardObservation> mrgingham_find_chessboard(
  const cv::Mat & img,
  int gridn,
  bool refine_corners = true
);

#endif
