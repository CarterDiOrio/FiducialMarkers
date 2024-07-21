/**
  * @file pi_eink/client.hpp
  * @brief Contains the client library for communicating the the eink http server
  *
  */

#ifndef INC_GUARD_CLIENT_HPP
#define INC_GUARD_CLIENT_HPP

#include "pi_eink/api.hpp"
#include <optional>


namespace pi_eink
{

/// @brief the client for the eink server
class EinkClient
{
public:
  enum Error
  {
    OK,
    NO_CONNECTION,
    NO_RESPONSE,
  };

  template<typename T>
  struct Response
  {
    Error error;
    std::optional<T> response;
  };

  EinkClient(const std::string & address);

  /// @brief Clears the display
  Error clear();

  /// @brief Puts the display to sleep
  Error sleep();

  /// @brief Draws a chessboard on the display
  /// @param request the request to draw the chessboard
  /// @return the response from the server
  Response<ChessboardResponse> draw(const ChessboardRequest & request);

  /// @brief Draws an april tag on the display
  /// @param request the request to draw the april tag
  /// @return the response from the server
  Response<AprilTagResponse> draw(const AprilTagRequest & request);

  /// @brief Draws an aruco tag on the display
  /// @param request the request to draw the aruco tag
  /// @return the response from the server
  Response<ArucoTagResponse> draw(const ArucoTagRequest & request);

private:
  const std::string address;
};

EinkClient::Error httperr_to_error(long status_code);

}

#endif
