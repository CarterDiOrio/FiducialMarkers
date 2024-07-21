#include "pi_eink/client.hpp"
#include "pi_eink/api.hpp"
#include <cpr/cpr.h>

#include <cpr/response.h>
#include <nlohmann/json.hpp>
#include <optional>
#include <iostream>

namespace pi_eink
{

EinkClient::EinkClient(const std::string & address)
: address{address}
{}

template<typename Response, typename Request>
EinkClient::Response<Response> json_post(
  const std::string & address,
  const std::string & handle,
  const Request & request)
{
  nlohmann::json j = request;

  const auto res = cpr::Post(
    cpr::Url{address + handle},
    cpr::Header{{"Content-Type", "application/json"}},
    cpr::Body{j.dump()}
  );

  std::cout << j.dump() << std::endl;

  std::cout << res.status_code << std::endl;
  std::cout << res.text << std::endl;

  const auto err_code = httperr_to_error(res.status_code);
  if (err_code != EinkClient::Error::OK) {
    return {
      .error = err_code,
      .response = std::nullopt
    };
  }

  const auto body = nlohmann::json::parse(res.text);
  return {
    .error = err_code,
    .response = body.template get<Response>()
  };
}


EinkClient::Error EinkClient::clear()
{
  const auto res = cpr::Post(cpr::Url{address + "/clear"});
  return httperr_to_error(res.status_code);
}

EinkClient::Error EinkClient::sleep()
{
  const auto res = cpr::Post(cpr::Url{address + "/sleep"});
  return httperr_to_error(res.status_code);
}


EinkClient::Response<ChessboardResponse> EinkClient::draw(
  const ChessboardRequest & request
)
{
  return json_post<ChessboardResponse, ChessboardRequest>(
    address,
    "/chessboard",
    request);
}


EinkClient::Response<AprilTagResponse> EinkClient::draw(
  const AprilTagRequest & request
)
{
  return json_post<AprilTagResponse, AprilTagRequest>(
    address,
    "/april_tag",
    request);
}

EinkClient::Response<ArucoTagResponse> EinkClient::draw(
  const ArucoTagRequest & request
)
{
  return json_post<ArucoTagResponse, ArucoTagRequest>(
    address,
    "/aruco_tag",
    request);
}

EinkClient::Error httperr_to_error(long status_code)
{
  switch (status_code) {
    case 200:
      return EinkClient::Error::OK;
    default:
      return EinkClient::Error::NO_RESPONSE;
  }
}


}
