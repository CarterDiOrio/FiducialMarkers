#ifndef INC_GUARD_EINK_HPP
#define INC_GUARD_EINK_HPP

#include "pi_eink/api.hpp"
#include <boost/filesystem/operations.hpp>
#include <cstdio>
#include <boost/process.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <format>
#include <nlohmann/json.hpp>
#include <concepts>

template<typename Request>
concept EinkRequest = std::is_same_v<Request, pi_eink::ChessboardRequest>||
  std::is_same_v<Request, pi_eink::AprilTagRequest>||
  std::is_same_v<Request, pi_eink::AprilTagRequest>;


template<EinkRequest Request, typename Response>
inline Response draw_on_eink(const Request request, const std::string & address)
{
  const auto in_file_name = boost::filesystem::temp_directory_path() /
    boost::filesystem::unique_path();
  const auto out_file_name = boost::filesystem::temp_directory_path() /
    boost::filesystem::unique_path();

  std::ofstream in_file(in_file_name.string());
  in_file << request;

  const std::string type = []() {
      if constexpr (std::is_same_v<Request, pi_eink::ChessboardRequest>) {
        return "chess";
      } else if constexpr (std::is_same_v<Request, pi_eink::AprilTagRequest>) {
        return "april";
      } else if constexpr (std::is_same_v<Request, pi_eink::ArucoTagRequest>) {
        return "aruco";
      }
    }();

  const auto command = std::format(
    "eink_cli --address {} --type {} --input {} --output {}",
    address,
    type,
    in_file_name.string(),
    out_file_name.string());

  boost::process::system(command); //TODO: Add error handling

  std::ifstream out_file(out_file_name.string());
  const auto json = nlohmann::json::parse(out_file);
  const auto response = json.template get<Response>();
  return response;
}

#endif
