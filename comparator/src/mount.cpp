#include "comparator/mount.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<std::string> tok(const std::string & line, char delim = ',')
{
  std::vector<std::string> tokens;
  std::istringstream stream(line);
  std::string token;

  while (std::getline(stream, token, delim)) {
    tokens.push_back(token);
  }

  return tokens;
}

Mount load_mount(const std::string & file_path)
{
  std::vector<std::string> lines;

  {
    std::ifstream file(file_path);
    while (file.good()) {
      std::string line;
      std::getline(file, line);
      lines.push_back(line);
    }
  }

  std::cout << "Read mount file..." << std::endl;

  // parse the transform from the file
  Eigen::Matrix4d T_mp;
  const auto & float_tokens = tok(lines[0]);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T_mp(i, j) = std::stof(float_tokens[i * 4 + j]);
    }
  }

  std::cout << "Parsed transform..." << std::endl;

  // parse the fiducial type from the file
  FiducialType fiducial_id;
  if (lines[1] == "ChessBoard") {
    fiducial_id = FiducialType::ChessBoard;
  } else {
    throw std::runtime_error("Unknown fiducial type");
  }

  // parse the fiducial parameters from the file
  std::string fiducial_parameters = lines[2];

  std::cout << "Parsed fiducial type and parameters..." << std::endl;

  // parse the fiducial corners from the file
  std::vector<Eigen::Vector3d> fiducial_corners;
  for (size_t i = 3; i < lines.size(); i++) {
    const auto & tokens = tok(lines[i]);
    fiducial_corners.push_back({std::stof(tokens[0]), std::stof(tokens[1]), 0});
  }

  std::cout << "Parsed fiducial corners..." << std::endl;

  return Mount{
    .T_mp = T_mp,
    .fiducial_id = fiducial_id,
    .fiducial_parameters = fiducial_parameters,
    .fiducial_corners = fiducial_corners
  };
}
