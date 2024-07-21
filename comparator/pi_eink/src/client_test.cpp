#include "pi_eink/client.hpp"
#include "pi_eink/api.hpp"
#include <iostream>

int main()
{
  std::cout << "Connecting to Display... \n" << std::endl;
  pi_eink::EinkClient client("169.254.100.135:8080");

  std::cout << "clearing" << std::endl;
  client.clear();

  std::cout << "Drawing Chessboard" << std::endl;

  pi_eink::ChessboardRequest chessboard_request;
  chessboard_request.desired_square_size = 10;
  chessboard_request.num_corners = 8;

  auto chessboard_response = client.draw(chessboard_request);

  std::cout << "clearing" << std::endl;
  client.clear();

  pi_eink::AprilTagRequest request;
  request.family = "tag36h11";
  request.id = 0;
  request.desired_square_size = 10;

  std::cout << " Drawing April Tag..." << std::endl;
  auto response = client.draw(request);

  const auto & tag = *response.response;

  std::cout << "Actual Square Size: " << tag.actual_square_size <<
    std::endl;

  std::cout << "clearing" << std::endl;
  client.clear();

  std::cout << "Drawing aurco" << std::endl;
  pi_eink::ArucoTagRequest aruco_request;
  aruco_request.dictionary_type = "aruco_mip_36h12";
  aruco_request.id = 0;
  aruco_request.desired_square_size = 10;

  auto aruco_response = client.draw(aruco_request);

  const auto & atag = *aruco_response.response;

  std::cout << "Actual Square Size: " << atag.actual_square_size <<
    std::endl;


  std::cout << "sleeping..." << std::endl;

  client.sleep();


  return 1;
}
