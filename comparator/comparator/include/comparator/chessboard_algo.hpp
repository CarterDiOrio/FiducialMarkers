#ifndef INC_GUARD_CHESSBOARD_ALGO_HPP
#define INC_GUARD_CHESSBOARD_ALGO_HPP

#include "comparator/camera.hpp"
#include "comparator/measurement_algo_interface.hpp"
#include "comparator/chessboard.hpp"
#include "comparator/extrinsic_calibration.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <pi_eink/api.hpp>
#include <pi_eink/client.hpp>

/// @brief An algorithm implemenation for detecting the chessboard
class ChessboardAlgo : public MeasurementAlgoInterface
{
public:
  ChessboardAlgo(
    std::shared_ptr<CameraIntf> camera,
    std::shared_ptr<pi_eink::EinkClient> eink_client,
    size_t n_corners,
    double square_size
  ): camera{camera}, 
     eink_client{eink_client}, 
     n_corners{n_corners}, 
     desired_square_size{square_size}
  {};

  std::string get_name() override {
    return "chess_" + std::to_string(n_corners) + "_" + std::to_string(actual_square_size);
  }

  void display() override {
    // display the chessboard on the eink display
    const auto response = eink_client->draw(pi_eink::ChessboardRequest{
      .desired_square_size = desired_square_size,
      .num_corners = n_corners
    });

    corners.clear();
    for (const auto & corner : response.response->corners) {
      corners.push_back(cv::Point3f(corner.first, corner.second, 0));
    }

    actual_square_size = response.response->actual_square_size;
  }

  std::optional<Sophus::SE3d> detect() override {
    // get the current intrinsics from the camera
    const auto K = camera->get_intrinsics();

    // get image from the camera
    auto image = camera->get_frame();
    cv::Mat gray;
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // detect the chessboard
    auto obs = mrgingham_find_chessboard(image, n_corners, true);

    if (!obs.has_value()) {
      return std::nullopt;
    }

    std::vector<cv::Point2f> img_points;
    for (const auto & corner : obs.value().corners) {
      img_points.push_back(cv::Point2d{corner.img_point.x(), corner.img_point.y()});
    }

    // perform pnp
    Sophus::SE3d T_camera_chess = calibration::solve_pnp(
      corners,
      img_points,
      K
    );

    T_camera_chess.translation() /= 1000.0;
    
    return T_camera_chess;
  }

private:
  std::shared_ptr<CameraIntf> camera;
  std::shared_ptr<pi_eink::EinkClient> eink_client;
  size_t n_corners;
  double desired_square_size;
  double actual_square_size;

  std::vector<cv::Point3f> corners;
};


#endif