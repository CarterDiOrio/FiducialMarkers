#include "comparator/chessboard.hpp"
#include <mrgingham/mrgingham.hh>
#include <opencv2/core/base.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <optional>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

std::optional<std::vector<Eigen::Vector2d>> mrgingham_find_chessboard(
  const cv::Mat & img,
  int gridn,
  bool refine_corners
)
{

  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(4);

  const cv::Mat processed_image = [&img, &clahe]() {
      // if image is not CV_8U, convert it, this is a limiation of mrgingham
      cv::Mat pre_img = img.clone();
      cv::Mat processed_image;

      if (img.depth() == CV_8U) {
        cv::normalize(pre_img, pre_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        clahe->apply(pre_img, processed_image);
      } else if (img.depth() == CV_16U) {
        // cv::normalize(pre_img, pre_img, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
        clahe->apply(pre_img, pre_img);
        // pre_img.convertTo(processed_image, CV_8U, 255. / 65535.);
      } else {
        throw std::runtime_error(
                "Unsupported image depth only supports CV_8U and CV_16U");
      }
      // blur the image
      cv::blur(
        processed_image, processed_image,
        cv::Size{1, 1});

      return processed_image;
    } ();

  // setting up corner refinement buffer
  signed char * refinement_level = NULL;

  std::vector<mrgingham::PointDouble> points_out;
  auto found_pyramid_level = mrgingham::find_chessboard_from_image_array(
    points_out,
    refine_corners ? &refinement_level : NULL,
    gridn,
    processed_image);

  if (found_pyramid_level < 0) {
    return std::nullopt;
  }

  // making sure to free the refinement buffer
  // mrgingham internally allocates this buffer
  free(refinement_level);

  // convert mrgingham points to eigen vectors
  std::vector<Eigen::Vector2d> corners;
  for (const auto & point : points_out) {
    corners.push_back(Eigen::Vector2d(point.x, point.y));
  }

  return corners;
}
