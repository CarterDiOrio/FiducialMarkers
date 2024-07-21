#include "pi_eink/april_tags.hpp"
#include "pi_eink/epaper.hpp"
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <string.h>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

namespace pi_eink::april_tags
{

AprilTagFamilies from_string(const std::string & str)
{
  if (str == "tag16h5") {
    return FAMILY_16_H_5;
  } else if (str == "tag25h9") {
    return FAMILY_25_H_9;
  } else if (str == "tag36h11") {
    return FAMILY_36_H_11;
  } else {
    throw std::runtime_error("Unknown family: " + str);
  }
}

AprilTagDataSource::AprilTagDataSource(
  std::string apriltag_imgs_location)
: apriltag_imgs_location{apriltag_imgs_location}
{}

cv::Mat AprilTagDataSource::load_april_tag(
  AprilTagFamilies family,
  int id) const
{
  std::string family_str;
  std::string family_filename;
  switch (family) {
    case FAMILY_16_H_5:
      family_str = "tag16h5";
      family_filename = "tag16_05_";
      break;
    case FAMILY_25_H_9:
      family_str = "tag25h9";
      family_filename = "tag25_09_";
      break;
    case FAMILY_36_H_11:
      family_str = "tag36h11";
      family_filename = "tag36_11_";
      break;
  }

  std::stringstream id_ss;
  id_ss << std::setw(5) << std::setfill('0') << id;
  const auto filename = family_filename + id_ss.str() + ".png";
  const auto full_path = apriltag_imgs_location / family_str / filename;
  return cv::imread(full_path.string(), cv::IMREAD_GRAYSCALE);
}

std::pair<pi_eink::Image, AprilTagInfo> draw_april_tag(
  const AprilTagDataSource & data_source,
  const EPaper & display,
  double desired_square_size,
  AprilTagFamilies family,
  int id
)
{
  const auto px_per_square = closest_pixel_size(display, desired_square_size);
  const auto actual_size = pixels_to_mm(display, px_per_square);
  const auto tag = data_source.load_april_tag(family, id);

  auto image = image_from_display(display);

  const size_t width = tag.cols;
  const size_t height = tag.rows;

  const size_t width_px = width * px_per_square;
  const size_t height_px = height * px_per_square;

  const size_t center_x_px = display.get_panel_width() / 2;
  const size_t center_y_px = display.get_panel_height() / 2;

  const size_t half_width_px = std::round(width_px / 2.0);
  const size_t half_height_px = std::round(height_px / 2.0);

  const size_t top_left_px = center_x_px - half_width_px;
  const size_t top_right_px = center_y_px - half_height_px;

  // draw tag
  for (size_t y = 0; y < height; y++) {
    for (size_t x = 0; x < width; x++) {
      const auto value = tag.at<uchar>(y, x);
      const auto x_px = top_left_px + x * px_per_square;
      const auto y_px = top_right_px + y * px_per_square;

      if (value > 0) {
        draw_square(image, x_px, y_px, px_per_square, 1.0);
      } else {
        draw_square(image, x_px, y_px, px_per_square, 0.0);
      }
    }
  }

  // calculate actual center position
  // this may differ from the actual center of the display because
  // the center of the tag may not fall along cell boundaries and rounding
  // to the nearest pixel
  const auto tag_center_x = top_left_px + half_width_px;
  const auto tag_center_y = top_right_px + half_height_px;

  // the center of the coordinate system we return is the center of the display
  const auto tag_center_x_centered = tag_center_x - center_x_px;
  const auto tag_center_y_centered = center_y_px - tag_center_y;

  return {
    image,
    AprilTagInfo{
      .size = actual_size,
      .x = pixels_to_mm(display, tag_center_x_centered),
      .y = pixels_to_mm(display, tag_center_y_centered)
    }
  };
}

}
