#include "pi_eink/aruco.hpp"
#include "pi_eink/drawing_utils.hpp"
#include <aruco/dictionary.h>
#include <iostream>

namespace pi_eink::aruco_tags
{

aruco::Dictionary::DICT_TYPES from_string(const std::string & str)
{
  return string_to_type.at(str);
}

std::pair<pi_eink::Image, ArucoTagInfo> generate_tag(
  const EPaper & display,
  aruco::Dictionary::DICT_TYPES dictionary_type,
  int id,
  double desired_square_size
)
{
  // load aruco dictionary
  aruco::Dictionary dictionary =
    aruco::Dictionary::loadPredefined(dictionary_type);

  // we will scale this image up so make this 1 px per bit for now
  const auto tag = dictionary.getMarkerImage_id(id, 1, false, false);

  // create the image
  auto image = image_from_display(display);

  const auto px_per_square = closest_pixel_size(display, desired_square_size);
  const auto actual_size = pixels_to_mm(display, px_per_square);

  const size_t width = tag.cols;
  const size_t height = tag.rows;

  std::cout << "width: " << width << " height: " << height << std::endl;

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
    ArucoTagInfo{
      .size = actual_size,
      .x = pixels_to_mm(display, tag_center_x_centered),
      .y = pixels_to_mm(display, tag_center_y_centered)
    }
  };
}

}
