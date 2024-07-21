#include "pi_eink/chessboard.hpp"
#include "pi_eink/epaper.hpp"
#include "pi_eink/fiducial.hpp"
#include <cmath>
#include <cstddef>
#include <iostream>


namespace pi_eink
{


EinkFiducial create_chessboard(
  const EPaper & display,
  double desired_square_size,
  size_t num_squares_x,
  size_t num_squares_y)
{
  // the closest size in pixels
  const auto px_per_square = closest_pixel_size(display, desired_square_size);

  auto image = image_from_display(display);

  // the center of the display in pixel coordinates
  const size_t center_x_px = display.get_panel_width() / 2;
  const size_t center_y_px = display.get_panel_height() / 2;

  // half the size of the chessboard in pixels
  const auto half_x_px = std::round(px_per_square * num_squares_x / 2);
  const auto half_y_py = std::round(px_per_square * num_squares_y / 2);

  // the location of each internal corner in mm
  std::vector<std::pair<double, double>> points;

  for (size_t y = 0; y < num_squares_y; y++) {
    for (size_t x = 0; x < num_squares_x; x++) {
      const auto x_px = center_x_px - half_x_px + x * px_per_square;
      const auto y_px = center_y_px + half_y_py - y * px_per_square;

      if ((x + y) % 2 != 0) {
        draw_square(image, x_px, y_px, px_per_square, 1.0);
      } else {
        draw_square(image, x_px, y_px, px_per_square, 0.0);
      }

      if (y > 0 && x > 0) {
        points.push_back(
          {
            pixels_to_mm(display, -half_x_px + x * px_per_square),
            pixels_to_mm(display, half_y_py - y * px_per_square)
          });
      }
    }
  }

  return {
    .points = points,
    .image = image
  };
}

EinkFiducial create_gingham_chessboard(
  const EPaper & display,
  double desired_square_size,
  size_t num_corners)
{
  const auto num_squares = num_corners - 1;
  const auto padded_size = num_squares + 4;

  // the closest size in pixels
  const auto px_per_square = closest_pixel_size(display, desired_square_size);

  Image image(
    display.get_panel_width(),
    display.get_panel_height());

  // the center of the display in pixel coordinates
  const size_t center_x_px = display.get_panel_width() / 2;
  const size_t center_y_px = display.get_panel_height() / 2;

  std::cout << "center_x_px: " << center_x_px << std::endl;
  std::cout << "center_y_px: " << center_y_px << std::endl;

  // half the size of the chessboard in pixels
  const auto half_x_px = std::round(px_per_square * padded_size / 2);
  const auto half_y_py = std::round(px_per_square * padded_size / 2);

  // logic to determine if a square is black
  const auto is_black = [&padded_size](size_t c, size_t r) {
      if ((r < 2 || r >= padded_size - 2) && (c < 2 || c >= padded_size - 2)) {
        return false;
      }

      if (r < 2 || r >= padded_size - 2) {
        return c % 2 == 0;
      }

      if (c < 2 || c >= padded_size - 2) {
        return r % 2 == 0;
      }

      return (c + r) % 2 != 0;
    };

  std::vector<std::pair<double, double>> points;

  std::cout << "left: " << center_x_px - half_x_px << std::endl;
  std::cout << "right: " << center_x_px + half_x_px << std::endl;
  std::cout << "top: " << center_y_px - half_y_py << std::endl;
  std::cout << "bottom: " << center_y_px + half_y_py << std::endl;
  std::cout << "pps: " << px_per_square << std::endl;

  for (size_t y = 0; y < padded_size; y++) {
    for (size_t x = 0; x < padded_size; x++) {
      const auto x_px = center_x_px - half_x_px + x * px_per_square;
      const auto y_px = center_y_px - half_y_py + y * px_per_square;

      if (is_black(x, y)) {
        draw_square(image, x_px, y_px, px_per_square, 0.0);
      } else {
        draw_square(image, x_px, y_px, px_per_square, 1.0);
      }

      if (y >= 2 && x >= 2 && y <= padded_size - 2 && x <= padded_size - 2) {
        points.push_back(
          {
            pixels_to_mm(display, -half_x_px + x * px_per_square),
            pixels_to_mm(display, half_y_py - y * px_per_square)
          });
      }
    }
  }

  return {
    .points = points,
    .image = image,
    .square_size = pixels_to_mm(display, px_per_square)
  };
}

}
