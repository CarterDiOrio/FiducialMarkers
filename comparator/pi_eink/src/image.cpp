#include "pi_eink/image.hpp"
#include <iostream>

namespace pi_eink
{

Image::Image(
  size_t width,
  size_t height,
  bool vertical_flip,
  bool horizontal_flip)
: width{width},
  height{height},
  vertical_flip{vertical_flip},
  horizontal_flip{horizontal_flip},
  data(width * height)
{
  // initialize the data to 0
  std::fill(data.begin(), data.end(), 1.0);
}

size_t Image::get_width() const
{
  return width;
}

size_t Image::get_height() const
{
  return height;
}

double Image::get_pixel(size_t x, size_t y) const
{
  if (vertical_flip) {
    y = height - y - 1;
  }

  if (horizontal_flip) {
    x = width - x - 1;
  }

  return data[y * width + x];
}

void Image::set_pixel(size_t x, size_t y, double value)
{
  if (vertical_flip) {
    y = height - y - 1;
  }

  if (horizontal_flip) {
    x = width - x - 1;
  }

  data[y * width + x] = value;
}

double Image::get_raw(size_t x, size_t y) const
{
  return data[y * width + x];
}

}
