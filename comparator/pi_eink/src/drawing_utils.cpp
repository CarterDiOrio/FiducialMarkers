#include "pi_eink/drawing_utils.hpp"

namespace pi_eink
{
void draw_square(
  Image & image,
  size_t x, size_t y,
  size_t square_size,
  double value
)
{
  for (size_t i = 0; i < square_size; ++i) {
    for (size_t j = 0; j < square_size; ++j) {
      image.set_pixel(x + j, y + i, value);
    }
  }
}

}
