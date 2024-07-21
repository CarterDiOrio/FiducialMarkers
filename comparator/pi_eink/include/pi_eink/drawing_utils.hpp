#ifndef INC_GUARD_DRAWING_UTILS_HPP
#define INC_GUARD_DRAWING_UTILS_HPP

#include "pi_eink/image.hpp"

namespace pi_eink
{

void draw_square(
  Image & image,
  size_t x, size_t y,
  size_t square_size,
  double value
);

}

#endif
