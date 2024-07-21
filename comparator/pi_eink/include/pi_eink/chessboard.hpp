#ifndef INC_GUARD_CHESSBOARD_HPP
#define INC_GUARD_CHESSBOARD_HPP

#include "pi_eink/epaper.hpp"
#include "pi_eink/fiducial.hpp"
#include "pi_eink/drawing_utils.hpp"
#include <cstddef>

namespace pi_eink
{

/// \brief Creates a chessboard fiducial
/// The actual square size will be calculated as,
///   actual_square_size = round(desired_square_size / pixel_size) * pixel_size
/// so that the squares take up a whole number of pixels
/// \param display The display to create the fiducial for
/// \param desired_square_size The size of the squares in mm
/// \param num_squares_x The number of squares in the x direction
/// \param num_squares_y The number of squares in the y direction
/// \return The fiducial
EinkFiducial create_chessboard(
  const EPaper & display,
  double desired_square_size,
  size_t num_squares_x,
  size_t num_squares_y
);

EinkFiducial create_gingham_chessboard(
  const EPaper & display,
  double desired_square_size,
  size_t num_corners
);


}

#endif
