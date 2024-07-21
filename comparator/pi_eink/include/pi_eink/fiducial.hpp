#ifndef INC_GUARD_FIDUCIAL_HPP
#define INC_GUARD_FIDUCIAL_HPP

#include "pi_eink/image.hpp"
namespace pi_eink
{
struct EinkFiducial
{
  /// \brief The relevant "points" of the fiducial in mm from the center
  std::vector<std::pair<double, double>> points;

  /// \brief The image that will be displayed
  Image image;

  double square_size;
};
}

#endif
