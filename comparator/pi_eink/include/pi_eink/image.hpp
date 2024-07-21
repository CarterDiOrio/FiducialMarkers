#ifndef INC_GUARD_IMAGE_HPP
#define INC_GUARD_IMAGE_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pi_eink
{

/// \brief Holds binary image data
class Image
{
public:
  Image(
    size_t width,
    size_t height,
    bool vertical_flip = false,
    bool horizontal_flip = false);

  size_t get_width() const;
  size_t get_height() const;

  /// \brief Gets the value of a pixel at the given coordinates
  double get_pixel(size_t x, size_t y) const;

  /// \brief Sets the value of a pixel at the given coordinates
  void set_pixel(size_t x, size_t y, double value);

  /// \brief Gets the raw data without any flipping
  double get_raw(size_t x, size_t y) const;

private:
  size_t width;
  size_t height;
  bool vertical_flip;
  bool horizontal_flip;
  std::vector<double> data;
};

}

#endif
