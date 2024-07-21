#ifndef INC_GUARD_EPAPER_HPP
#define INC_GUARD_EPAPER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include "pi_eink/image.hpp"

extern "C"
{
  #include "e-Paper/EPD_IT8951.h"
  #include "Config/DEV_Config.h"
}

namespace pi_eink
{

/// \brief Class Wrapping the EPD_IT8951 library functionality for the RPi
/// Not All of the functionality is implemeneted.
class EPaper
{
public:
  enum BytesPerPixel
  {
    ONE = 1,
    TWO = 2,
    FOUR = 4,
    EIGHT = 8,
  };

  /// \brief The different modes the display can be in
  /// These numbers map to the values in the EPD_IT8951.h file so this enum
  /// can be used directly in the EPD_IT8951 functions
  enum Mode
  {
    INIT = 0,
    GC16 = 2,
    A2 = 6,
  };

  /// \brief Constructor
  /// \param VCOM The VCOM voltage to use, can be found on device.
  /// \param pixel_size The size of the pixels in mm
  EPaper(
    double VCOM,
    double pixel_size,
    bool vertical_flip = false,
    bool horizontal_flip = false);

  /// \brief Uses a custom deconstructor to shutoff the display properly
  ~EPaper();

  /// \brief Gets the size of the pixels in mm
  inline double get_pixel_size() const {return pixel_size;}

  /// \brief Gets the width of the panel
  inline uint16_t get_panel_width() const {return panel_width;}

  /// \brief Gets the height of the panel
  inline uint16_t get_panel_height() const {return panel_height;}

  /// \brief Gets if the display is in vertical flip mode
  inline bool is_vertical_flip() const {return vertical_flip;}

  /// \brief Gets if the display is in horizontal flip mode
  inline bool is_horizontal_flip() const {return horizontal_flip;}

  /// \brief Clears the display with a full refresh
  void clear();

  /// \brief sleeps the display
  void sleep();

  /// \brief puts the display into standby mode
  /// Warning: The display is still powered in this state
  void standby();

  /// \brief Writes the image data to the display
  /// \param image_data The image data to write to the display
  /// \param x The x coordinate to start writing the image data
  /// \param y The y coordinate to start writing the image data
  /// \param width The width of the image data
  /// \param height The height of the image data
  void display_bytes(
    std::vector<uint8_t> & image_data,
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    BytesPerPixel bpp,
    Mode mode);

private:
  uint16_t vcom;

  /// \brief The width of the panel
  uint16_t panel_width;

  /// \brief The height of the panel
  uint16_t panel_height;

  /// \brief The initial target memory address
  /// TODO: add use case
  uint32_t init_target_memory_addr;

  /// \brief The LUT version of the display
  std::string lut_version;

  /// \brief The device information
  IT8951_Dev_Info dev_info;

  bool is_asleep = true;

  /// \brief The pixel size of the display in mm
  double pixel_size;

  bool vertical_flip;

  bool horizontal_flip;
};


/// \brief Creates an image to match the display settings
Image image_from_display(
  const EPaper & display);

/// \brief Renders an image to a vector of bytes for the displays
/// \param image The image to render
/// \param bpp The bytes per pixel to render the image as
std::vector<uint8_t> render_image(
  const Image & image,
  EPaper::BytesPerPixel bpp
);

/// \brief Converts mm to pixels
size_t mm_to_pixels(
  const EPaper & display,
  double mm
);

/// \brief Converts pixels to mm
double pixels_to_mm(
  const EPaper & display,
  double pixels
);


/// \brief Gets the closest whole number of pixels to the given mm by rounding
/// \param display The display to get the pixel size from
/// \param mm The mm to convert to pixels
/// \return The closest whole number of pixels to the given mm
size_t closest_pixel_size(
  const EPaper & display,
  double mm
);


struct DisplayImageOptions
{
  size_t x = 0;
  size_t y = 0;
  EPaper::Mode mode = EPaper::Mode::A2;
  EPaper::BytesPerPixel bpp = EPaper::BytesPerPixel::ONE;
};

/// \brief Draws an image to the display
/// \param display The display to draw to
/// \param image The image to draw
/// \param x The x coordinate to draw the image
/// \param y The y coordinate to draw the image
/// \param mode The mode to draw the image in
/// \param bpp The bytes per pixel to draw the image as
void display_image(
  EPaper & display,
  const Image & image,
  DisplayImageOptions options = {}
);

}


#endif
