#include "pi_eink/epaper.hpp"
#include "Config/DEV_Config.h"
#include "e-Paper/EPD_IT8951.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <iostream>
#include <sys/types.h>

namespace pi_eink
{
EPaper::EPaper(
  double VCOM,
  double pixel_size,
  bool vertical_flip,
  bool horizontal_flip)
: pixel_size{pixel_size}, vertical_flip{vertical_flip},
  horizontal_flip{horizontal_flip}
{
  if (DEV_Module_Init() != 0) {
    throw std::runtime_error("Failed to initialize DEV_Module");
  }

  std::cout << "DEV_Module_Init success" << std::endl;


  // conversion taken from example code
  vcom = (UWORD)(std::fabs(VCOM) * 1000);

  std::cout << "VCOM Voltage: " << vcom << std::endl;

  // initialize the display
  dev_info = EPD_IT8951_Init(vcom);

  is_asleep = false;

  // pull useful information from the device info
  panel_width = dev_info.Panel_W;
  panel_height = dev_info.Panel_H;
  init_target_memory_addr = dev_info.Memory_Addr_L |
    (dev_info.Memory_Addr_H << 16);

  // this casting isn't great, but it's what the example code does
  // LUT_Version is stored as an array of uint16_ts
  // lut_version = (char *)dev_info.LUT_Version;
}

EPaper::~EPaper()
{
  std::cout << "Putting Display to sleep and exiting..." << std::endl;

  sleep();

  DEV_Delay_ms(5000);

  DEV_Module_Exit();
}


void EPaper::clear()
{
  if (is_asleep) {
    dev_info = EPD_IT8951_Init(vcom);
    is_asleep = false;
  }
  EPD_IT8951_Clear_Refresh(dev_info, init_target_memory_addr, Mode::INIT);
}

void EPaper::sleep()
{
  EPD_IT8951_Sleep();
  is_asleep = true;
}

void EPaper::standby()
{
  EPD_IT8951_Standby();
}

void EPaper::display_bytes(
  std::vector<uint8_t> & image_data,
  uint16_t x,
  uint16_t y,
  uint16_t width,
  uint16_t height,
  BytesPerPixel bpp,
  Mode mode)
{
  if (is_asleep) {
    dev_info = EPD_IT8951_Init(vcom);
    is_asleep = false;
  }

  switch (bpp) {
    case BytesPerPixel::ONE:
      EPD_IT8951_1bp_Refresh(
        image_data.data(),
        x,
        y,
        width,
        height,
        mode,
        init_target_memory_addr,
        true);
      break;
    default:
      throw std::runtime_error("Unsupported BytesPerPixel");
  }
}


std::vector<uint8_t> render_1bbp(
  const Image & image
)
{
  std::vector<uint8_t> data;

  const auto size = image.get_width() * image.get_height();

  for (size_t idx = 0; idx < size; idx += 8) {

    uint8_t byte = 0;

    for (size_t bit = 0; bit < 8; ++bit) {
      const auto idx2 = idx + bit;
      const auto x = idx2 % image.get_width();
      const size_t y = idx2 / image.get_width();

      const auto pixel = image.get_raw(x, y);

      // might need to invert this
      byte |= (pixel > 0.5) << bit;
    }

    data.push_back(byte);

  }

  return data;
}

Image image_from_display(const EPaper & display)
{
  return Image(
    display.get_panel_width(),
    display.get_panel_height(),
    display.is_vertical_flip(),
    display.is_horizontal_flip());
}

std::vector<uint8_t> render_image(
  const Image & image,
  EPaper::BytesPerPixel bpp
)
{
  switch (bpp) {
    case EPaper::BytesPerPixel::ONE:
      return render_1bbp(image);
    default:
      throw std::runtime_error("Unsupported BytesPerPixel");
  }

  return {};
}

size_t mm_to_pixels(
  const EPaper & display,
  double mm
)
{
  return mm / display.get_pixel_size();
}

double pixels_to_mm(
  const EPaper & display,
  double pixels
)
{
  return pixels * display.get_pixel_size();
}

size_t closest_pixel_size(
  const EPaper & display,
  double mm
)
{
  return std::round(mm / display.get_pixel_size());
}

void display_image(
  EPaper & display,
  const Image & image,
  DisplayImageOptions options)
{
  // render the image to a vector of bytes
  auto data = render_image(image, options.bpp);

  // display the image
  display.display_bytes(
    data,
    options.x,
    options.y,
    image.get_width(),
    image.get_height(),
    options.bpp,
    options.mode);
}

}
