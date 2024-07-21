#ifndef INC_GUARD_PI_EINK_ARUCO_HPP
#define INC_GUARD_PI_EINK_ARUCO_HPP

#include "pi_eink/epaper.hpp"
#include <aruco/dictionary.h>
#include <aruco/posetracker.h>

namespace pi_eink::aruco_tags
{

struct ArucoTagInfo
{
  double size;
  double x;
  double y;
};

/// \brief Converts a string to an aruco dictionary type
/// \param str the string to convert
aruco::Dictionary::DICT_TYPES from_string(const std::string & str);
const std::map<std::string, aruco::Dictionary::DICT_TYPES> string_to_type = {
  {"aruco", aruco::Dictionary::ARUCO},
  {"aruco_mip_25h7", aruco::Dictionary::ARUCO_MIP_25h7},
  {"aruco_mip_36h12", aruco::Dictionary::ARUCO_MIP_36h12}
};

/// \brief Generates an aruco tag image
/// \param display the display to generate the tag for
/// \param dictionary_type the type of dictionary to use
/// \param id the id of the tag
/// \param desired_square_size the desired size of each square in mm
/// \return the generated image and the tag info
std::pair<pi_eink::Image, ArucoTagInfo> generate_tag(
  const EPaper & dispay,
  aruco::Dictionary::DICT_TYPES dictionary_type,
  int id,
  double desired_square_size
);

}


#endif
