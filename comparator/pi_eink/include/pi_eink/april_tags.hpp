#ifndef INC_GUARD_EINK_APRIL_TAGS_HPP
#define INC_GUARD_EINK_APRIL_TAGS_HPP

#include <nlohmann/detail/macro_scope.hpp>
#include <opencv2/core.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "pi_eink/epaper.hpp"
#include "pi_eink/image.hpp"
#include "pi_eink/drawing_utils.hpp"

namespace pi_eink::april_tags
{


/// \brief Contains the information of a drawn sapril tag
struct AprilTagInfo
{
  /// \brief the actual size of each april tag cell in mm
  double size;

  /// \brief the x y position of the april tag in mm
  double x;
  double y;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AprilTagInfo, size, x, y);

enum AprilTagFamilies
{
  FAMILY_16_H_5,
  FAMILY_25_H_9,
  FAMILY_36_H_11

};
AprilTagFamilies from_string(const std::string & str);

class AprilTagDataSource
{
public:
  /// \brief creates a new AprilTagDataSource
  /// \param apriltag_imgs_location the location of the april tag images repository
  AprilTagDataSource(std::string apriltag_imgs_location);

  /// \brief loads an april tag from the given family and id
  cv::Mat load_april_tag(AprilTagFamilies family, int id) const;

private:
  const std::filesystem::path apriltag_imgs_location;
};

/// \brief Draws an april tag onto an image to be displayed
std::pair<pi_eink::Image, AprilTagInfo> draw_april_tag(
  const AprilTagDataSource & data_source,
  const EPaper & display,
  double desired_square_size,
  AprilTagFamilies family,
  int id
);

}


#endif
