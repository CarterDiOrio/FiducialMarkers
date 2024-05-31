#ifndef INC_GUARD_MEASUREMENT_HPP
#define INC_GUARD_MEASUREMENT_HPP

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

/// \brief A measurement of the fiducial for extrinsic calibration
struct Measurement
{
  /// \brief The image points
  std::vector<cv::Point2f> image_points;

  /// \brief The world points
  std::vector<cv::Point3f> world_points;

  /// \brief The points on the fiducial
  std::vector<cv::Point3f> fiducial_points;
};

/// \brief Writes the measurements to a file
/// \param measurements The measurements to write
/// \param file_path The path to the file to write the measurements to
void write_measurements(
  const std::vector<Measurement> & measurements,
  const std::string & file_path);

/// \brief Reads a measurement from a file node
/// \param node The file node to read the measurement from
/// \return The measurement
Measurement read_measurement(
  const cv::FileNode & node);

/// \brief Writes a measurement to a file
/// \param fs The file storage to write the measurement to
void write_measurement(
  cv::FileStorage & fs,
  const Measurement & measurement);

/// \brief Loads the measurements from a file
/// \param file_path The path to the file to load the measurements from
/// \return The measurements
std::vector<Measurement> load_measurements(const std::string & file_path);


#endif
