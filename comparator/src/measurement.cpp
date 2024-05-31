#include "comparator/measurement.hpp"
#include <iostream>

void write_measurement(
  cv::FileStorage & fs,
  const Measurement & measurement)
{
  fs << '{';
  fs << "image points" << cv::Mat(measurement.image_points).reshape(1);
  fs << "world points" << cv::Mat(measurement.world_points).reshape(1);
  fs << "fiducial points" << cv::Mat(measurement.fiducial_points).reshape(1);
  fs << '}';
}

Measurement read_measurement(const cv::FileNode & node)
{
  cv::Mat image_points, world_points, fiducial_points;
  node["image_points"] >> image_points;
  node["world_points"] >> world_points;
  node["fiducial_points"] >> fiducial_points;

  Measurement measurement;

  // conver the mats to vectors
  cv::Mat_<cv::Point2f> image_points_mat(image_points);
  cv::Mat_<cv::Point3f> world_points_mat(world_points);
  cv::Mat_<cv::Point3f> fiducial_points_mat(fiducial_points);

  measurement.image_points = std::vector<cv::Point2f>(
    image_points_mat.begin(),
    image_points_mat.end());
  measurement.world_points = std::vector<cv::Point3f>(
    world_points_mat.begin(),
    world_points_mat.end());
  measurement.fiducial_points = std::vector<cv::Point3f>(
    fiducial_points_mat.begin(), fiducial_points_mat.end());

  return measurement;
}

void write_measurements(
  const std::vector<Measurement> & measurements,
  const std::string & file_path)
{
  cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
  fs << "measurements" << '[';
  for (const auto & measurement: measurements) {
    write_measurement(fs, measurement);
  }
  fs << ']';
}

std::vector<Measurement> load_measurements(const std::string & file_path)
{
  cv::FileStorage fs(file_path, cv::FileStorage::READ);
  cv::FileNode measurements_node = fs["measurements"];
  std::vector<Measurement> measurements;
  for (const auto & measurement_node: measurements_node) {
    measurements.push_back(read_measurement(measurement_node));
  }
  return measurements;
}
