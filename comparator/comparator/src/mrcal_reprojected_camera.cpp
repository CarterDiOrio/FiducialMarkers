#include "comparator/mrcal_reprojected_camera.hpp"
#include <algorithm>
#include <memory>
#include <iostream>

extern "C" {
  #include <mrcal/basic_geometry.h>
  #include <mrcal/mrcal.h>
}

#include <opencv2/imgproc.hpp>

std::shared_ptr<MrCalReprojectedCamera> MrCalReprojectedCamera::from_files(
  std::shared_ptr<CameraIntf> && camera,
  const std::string & rich_camera_model_file,
  const std::string & lean_camera_model_file)
{
  return std::make_shared<MrCalReprojectedCamera>(
    std::move(camera),
    load_lens_model(rich_camera_model_file),
    load_lens_model(lean_camera_model_file)
  );
}

MrCalReprojectedCamera::MrCalReprojectedCamera(
  std::shared_ptr<CameraIntf> && camera,
  std::unique_ptr<mrcal_cameramodel_t> && rich_lens_model,
  std::unique_ptr<mrcal_cameramodel_t> && lean_lens_model)
: _camera{camera},
  _rich_lens_model{std::move(rich_lens_model)},
  _lean_lens_model{std::move(lean_lens_model)},
  _reprojection_maps{
    create_reprojection_map(
      *_rich_lens_model,
      *_lean_lens_model,
      _camera->get_resolution()
    )
  }
{
  std::cout << "MrCalReprojectedCamera constructor" << std::endl;
}

cv::Size MrCalReprojectedCamera::get_resolution() const
{
  return _camera->get_resolution();
}

cv::Mat MrCalReprojectedCamera::get_frame() const
{
  const auto frame = _camera->get_frame();
  cv::Mat remapped_frame;
  cv::remap(
    frame,
    remapped_frame,
    _reprojection_maps.map_x,
    _reprojection_maps.map_y,
    cv::INTER_LINEAR
  );
  return remapped_frame;
}

cv::Mat MrCalReprojectedCamera::get_intrinsics() const
{
  const auto intrinsics = _lean_lens_model->intrinsics;

  cv::Mat intrinsics_mat = cv::Mat::zeros(3, 3, CV_64F);
  intrinsics_mat.at<double>(0, 0) = intrinsics[0];
  intrinsics_mat.at<double>(0, 2) = intrinsics[2];
  intrinsics_mat.at<double>(1, 1) = intrinsics[1];
  intrinsics_mat.at<double>(1, 2) = intrinsics[3];
  intrinsics_mat.at<double>(2, 2) = 1.0;

  return intrinsics_mat;
}

std::unique_ptr<mrcal_cameramodel_t> load_lens_model(
  const std::string & file_path)
{
  std::unique_ptr<mrcal_cameramodel_t> model{
    mrcal_read_cameramodel_file(file_path.c_str())
  };
  return model;
}

cv::Mat get_intrinsics_from_camera_model(
  const mrcal_cameramodel_t & camera_model)
{
  const auto intrinsics = camera_model.intrinsics;

  cv::Mat intrinsics_mat = cv::Mat::zeros(3, 3, CV_64F);
  intrinsics_mat.at<double>(0, 0) = intrinsics[0];
  intrinsics_mat.at<double>(0, 2) = intrinsics[2];
  intrinsics_mat.at<double>(1, 1) = intrinsics[1];
  intrinsics_mat.at<double>(1, 2) = intrinsics[3];
  intrinsics_mat.at<double>(2, 2) = 1.0;

  return intrinsics_mat;
}

ReprojectionMaps create_reprojection_map(
  const mrcal_cameramodel_t & rich_lens_model,
  const mrcal_cameramodel_t & lean_lens_model,
  const cv::Size & resolution)
{
  std::cout << "Creating reprojection map of size: " << resolution.area() <<
    std::endl;


  // create a grid of points over the destination image
  const auto points_to = [&resolution] {
      std::vector<mrcal_point2_t> points;
      for (int r = 0; r < resolution.height; r++) {
        for (int c = 0; c < resolution.width; c++) {
          mrcal_point2_t point;
          point.x = c;
          point.y = r;
          points.push_back(point);
        }
      }
      return points;
    }();

  std::cout << "Points created" << std::endl;

  const auto rays = [&points_to, &lean_lens_model] {
      std::vector<mrcal_point3_t> rays;
      rays.resize(points_to.size() + 10);
      mrcal_unproject(
        rays.data(),
        points_to.data(),
        points_to.size(),
        &lean_lens_model.lensmodel,
        lean_lens_model.intrinsics);
      return rays;
    }();

  std::cout << "Rays created: " << rays.size() << std::endl;

  const auto projected_points = [&rays, &rich_lens_model] {
      std::vector<mrcal_point2_t> projected_points;
      projected_points.resize(rays.size() + 10);
      mrcal_project(
        projected_points.data(),
        NULL, // do not care about the gradients
        NULL, // do not care about the gradients
        rays.data(),
        rays.size(),
        &rich_lens_model.lensmodel,
        rich_lens_model.intrinsics);
      return projected_points;
    }();


  std::cout << "Projected points created" << std::endl;

  ReprojectionMaps maps {
    cv::Mat::zeros(resolution, CV_32FC1),
    cv::Mat::zeros(resolution, CV_32FC1)
  };
  for (int r = 0; r < resolution.height; r++) {
    for (int c = 0; c < resolution.width; c++) {
      const auto idx = r * resolution.width + c;
      maps.map_x.at<float>(r, c) = projected_points.at(idx).x;
      maps.map_y.at<float>(r, c) = projected_points.at(idx).y;
    }
  }

  return maps;
}
