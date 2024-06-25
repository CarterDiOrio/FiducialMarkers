#include "comparator/mrcal_reprojected_camera.hpp"
#include <algorithm>
#include <memory>
#include <mrcal/basic_geometry.h>
#include <mrcal/mrcal.h>
#include <opencv2/imgproc.hpp>

std::unique_ptr<MrCalReprojectedCamera> MrCalReprojectedCamera::from_files(
  std::unique_ptr<CameraIntf> && camera,
  const std::string & rich_camera_model_file,
  const std::string & lean_camera_model_file)
{
  return std::make_unique<MrCalReprojectedCamera>(
    std::move(camera),
    load_lens_model(rich_camera_model_file),
    load_lens_model(lean_camera_model_file)
  );
}

MrCalReprojectedCamera::MrCalReprojectedCamera(
  std::unique_ptr<CameraIntf> && camera,
  std::unique_ptr<mrcal_cameramodel_t> && rich_lens_model,
  std::unique_ptr<mrcal_cameramodel_t> && lean_lens_model)
: _camera{std::move(camera)},
  _rich_lens_model{std::move(rich_lens_model)},
  _lean_lens_model{std::move(lean_lens_model)},
  _reprojection_maps{
    create_reprojection_map(
      *_rich_lens_model,
      *_lean_lens_model,
      _camera->get_resolution()
    )
  }
{}

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
  return cv::Mat();
}

std::unique_ptr<mrcal_cameramodel_t> load_lens_model(
  const std::string & file_path)
{
  std::unique_ptr<mrcal_cameramodel_t> model{
    mrcal_read_cameramodel_file(file_path.c_str())
  };
  return model;
}

ReprojectionMaps create_reprojection_map(
  const mrcal_cameramodel_t & rich_lens_model,
  const mrcal_cameramodel_t & lean_lens_model,
  const cv::Size & resolution)
{
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

  const auto rays = [&points_to, &lean_lens_model] {
      std::vector<mrcal_point3_t> rays;
      rays.reserve(points_to.size());
      mrcal_unproject(
        rays.data(),
        points_to.data(),
        points_to.size(),
        &lean_lens_model.lensmodel,
        lean_lens_model.intrinsics);
      return rays;
    }();

  const auto projected_points = [&rays, &rich_lens_model] {
      std::vector<mrcal_point2_t> projected_points;
      projected_points.reserve(rays.size());
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

  ReprojectionMaps maps {
    cv::Mat(resolution, CV_64FC1),
    cv::Mat(resolution, CV_64FC1)
  };
  for (int r = 0; r < resolution.height; r++) {
    for (int c = 0; c < resolution.width; c++) {
      const auto idx = r * resolution.width + c;
      maps.map_x.at<double>(r, c) = projected_points[idx].x;
      maps.map_y.at<double>(r, c) = projected_points[idx].y;
    }
  }

  return maps;
}
