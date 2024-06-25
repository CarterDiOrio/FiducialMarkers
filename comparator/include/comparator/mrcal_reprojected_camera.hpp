#ifndef INC_GUARD_MRCAL_REPROJECTED_CAMERA_HPP
#define INC_GUARD_MRCAL_REPROJECTED_CAMERA_HPP

#include "comparator/camera.hpp"
#include <mrcal/mrcal.h>
#include <opencv2/core/types.hpp>

/// \brief The reprojection maps
struct ReprojectionMaps
{
  cv::Mat map_x;
  cv::Mat map_y;
};

/// \brief Reprojects from a rich to lean model using MrCal
/// Most image/3D toolkits do not support the rich spline models provided by
/// by MrCal. This class wraps a camera to reproject from a rich model
/// of the lens down to a lean pinhole model.
/// follows the "Reprojecting to a lean production model" section of the
/// MrCal documentation: https://mrcal.secretsauce.net/recipes.html
/// Does not currently support mapping between two different resolutions
class MrCalReprojectedCamera : public CameraIntf
{
public:
  /// \brief Constructs a MrCalReprojectedCamera
  /// \param camera The camera to reproject from
  /// \param rich_lens_model The rich lens model describing the camera's lens
  /// \param the lean pinhole model to reproject to
  MrCalReprojectedCamera(
    std::unique_ptr<CameraIntf> && camera,
    std::unique_ptr<mrcal_cameramodel_t> && rich_lens_model,
    std::unique_ptr<mrcal_cameramodel_t> && lean_lens_model
  );

  /// \brief loads lens model files and constructs a MrCalReprojectedCamera
  /// \brief camera The camera wrap
  /// \brief rich_camera_model_file The path to the rich camera model file
  /// \brief lean_camera_model_file The path to the lean camera model file
  /// \return The MrCalReprojectedCamera
  static std::unique_ptr<MrCalReprojectedCamera> from_files(
    std::unique_ptr<CameraIntf> && camera,
    const std::string & rich_camera_model_file,
    const std::string & lean_camera_model_file
  );

  /// \brief Gets the resolution of the Camera in pixels
  /// \return The resolution of the camera {width, height}
  virtual cv::Size get_resolution() const override;

  /// \brief Gets a frame from the Camera
  /// \return The frame from the camera
  virtual cv::Mat get_frame() const override;

  /// \brief Gets the intrinsics of the Camera
  /// \return The intrinsics of the camera
  virtual cv::Mat get_intrinsics() const override;

private:
  const std::unique_ptr<CameraIntf> _camera;
  const std::unique_ptr<mrcal_cameramodel_t> _rich_lens_model;
  const std::unique_ptr<mrcal_cameramodel_t> _lean_lens_model;
  const ReprojectionMaps _reprojection_maps;
};

/// \brief Loads a lens model from a file
/// \param file_path The path to the file
std::unique_ptr<mrcal_cameramodel_t> load_lens_model(
  const std::string & file_path);


//// \brief Creates a reprojection map from a rich to lean model
/// Map is to be used with cv2::remap
/// \param rich_lens_model The rich lens model describing the camera's lens
/// \param lean_lens_model The lean pinhole model to reproject to
/// \param resolution The resolution of the camera
ReprojectionMaps create_reprojection_map(
  const mrcal_cameramodel_t & rich_lens_model,
  const mrcal_cameramodel_t & lean_lens_model,
  const cv::Size & resolution);


#endif
