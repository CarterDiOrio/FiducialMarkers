import dataclasses
from typing import List
import numpy as np
import json
import cv2 as cv2
from scipy.spatial.transform import Rotation

@dataclasses.dataclass
class Measurement:
  image_points: List[np.ndarray]
  T_world_hand: np.ndarray
  T_world_mount: np.ndarray

@dataclasses.dataclass
class Extrinsics:
  T_hand_eye: np.ndarray
  T_mount_fiducial: np.ndarray

def load_measurement_file(measurement_file: str) -> List[Measurement]:
  """Loads a measurement file and returns a list of measurements."""
  measurements = []

  with open(measurement_file, 'r') as f:
    data = json.load(f)
    for measurement in data["observations"]:

      image_points = []
      for point_obs in measurement["chessboard_observation"]["corners"]:
        image_points.append(np.array(point_obs["img_point"]))
        
      T_world_hand = np.array(measurement['T_world_cmount']).reshape(4, 4).T
      T_world_mount = np.array(measurement['T_world_mount']).reshape(4, 4).T
      measurements.append(Measurement(image_points, T_world_hand, T_world_mount))
    
  return measurements

def load_extrinsics(extrinsics_file: str) -> Extrinsics:
  """Loads the extrinsics file and returns the extrinsics."""
  with open(extrinsics_file, 'r') as f:
    data = json.load(f)
    T_hand_eye = np.array(data['T_hand_eye']).reshape(4, 4).T
    T_mount_fiducial = np.array(data['T_mount_fiducial']).reshape(4, 4).T
    return Extrinsics(T_hand_eye, T_mount_fiducial)
  
def evaluate_measurement(measurement: Measurement, extrinsics: Extrinsics):

  T_world_camera = measurement.T_world_hand @ extrinsics.T_hand_eye
  T_world_object = measurement.T_world_mount @ extrinsics.T_mount_fiducial
  T_camera_object = np.linalg.inv(T_world_camera) @ T_world_object

  # create chessboard points matrix for pnp
  obj_points = []
  # for r in range(0, 8):
  #   for c in range(0, 8):
  #     obj_points.append([
  #       c * 11.984 - (11.984*7) / 2,
  #       -r*11.984 + (11.984*7) / 2,
  #       0
  #     ])
  for r in range(0, 10):
    for c in range(0, 10):
      obj_points.append([
        c * 15 - (15*9) / 2,
        -r*15 + (15*9) / 2,
        0
      ])


  K = np.array([
      [1540.267666, 0, 960.5708739],
      [0, 1465.517343, 538.4441983],
      [0, 0, 1]
    ])

  obj_points = np.array(obj_points, dtype=np.float32)
  image_points = np.array(measurement.image_points, dtype=np.float32)

  ret, rvec, tvec, out = cv2.solvePnPRansac(obj_points, image_points, K, None, flags=cv2.SOLVEPNP_SQPNP)

  # refine rvec and tvec
  rvec, tvec = cv2.solvePnPRefineLM(obj_points, image_points, K, None, rvec, tvec)


  # convert rvec to rotation matrix
  R = cv2.Rodrigues(rvec)[0]
  T = np.array(tvec).reshape(3, 1)

  T_camera_object_m = np.hstack((R, T))
  T_camera_object_m = np.vstack((T_camera_object_m, np.array([0, 0, 0, 1])))

  T_err = np.linalg.inv(T_camera_object_m) @ T_camera_object
  
  # convert T_err to translation and rotation
  translation_err = T_err[:3, 3]
  rotation_err = T_err[:3, :3]

  rot = Rotation.from_matrix(rotation_err)

  # convert to euler angles
  euler = rot.as_euler('xyz', degrees=True)

  return translation_err, euler

measurements = load_measurement_file('measurements_still_test.json')
extrinsics = load_extrinsics('extrinsics.json')

# measurements = load_measurement_file('measurements_still_test.json')
# extrinsics = load_extrinsics('extrinsics.json')

errors = [evaluate_measurement(measurement, extrinsics) for measurement in measurements]

translation_errors = [error[0] for error in errors]
rotation_errors = [error[1] for error in errors]


translation_errors = np.array(translation_errors)
rotation_errors = np.array(rotation_errors)

print('Mean translation error:', np.mean(translation_errors, axis=0))
print('Std dev translation error:', np.std(translation_errors, axis=0))
print('Mean rotation error:', np.mean(rotation_errors, axis=0))
print('Std dev rotation error:', np.std(rotation_errors, axis=0))
print()

# Get the magnitude of the translation error
translation_error_magnitudes = np.linalg.norm(translation_errors, axis=1)
rotation_error_magnitudes = np.linalg.norm(rotation_errors, axis=1)

print('Mean translation error magnitude:', np.mean(translation_error_magnitudes))
print('Std dev translation error magnitude:', np.std(translation_error_magnitudes))
print('Mean rotation error magnitude:', np.mean(rotation_error_magnitudes))
print('Std dev rotation error magnitude:', np.std(rotation_error_magnitudes))
print()

# create a histogram of the translation error magnitudes
import matplotlib.pyplot as plt
plt.hist(translation_error_magnitudes, bins=40)
plt.xlabel('Translation error magnitude (mm)')
plt.ylabel('Frequency')
plt.show()

# create a histogram of the rotation error magnitudes
plt.hist(rotation_error_magnitudes, bins=40)
plt.xlabel('Rotation error magnitude (degrees)')
plt.ylabel('Frequency')
plt.show()






