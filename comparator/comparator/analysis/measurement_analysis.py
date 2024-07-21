import dataclasses
from typing import List
import numpy as np
import json
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import scipy.stats as stats
import pandas as pd

@dataclasses.dataclass
class Extrinsics:
  T_hand_eye: np.ndarray
  T_mount_fiducial: np.ndarray

@dataclasses.dataclass
class Measurement:
  algorithm_name: str
  T_eye_fiducial: List[np.ndarray]
  T_world_hand: np.ndarray
  T_world_mount: np.ndarray

@dataclasses.dataclass
class MeasurementSet:
  prefix: str
  T_world_hand: np.ndarray
  T_world_mount: np.ndarray
  measurements: List[Measurement]

@dataclasses.dataclass
class MeasurementFile:
  extrinsics: Extrinsics
  measurement_sets: List[MeasurementSet]

@dataclasses.dataclass
class MeasurementsPd:
  position_df: pd.DataFrame
  measurements_df: pd.DataFrame

def to_xyz_rpy(T: np.ndarray) -> np.ndarray:
  """Converts a 4x4 transformation matrix to xyz rpy"""
  r = Rotation.from_matrix(T[:3, :3])
  xyz = T[:3, 3].T
  rpy = r.as_euler('xyz', degrees=True)
  return np.concatenate([xyz, rpy])


def load_extrinsics(json: dict) -> Extrinsics:
  """Parses the extrinsics from the json object"""
  T_hand_eye = np.array(json["T_hand_eye"]).reshape(4, 4).T
  T_mount_fiducial = np.array(json["T_mount_fiducial"]).reshape(4, 4).T
  return Extrinsics(T_hand_eye, T_mount_fiducial)

def load_measurement(json: dict, T_world_hand: np.ndarray, T_world_mount: np.ndarray) -> Measurement: 
  """Parses a measurement from the json object"""
  algorithm_name = json["algorithm_name"]
  T_eye_fiducial = []
  for T in json["eye_fiducial_transforms"]:
    T_eye_fiducial.append(np.array(T).reshape(4, 4).T)

  #filter egrious outliers only if apriltag
  if algorithm_name[0:5] != "april":
    return Measurement(algorithm_name, T_eye_fiducial, T_world_hand, T_world_mount)

  # reject outliers
  inliers = []
  for T in T_eye_fiducial:

    pose_vec = to_xyz_rpy(T)

    if np.linalg.norm(pose_vec[0:3]) < 0.75 and np.linalg.norm(pose_vec[3:]) < 20: #everything is within 2 meters
      inliers.append(T)
  T_eye_fiducial = inliers

  translations = []
  for T in T_eye_fiducial:
    translations.append(T[:3, 3].T)
  translations = np.array(translations)
  
  norms = np.linalg.norm(translations, axis=1)
  d = np.abs(norms - np.median(norms))
  mdev = np.mean(d)
  s = d / mdev if mdev else np.zeros(len(d))

  # scale the translational component of the translation by 1000
  for T in T_eye_fiducial:
    T[:3, 3] = T[:3, 3] * 1000

  # inliers = []
  # for i, T in enumerate(T_eye_fiducial):
  #   if s[i] < 3.0:
  #     inliers.append(T)
  #   else:
  #     print(f"{algorithm_name} outlier: {T[:3, 3].T}")

  return Measurement(algorithm_name, T_eye_fiducial,  T_world_hand, T_world_mount)

def load_measurement_set(json: dict) -> MeasurementSet:
  """Parses a measurement set from the json object"""
  T_world_hand = np.array(json["T_world_hand"]).reshape(4, 4).T
  T_world_mount = np.array(json["T_world_mount"]).reshape(4, 4).T
  prefix = json["prefix"]
  measurements = []
  for measurement in json["measurements"]:
    measurements.append(load_measurement(measurement, T_world_hand, T_world_mount))
  return MeasurementSet(prefix, T_world_hand, T_world_mount, measurements)

def load_measurement_file(filename: str) -> MeasurementFile:
  """Loads a measurement file and returns a list of measurements."""
  with open(filename, 'r') as f:
    data = json.load(f)
    extrinsics = load_extrinsics(data["extrinsics"])

    measurement_sets = []
    for measurement_set in data["measurement_sets"]:
      measurement_sets.append(load_measurement_set(measurement_set))
    return MeasurementFile(extrinsics, measurement_sets)

def measurements_to_pd(measurements: MeasurementFile) -> MeasurementsPd:

  # we want a dataframe of the position of each measurement set with id
  prefixes, t_hand, t_mount = list(zip(*[(ms.prefix, ms.T_world_hand, ms.T_world_mount) for ms in measurements.measurement_sets]))
  position_df = pd.DataFrame(
    {
      'prefix': prefixes,
      'transform_hand': t_hand,
      'transform_mount': t_mount
    },
    index=list(range(0, len(prefixes)))
  )

  # in the name of each algorithm, the first part of the string denotes the type of algorithm
  # the rest of it is various data about the algorithm

  # each individual measurement is then keyed by the position id and measurement id 

  measurements_df = pd.DataFrame(columns=['position_id', 'measurement_id', 'algorithm', 'info', 't_eye_fiducial'])
  
  rows = []
  mid = 0
  for id, measurement_set in enumerate(measurements.measurement_sets):
    
    
    for measurement in measurement_set.measurements:      
      algorithm = measurement.algorithm_name.split('_')[0]  
      algorithm_info = measurement.algorithm_name.removeprefix(algorithm + '_')
      
      for T in measurement.T_eye_fiducial:
        new_row = pd.DataFrame(
          {
            'position_id': [id],
            'measurement_id': [mid],
            'algorithm': [algorithm],
            'info': [algorithm_info],
            't_eye_fiducial': [T]
          }
        )
        
        rows.append(new_row)

      mid += 1

  measurements_df = pd.concat(rows, ignore_index=True)

  return MeasurementsPd(position_df, measurements_df)

def calculate_relative_std(measurements: MeasurementsPd) -> pd.DataFrame:
  """Calculate the the standard deviation for each measurement"""
  
  position_df = measurements.position_df
  measurements_df = measurements.measurements_df

  std_dev_pd = pd.DataFrame(columns=['position_id', 'measurement_id', 'std_translation', 'std_rotation', 'std_xyz', 'std_rpy'])

  rows = []

  for pid, mid in zip(position_df.index, pd.unique(measurements_df.measurement_id)):

    transforms_pd = measurements_df[measurements_df.measurement_id == mid]["t_eye_fiducial"]

    pose_vecs = np.array([
      to_xyz_rpy(T) for T in transforms_pd
    ])

    std_xyz = np.std(pose_vecs[:, 0:3], axis=0)
    std_rpy = np.std(pose_vecs[:, 3:6], axis=0)

    translation_magnitudes = np.linalg.norm(pose_vecs[:, 0:3], axis=1)
    rotation_magnitudes = np.linalg.norm(pose_vecs[:, 3:6], axis=1)
    
    std_translation = np.std(translation_magnitudes)
    std_rotation = np.std(rotation_magnitudes)

    new_row = pd.DataFrame(
      {
        'position_id': [pid],
        'measurement_id': [mid],
        'std_translation': [std_translation],
        'std_rotation': [std_rotation],
        'std_xyz': [std_xyz],
        'std_rpy': [std_rpy]
      },
    )

    rows.append(new_row)

  std_dev_pd = pd.concat(rows, ignore_index=True)
  
  return std_dev_pd
      
if __name__ == "__main__":
  measurement_file = load_measurement_file("scale1080.json")
  measurement_file_720 = load_measurement_file("scale720.json")
  extrinsics = measurement_file.extrinsics


  measurements_pd = measurements_to_pd(measurement_file)
  measurements_pd_720 = measurements_to_pd(measurement_file_720)
  std_dev_pd = calculate_relative_std(measurements_pd)
  std_dev_pd_720 = calculate_relative_std(measurements_pd_720)

  std_dev_pd['prefix'] = measurements_pd.position_df['prefix']
  std_dev_pd_720['prefix'] = measurements_pd_720.position_df['prefix']

  # plot std dev translation and rotation over scale
  scale_sizes = [float(s.split('_')[1]) for s in std_dev_pd['prefix']]
  scale_720 = [float(s.split('_')[1]) * 0.666666 for s in std_dev_pd_720['prefix']]
  
  std_translation = std_dev_pd['std_translation']
  std_rotation = std_dev_pd['std_rotation']

  std_translation_720 = std_dev_pd_720['std_translation']
  std_rotation_720 = std_dev_pd_720['std_rotation']

  # get it just for z axis
  std_xyz = std_dev_pd['std_xyz']
  std_z = [x[2] for x in std_xyz]

  std_xyz_720 = std_dev_pd_720['std_xyz']
  std_z_720 = [x[2] for x in std_xyz_720]

  # calculate the line of best fit
  result = stats.linregress(scale_sizes, std_translation)
  slope = result.slope
  intercept = result.intercept


  plt.figure()
  plt.scatter(scale_sizes, std_translation, label='1080p')
  plt.plot(scale_sizes, slope * np.array(scale_sizes) + intercept, label=f"{slope:.2f}")
  plt.scatter(scale_720, std_translation_720, label='720p')
  plt.xlabel('Scale Size')
  plt.ylabel('Std Dev of Translation Error (mm)')
  plt.title("Std Dev over Scale Size (Translation)")
  plt.legend()

  # calculate line of best fit for rotation
  result = stats.linregress(scale_sizes, std_rotation)
  slope = result.slope
  intercept = result.intercept

  plt.figure()
  plt.scatter(scale_sizes, std_rotation, label='1080p')
  plt.plot(scale_sizes, slope * np.array(scale_sizes) + intercept, label=f"{slope:.2f}")
  plt.scatter(scale_720, std_rotation_720, label='720p')
  plt.xlabel('Scale Size')
  plt.ylabel('Std Dev of Rotation Error (degrees)')
  plt.title("Std Dev over Scale Size (RPY)")
  plt.legend()

  plt.figure()

  #calcualte line of best fit for z axis
  result = stats.linregress(scale_sizes, std_z)
  slope = result.slope
  intercept = result.intercept

  plt.scatter(scale_sizes, std_z, label='1080p')
  plt.plot(scale_sizes, slope * np.array(scale_sizes) + intercept, label=f"{slope:.2f}")
  plt.scatter(scale_720, std_z_720, label='720p')
  plt.xlabel('Scale Size')
  plt.ylabel('Std Dev of Z Translation Error (mm)')
  plt.title("Std Dev over Scale Size (Z)")
  plt.legend()



  plt.show()

  # algo_metrics = {}
  # for algorithm, measurements in algorithm_measurements.items():

  #   translation_errors_by_distance = []

  #   for measurement in measurements:

  #     if len(measurement.T_eye_fiducial) == 0:
  #       continue
        
  #     translations = []
  #     for T in measurement.T_eye_fiducial:
  #       translations.append(T[:3, 3].T)
  #     translations = np.array(translations)

  #     mean_translation = np.mean(translations, axis=0)
  #     mean_centered = translations - mean_translation

  #     dist = np.linalg.inv(measurement.T_world_hand @ extrinsics.T_hand_eye) @ measurement.T_world_mount @ extrinsics.T_mount_fiducial
  #     dist = np.linalg.norm(dist[:3, 3].T)
  #     translation_errors_by_distance.append((dist, mean_centered))

  #   std_dev_by_distance = [(dist, np.std(translations, axis=0) * 1000) 
  #                           for dist, translations in translation_errors_by_distance]

  #   translation_errors = [x[1] for x in translation_errors_by_distance]
  #   translation_errors = np.vstack(translation_errors)
  #   std_dev_translation_error = np.std(translation_errors, axis=0) * 1000

  #   algo_metrics[algorithm] = (std_dev_translation_error)

  #   print(f"Algorithm: {algorithm}: Total StdDev (mm): {std_dev_translation_error}")

  # # plot the std dev by number of corners
  # n_corners = list(range(4, 50, 2))
  # errors = []
  # for algo, metric in algo_metrics.items():
  #   errors.append(np.linalg.norm(metric))

  # print(len(n_corners))
  # print(len(errors))
  # plt.scatter(n_corners, errors)
  # plt.xlabel('Number of Corners')
  # plt.ylabel('Std Dev of Translation Error (mm)')
  # plt.title("Std Dev over number of corners")
  # plt.show()

  # group by algo type
  # types = [
  #   "chess",
  #   "april",
  #   "aruco",
  # ]

  # metric_by_type = {}
  # for algo_type in types:
  #   metric_by_type[algo_type] = []

  # for algo_type in types:
  #   for algo, metric in algo_metrics.items():
  #     if algo[0:len(algo_type)] == algo_type:
  #       metric_by_type[algo_type].append((algo, metric)) 

  # for algo_type, metrics in metric_by_type.items():
  #   for algo, metric in metrics:
  #     # create a scatter plot of translation error std by distance
  #     translation_std_by_distance = metric
  #     distances = [x[0] for x in translation_std_by_distance]
  #     errors = [np.linalg.norm(x[1]) for x in translation_std_by_distance]

  #     result = stats.linregress(distances, errors)
  #     slope = result.slope
  #     intercept = result.intercept

  #     # plot line of best fit
  #     x = np.linspace(100, 1000, 100)
  #     y = slope * x + intercept
  #     plt.plot(x, y, label=algo[0:-3])
  #     plt.scatter(distances, errors)

  #   plt.title("Std Dev over distance")
  #   plt.xlabel('Distance (mm)')
  #   plt.ylabel('Std Dev of Translation Error (mm)')
  #   plt.xlim(250, 1000)
  #   plt.legend()
  #   plt.show()

    







      
    
