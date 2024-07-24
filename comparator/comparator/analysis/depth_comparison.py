import numpy as np 
import matplotlib.pyplot as plt
import dataclasses

@dataclasses.dataclass
class Measurement:
    tag_translations: np.ndarray
    tag_rotations: np.ndarray 
    depth_translations: np.ndarray
    depth_rotations: np.ndarray

def load_measurements(file_path: str):
    with open(file_path, 'r') as file:
        data = np.loadtxt(file_path)

        tag_translations = data[:, 0:3] * 1000
        depth_translations = data[:, 3:6] * 1000
        tag_rotations = data[:, 6:8]
        depth_rotations = data[:, 8:10]

        return Measurement(
            tag_translations=tag_translations,
            tag_rotations=tag_rotations,
            depth_translations=depth_translations,
            depth_rotations=depth_rotations)

translation_files = [
    'straight_depth1.json',
    'straight_depth2.json',
    'straight_depth3.json',
]

rotation_files = [
    'rotation_depth0.json',
    'rotation_depth00.json',
    'rotation_depth1.json',
    'rotation_depth11.json',
    'rotation_depth2.json',
    'rotation_depth22.json',
    # 'rotation_depth33.json',
    'rotation_depth3.json',
    'rotation_depth44.json',
    'rotation_depth4.json',
    'rotation_depth55.json',
    'rotation_depth5.json',
]


translation_measurements = [
    load_measurements(file) for file in translation_files
]

rotation_measurements = [
    load_measurements(file) for file in rotation_files
]

rotation_measurements.sort(key=lambda m: np.mean(m.tag_rotations, axis=0)[1])


average_depth = [
    np.mean(m.tag_translations, axis=0)[2]
    for m in translation_measurements
]

for i, m in enumerate(translation_measurements):
  plt.figure()
  plt.title(f'Translation Z at {np.round(average_depth[i], 3)} mm')
  plt.hist(m.tag_translations[:, 2], bins=40, alpha=0.5, label='Tag Z')
  plt.hist(m.depth_translations[:, 2], bins=40, alpha=0.5, label='Depth Z')

  tag_std = np.std(m.tag_translations[:, 2])
  depth_std = np.std(m.depth_translations[:, 2])

  min_tag = np.mean(m.tag_translations[:, 2]) - 3 * tag_std
  max_tag = np.mean(m.tag_translations[:, 2]) + 3 * tag_std

  min_depth = np.mean(m.depth_translations[:, 2]) - 2 * depth_std
  max_depth = np.mean(m.depth_translations[:, 2]) + 2 * depth_std

  print(depth_std)

  x_min = min(min_tag, min_depth)
  x_max = max(max_tag, max_depth)

  plt.xlim(x_min, x_max)

  plt.legend()


tag_std_z = [
    np.std(m.tag_translations, axis=0)[2]
    for m in translation_measurements
]

depth_std_z = [
    np.std(m.depth_translations, axis=0)[2]
    for m in translation_measurements
]

plt.figure()
plt.plot(average_depth, tag_std_z, label='Tag')
plt.plot(average_depth, depth_std_z, label='Depth')
plt.xlabel('Average Distance (mm)')
plt.ylabel('Standard Deviation (mm)')
plt.title('Standard Deviation of Z Translation')
plt.legend()
plt.show()

tag_std_rot_x = [
    np.std(m.tag_rotations, axis=0)[0]
    for m in rotation_measurements
]

depth_std_rot_x = [
    np.std(m.depth_rotations, axis=0)[0]
    for m in rotation_measurements
]

tag_std_rot_y = [
    np.std(m.tag_rotations, axis=0)[0]
    for m in rotation_measurements
]

depth_std_rot_y = [
    np.std(m.depth_rotations, axis=0)[1]
    for m in rotation_measurements
]

average_rot_x = [
    np.mean(m.tag_rotations, axis=0)[0]
    for m in rotation_measurements
]

average_rot_y = [
    np.mean(m.tag_rotations, axis=0)[1]
    for m in rotation_measurements
]

print(tag_std_rot_y)

plt.figure()
# plt.plot(average_rot_y, tag_std_rot_x, label='Tag X Angle (deg)')
# plt.plot(average_rot_y, depth_std_rot_x, label='Depth X Angle (deg)')
plt.plot(average_rot_y, tag_std_rot_y, label='Tag Y Angle (deg)')
plt.plot(average_rot_y, depth_std_rot_y, label='Depth Y Angle (deg)')
plt.xlabel('Average Y Rotation (deg)')
plt.ylabel('Standard Deviation (deg)')
plt.title('Standard Deviation of Rotation')
plt.legend()


plt.figure()

average_depth_rot_y = [
    np.mean(m.depth_rotations, axis=0)[1]
    for m in rotation_measurements
]

average_depth_rot_x = [
    np.mean(m.depth_rotations, axis=0)[0]
    for m in rotation_measurements
]


plt.plot(average_rot_y, np.abs(np.subtract(average_rot_y, average_depth_rot_y)), label='Difference Y (deg)')
plt.plot(average_rot_y, np.abs(np.subtract(average_rot_x, average_depth_rot_x)), label='Difference X (deg)')
plt.xlabel('Average Y Rotation (deg)')
plt.ylabel('Difference (deg)')
plt.title('Aboslute Difference in Average Rotation')
plt.legend()
plt.show()





            
