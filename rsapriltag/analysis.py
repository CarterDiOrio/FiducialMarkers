import numpy as np 
import matplotlib.pyplot as plt
import dataclasses
import json


@dataclasses.dataclass
class Measurement:
    """contains rotation and translation ndarrays where each row is a 3 vector the measurement"""
    translations: np.ndarray
    rotations: np.ndarray

def load_measurements(file_path: str):
    
    measurements = {};

    with open(file_path, 'r') as file:
        data = json.load(file)  


        for name, value in data["observations"].items():
            
            translations = np.array(value["translation"])
            rotations = np.array(value["rotation"])

            measurements[name] = Measurement(
                translations * 1000,
                rotations * 180 / np.pi
            )
        
    return measurements


def analyze(measurements):

    # mean center the translations
    color_mean = np.mean(measurements['color'].translations, axis=0)
    irleft_mean = np.mean(measurements['irleft'].translations, axis=0)
    irright_mean = np.mean(measurements['irright'].translations, axis=0)
    multi_mean = np.mean(measurements['multi'].translations, axis=0)
    average_mean = np.mean(measurements['average'].translations, axis=0)

    color_translations = measurements['color'].translations - color_mean
    irleft_translations = measurements['irleft'].translations - irleft_mean
    irright_translations = measurements['irright'].translations - irright_mean
    multi_translations = measurements['multi'].translations - multi_mean
    average_translations = measurements['average'].translations - average_mean

    # take the magnitude of each
    color_err = np.atleast_2d(np.linalg.norm(color_translations, axis=1)).T
    irleft_err = np.atleast_2d(np.linalg.norm(irleft_translations, axis=1)).T
    irright_err = np.atleast_2d(np.linalg.norm(irright_translations, axis=1)).T
    multi_err = np.atleast_2d(np.linalg.norm(multi_translations, axis=1)).T
    average_err = np.atleast_2d(np.linalg.norm(average_translations, axis=1)).T

    # get the std deviation of each translation and rotation
    color_std_dev_translation = np.std(color_err)
    irleft_std_dev_translation = np.std(irleft_err)
    irright_std_dev_translation = np.std(irright_err)
    multi_std_dev_translation = np.std(multi_err)
    average_std_dev_translation = np.std(average_err)

    color_var = np.var(color_err)
    irleft_var = np.var(irleft_err)
    irright_var = np.var(irright_err)
    multi_var = np.var(multi_err)
    average_var = np.var(average_err)

    print()
    print(f"color mean: {color_mean}")
    print(f"irleft mean: {irleft_mean}")
    print(f"irright mean: {irright_mean}")
    print(f"multi mean: {multi_mean}")
    print(f"average mean: {average_mean}")

    print()
    print(f"color trans std: {color_std_dev_translation}")
    print(f"irleft trans std: {irleft_std_dev_translation}")
    print(f"irright trans std: {irright_std_dev_translation}")
    print(f"multi trans std: {multi_std_dev_translation}")
    print(f"average trans std: {average_std_dev_translation}")


    # make a histogram of all the errors 
    plt.figure()
    plt.hist(color_err, bins=50, alpha=0.5, label='color')
    plt.hist(irleft_err, bins=50, alpha=0.5, label='irleft')
    plt.hist(irright_err, bins=50, alpha=0.5, label='irright')
    plt.hist(multi_err, bins=50, alpha=0.5, label='multi')
    plt.hist(average_err, bins=50, alpha=0.5, label='average')
    plt.title('Translation Error Magnitude')
    plt.ylabel('Frequency')
    plt.xlabel('Error (mm)')
    plt.legend(loc='upper right')

    plt.figure()
    plt.hist(color_translations[:, 2], bins=50, alpha=0.5, label='color')
    plt.hist(irleft_translations[:, 2], bins=50, alpha=0.5, label='irleft')
    plt.hist(irright_translations[:, 2], bins=50, alpha=0.5, label='irright')
    plt.hist(multi_translations[:, 2], bins=50, alpha=0.5, label='multi')
    plt.hist(average_translations[:, 2], bins=50, alpha=0.5, label='average')
    plt.title('Z Translation Error')
    plt.ylabel('Frequency')
    plt.xlabel('Error (mm)')
    plt.legend(loc='upper right')

    # make a histogram of all the rotations
    # mean center the translations
    color_mean = np.mean(measurements['color'].rotations, axis=0)
    irleft_mean = np.mean(measurements['irleft'].rotations, axis=0)
    irright_mean = np.mean(measurements['irright'].rotations, axis=0)
    multi_mean = np.mean(measurements['multi'].rotations, axis=0)
    average_mean = np.mean(measurements['average'].rotations, axis=0)

    color_rotations = measurements['color'].rotations - color_mean
    irleft_rotations = measurements['irleft'].rotations - irleft_mean
    irright_rotations = measurements['irright'].rotations - irright_mean
    multi_rotations = measurements['multi'].rotations - multi_mean
    average_rotations = measurements['average'].rotations - average_mean

    # take the magnitude of each
    color_err = np.atleast_2d(np.linalg.norm(color_rotations, axis=1)).T
    irleft_err = np.atleast_2d(np.linalg.norm(irleft_rotations, axis=1)).T
    irright_err = np.atleast_2d(np.linalg.norm(irright_rotations, axis=1)).T
    multi_err = np.atleast_2d(np.linalg.norm(multi_rotations, axis=1)).T
    average_err = np.atleast_2d(np.linalg.norm(average_rotations, axis=1)).T

    # get the std deviation of each translation and rotation
    color_std_dev_rotations = np.std(color_err)
    irleft_std_dev_rotations = np.std(irleft_err)
    irright_std_dev_rotations = np.std(irright_err)
    multi_std_dev_rotations = np.std(multi_err)
    average_std_dev_rotations = np.std(average_err)

    # find std deviation on each rotation axis
    color_x_std = np.std(color_rotations[:, 0])
    color_y_std = np.std(color_rotations[:, 1])
    color_z_std = np.std(color_rotations[:, 2])

    irleft_x_std = np.std(irleft_rotations[:, 0])
    irleft_y_std = np.std(irleft_rotations[:, 1])
    irleft_z_std = np.std(irleft_rotations[:, 2])

    irright_x_std = np.std(irright_rotations[:, 0])
    irright_y_std = np.std(irright_rotations[:, 1])
    irright_z_std = np.std(irright_rotations[:, 2])

    multi_x_std = np.std(multi_rotations[:, 0])
    multi_y_std = np.std(multi_rotations[:, 1])
    multi_z_std = np.std(multi_rotations[:, 2])

    average_x_std = np.std(average_rotations[:, 0])
    average_y_std = np.std(average_rotations[:, 1])
    average_z_std = np.std(average_rotations[:, 2])


    print()
    print(f"color mean: {color_mean}")
    print(f"irleft mean: {irleft_mean}")
    print(f"irright mean: {irright_mean}")
    print(f"multi mean: {multi_mean}")
    print(f"average mean: {average_mean}")

    print()
    print(f"color rot std: {color_std_dev_rotations}")
    print(f"irleft rot std: {irleft_std_dev_rotations}")
    print(f"irright rot std: {irright_std_dev_rotations}")
    print(f"multi rot std: {multi_std_dev_rotations}")
    print(f"average rot std: {average_std_dev_rotations}")

    # print()
    # print(f"color x rot std: {color_x_std}")
    # print(f"irleft x rot std: {irleft_x_std}")
    # print(f"irright x rot std: {irright_x_std}")
    # print(f"multi x rot std: {multi_x_std}")
    # print(f"average x rot std: {average_x_std}")

    # print()
    # print(f"color y rot std: {color_y_std}")
    # print(f"irleft y rot std: {irleft_y_std}")
    # print(f"irright y rot std: {irright_y_std}")
    # print(f"multi y rot std: {multi_y_std}")
    # print(f"average y rot std: {average_y_std}")

    # print()
    # print(f"color z rot std: {color_z_std}")
    # print(f"irleft z rot std: {irleft_z_std}")
    # print(f"irright z rot std: {irright_z_std}")
    # print(f"multi z rot std: {multi_z_std}")
    # print(f"average z rot std: {average_z_std}")



    # make a histogram of all the errors 
    plt.figure()
    plt.hist(color_err, bins=50, alpha=0.5, label='color')
    plt.hist(irleft_err, bins=50, alpha=0.5, label='irleft')
    plt.hist(irright_err, bins=50, alpha=0.5, label='irright')
    # plt.hist(multi_err, bins=50, alpha=0.5, label='multi')
    plt.hist(average_err, bins=50, alpha=0.5, label='average')
    plt.ylabel('Frequency')
    plt.xlabel('Error (deg)')
    plt.title('Rotation Error Magnitude')
    plt.legend(loc='upper right')

    plt.figure()
    plt.hist(color_rotations[:, 1], bins=50, alpha=0.5, label='color')
    plt.hist(irleft_rotations[:, 1], bins=50, alpha=0.5, label='irleft')
    plt.hist(irright_rotations[:, 1], bins=50, alpha=0.5, label='irright')
    # plt.hist(multi_rotations[:, 1], bins=50, alpha=0.5, label='multi')
    plt.hist(average_rotations[:, 1], bins=50, alpha=0.5, label='average')
    plt.title('Y Rotation Error')
    plt.ylabel('Frequency')
    plt.xlabel('Error (deg)')
    plt.legend(loc='upper right')

    plt.figure()
    plt.hist(color_rotations[:, 0], bins=50, alpha=0.5, label='color')
    plt.hist(irleft_rotations[:, 0], bins=50, alpha=0.5, label='irleft')
    plt.hist(irright_rotations[:, 0], bins=50, alpha=0.5, label='irright')
    # plt.hist(multi_rotations[:, 0], bins=50, alpha=0.5, label='multi')
    plt.hist(average_rotations[:, 0], bins=50, alpha=0.5, label='average')
    plt.title('X Rotation Error')
    plt.legend(loc='upper right')
    plt.ylabel('Frequency')
    plt.xlabel('Error (deg)')

    plt.figure()
    plt.hist(color_rotations[:, 2], bins=50, alpha=0.5, label='color')
    plt.hist(irleft_rotations[:, 2], bins=50, alpha=0.5, label='irleft')
    plt.hist(irright_rotations[:, 2], bins=50, alpha=0.5, label='irright')
    # plt.hist(multi_rotations[:, 2], bins=50, alpha=0.5, label='multi')
    plt.hist(average_rotations[:, 2], bins=50, alpha=0.5, label='average')
    plt.title('Z Rotation Error')
    plt.legend(loc='upper right')
    plt.ylabel('Frequency')
    plt.xlabel('Error (deg)')
    plt.show()

    return {
        'color': (color_std_dev_translation, color_std_dev_rotations),
        'irleft': (irleft_std_dev_translation, irleft_std_dev_rotations),
        'irright': (irright_std_dev_translation, irright_std_dev_rotations),
        'multi': (multi_std_dev_translation, multi_std_dev_rotations),
        'average': (average_std_dev_translation, average_std_dev_rotations)
    }


files = [
  'data.json'
]



results = [analyze(load_measurements(file)) for file in files]

# average the std deviation of each translation and rotation
color_translation = np.mean([result['color'][0] for result in results])
color_rotation = np.mean([result['color'][1] for result in results])

irleft_translation = np.mean([result['irleft'][0] for result in results])
irleft_rotation = np.mean([result['irleft'][1] for result in results])

irright_translation = np.mean([result['irright'][0] for result in results])
irright_rotation = np.mean([result['irright'][1] for result in results])

multi_translation = np.mean([result['multi'][0] for result in results])
multi_rotation = np.mean([result['multi'][1] for result in results])

average_translation = np.mean([result['average'][0] for result in results])
average_rotation = np.mean([result['average'][1] for result in results])

print("Overall: ")
print(f"  Color std deviation: {color_translation}, {color_rotation}")
print(f"  Irleft std deviation: {irleft_translation}, {irleft_rotation}")
print(f"  Irright std deviation: {irright_translation}, {irright_rotation}")
print(f"  Multi std deviation: {multi_translation}, {multi_rotation}")
print(f"  Average std deviation: {average_translation}, {average_rotation}")

print("Compared to Color")
print(f" IRLEFT: {irleft_translation / color_translation}, {irleft_rotation / color_rotation}")
print(f" IRRIGHT: {irright_translation / color_translation}, {irright_rotation / color_rotation}")
print(f" MULTI: {multi_translation / color_translation}, {multi_rotation / color_rotation}")
print(f" AVERAGE: {average_translation / color_translation}, {average_rotation / color_rotation}")

# 1920 - 1920x = 1280
# (1920 - 1280)  / 1920
decrease = lambda x, y: (x - y) / x
print("Decrease in std deviation: ")
print(f"  IRLEFT: {decrease(color_translation, irleft_translation)}, {decrease(color_rotation, irleft_rotation)}")
print(f"  IRRIGHT: {decrease(color_translation, irright_translation)}, {decrease(color_rotation, irright_rotation)}")
print(f"  MULTI: {decrease(color_translation, multi_translation)}, {decrease(color_rotation, multi_rotation)}")
print(f"  AVERAGE: {decrease(color_translation, average_translation)}, {decrease(color_rotation, average_rotation)}")
