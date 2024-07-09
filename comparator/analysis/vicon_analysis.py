import numpy as np
import matplotlib.pyplot as plt
import dataclasses
from scipy.spatial.transform import Rotation 


@dataclasses.dataclass
class Errors:
    mean_reprojection_error: float
    std_reprojection_error: float
    mean_reprojection_error_norm: float
    std_reprojection_error_norm: float
    reprojection_errors: list
    mean_unprojected_error: float = None
    std_dev_unprojected_error: float = None

@dataclasses.dataclass
class DistributionData:
    std_dev_x: float
    std_dev_y: float
    std_dev_z: float
    std_dev_roll: float
    std_dev_pitch: float
    std_dev_yaw: float

    covariance: np.ndarray

    mean_x: float
    mean_y: float
    mean_z: float
    mean_roll: float
    mean_pitch: float
    mean_yaw: float

def load_data(filename) -> DistributionData:
    data = np.loadtxt(filename)
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]
    roll = data[:,3]
    pitch = data[:,4]
    yaw = data[:,5]

    # get the mean of each column
    mean_x = np.mean(x)
    mean_y = np.mean(y)
    mean_z = np.mean(z)
    mean_roll = np.mean(roll)
    mean_pitch = np.mean(pitch)
    mean_yaw = np.mean(yaw)

    std_x = np.std(x)
    std_y = np.std(y)
    std_z = np.std(z)
    std_roll = np.std(roll)
    std_pitch = np.std(pitch)
    std_yaw = np.std(yaw)

    cov = np.cov(data, rowvar=False)

    # find how many data points it takes to get within 0.001 of the mean
    print(f"Data from {filename}")
    print(f"Std X: {std_x}")
    print(f"Std Y: {std_y}")
    print(f"Std Z: {std_z}")
    print(f"Std Roll: {std_roll}  Degrees: {np.rad2deg(std_roll)}")
    print(f"Std Pitch: {std_pitch} Degrees: {np.rad2deg(std_pitch)}")
    print(f"Std Yaw: {std_yaw} Degrees: {np.rad2deg(std_yaw)}")
    print() 

    return DistributionData(
        std_x, std_y, std_z, std_roll, std_pitch, std_yaw, cov,
        mean_x, mean_y, mean_z, mean_roll, mean_pitch, mean_yaw)

def simulate_point(point_world, K, world_camera):
    # calculate the perfect projection
    ideal_projection = K @ point_world
    ideal_projection /= ideal_projection[2]

    reprojection_errors = []
    reprojection_error_norms = []
    unprojected_point_error = []
    for (x, y, z, roll, pitch, yaw) in world_camera:
        R = Rotation.from_euler('zyx', [yaw, pitch, roll]).as_matrix()

        translation = np.array([
            [x],
            [y],
            [z]
        ])

        T = np.hstack((R, translation))
        T = np.vstack((T, [0, 0, 0, 1]))
        T = np.linalg.inv(T)

        # transform the point to the camera frame
        point_camera = K @ T @ point_world
        point_image  = point_camera / point_camera[2]

        # calculate the reprojection error
        reprojection_error_x = ideal_projection[0] - point_image[0]
        reprojection_error_y = ideal_projection[1] - point_image[1]
        reprojection_error_norms.append(np.linalg.norm(ideal_projection - point_image))

        reprojection_errors.append(reprojection_error_x)
        reprojection_errors.append(reprojection_error_y)
        
        unprojected_camera = np.array([
            [(point_image[0] - K[0, 2]) / K[0, 0] * point_world[2]],
            [(point_image[1] - K[1, 2]) / K[1, 1] * point_world[2]],
            [point_world[2]],
            [1]
        ])
        
        unprojected_point_error.append(np.linalg.norm(unprojected_camera.T[0] - point_world)) 
    

    # calculate the mean and standard deviation of the reprojection errors
    mean_reprojection_error = np.mean(np.abs(reprojection_errors))
    std_reprojection_error = np.std(np.abs(reprojection_errors))

    mean_reprojection_error_norm = np.mean(reprojection_error_norms)
    std_reprojection_error_norm = np.std(reprojection_error_norms)

    mean_unprojected_error = np.mean(unprojected_point_error)
    std_dev_unprojected_error = np.std(unprojected_point_error)

    return Errors(
        mean_reprojection_error, 
        std_reprojection_error, 
        mean_reprojection_error_norm, 
        std_reprojection_error_norm, 
        reprojection_errors,
        mean_unprojected_error,
        std_dev_unprojected_error)

def simulate_point_over_distance(min_distance, max_distance, world_cameras, K, steps):
    errors_by_distance = []
    for i in np.linspace(min_distance, max_distance, steps):
        test_point = np.array([0, 0, i, 1])
        errors = simulate_point(test_point, K, world_cameras)
        errors_by_distance.append(errors)

        print(f"Distance: {i}")
        print(f"Mean Reprojection Error (Seperate X and Y): {errors.mean_reprojection_error} Std Dev: {errors.std_reprojection_error}")
        print(f"Mean Reprojection Error (Norm): {errors.mean_reprojection_error_norm} Std Dev: {errors.std_reprojection_error_norm}")
        print()

    return errors_by_distance


def simulate_point_on_mount(camera_dist: DistributionData, 
                            mount_dist: DistributionData,
                            mount_position: np.ndarray, 
                            point_on_mount: np.ndarray, 
                            K: np.ndarray,
                            N_samples):
    camera_distribution = np.random.multivariate_normal(
        np.array([0, 0, 0, 0, 0, 0]),
        camera_dist.covariance,
        N_samples
    )

    mount_distribution = np.random.multivariate_normal(
        np.array([0, 0, 0, 0, 0, 0]),
        mount_dist.covariance,
        N_samples
    )

    T_world_mount_ideal = np.array([
        [1, 0 ,0, mount_position[0]],
        [0, 1, 0, mount_position[1]],
        [0, 0, 1, mount_position[2]],
        [0, 0, 0, 1]
    ])

    ideal_reprojection = K @ T_world_mount_ideal @ point_on_mount
    ideal_reprojection /= ideal_reprojection[2]

    reprojection_errors = []
    reprojection_error_norms = []
    for (camera, mount) in zip(camera_distribution, mount_distribution):
        Camera_rot = Rotation.from_euler('zyx', [camera[5], camera[4], camera[3]]).as_matrix()
        Mount_rot = Rotation.from_euler('zyx', [mount[5], mount[4], mount[3]]).as_matrix()

        Camera_translation = np.array([
            [camera[0]],
            [camera[1]],
            [camera[2]]
        ])

        Mount_translation = np.array([
            [mount[0] + mount_position[0]],
            [mount[1] + mount_position[1]],
            [mount[2] + mount_position[2]]
        ])

        T_camera_world = np.hstack((Camera_rot, Camera_translation))
        T_camera_world = np.vstack((T_camera_world, [0, 0, 0, 1]))
        T_camera_world = np.linalg.inv(T_camera_world)

        T_world_mount = np.hstack((Mount_rot, Mount_translation))
        T_world_mount = np.vstack((T_world_mount, [0, 0, 0, 1]))

        point_world = T_world_mount @ point_on_mount
        point_camera = T_camera_world @ point_world

        point_image = K @ point_camera
        point_image /= point_image[2]

        reprojection_error_x = ideal_reprojection[0] - point_image[0]
        reprojection_error_y = ideal_reprojection[1] - point_image[1]
        reprojection_error_norms.append(np.linalg.norm(ideal_reprojection - point_image))

        reprojection_errors.append(reprojection_error_x)
        reprojection_errors.append(reprojection_error_y)

    # calculate the mean and standard deviation of the reprojection errors
    mean_reprojection_error = np.mean(np.abs(reprojection_errors))
    std_reprojection_error = np.std(np.abs(reprojection_errors))

    mean_reprojection_error_norm = np.mean(reprojection_error_norms)
    std_reprojection_error_norm = np.std(reprojection_error_norms)

    return Errors(mean_reprojection_error, std_reprojection_error, mean_reprojection_error_norm, std_reprojection_error_norm, reprojection_errors)

def analyze_data(camera_filename, mount_filename):

    camera_dist = load_data(camera_filename)
    mount_dist = load_data(mount_filename)    

    # create a gaussian distribution for each
    N_samples = 10000
    # x_normal = np.random.normal(0, std_x, N_samples)
    # y_normal = np.random.normal(0, std_y, N_samples)
    # z_normal = np.random.normal(0, std_z, N_samples)
    # roll_normal = np.random.normal(0, std_roll, N_samples)
    # pitch_normal = np.random.normal(0, std_pitch, N_samples)
    # yaw_normal = np.random.normal(0, std_yaw, N_samples)

    # Pinhole model used on camera
    K = np.array([
      [1540.267666, 0, 960.5708739, 0],
      [0, 1465.517343, 538.4441983, 0],
      [0, 0, 1, 0]
    ])

    distribution = np.random.multivariate_normal(
        np.array([0, 0, 0, 0, 0, 0]),
        camera_dist.covariance,
        N_samples
    )

    x_normal = distribution[:,0]
    y_normal = distribution[:,1]
    z_normal = distribution[:,2]
    roll_normal = distribution[:,3]
    pitch_normal = distribution[:,4]
    yaw_normal = distribution[:,5]

    world_camera = list(zip(x_normal, y_normal, z_normal, roll_normal, pitch_normal, yaw_normal))

    # create distirbution at 1000 mm
    test_point = np.array([0, 0, 1000, 1])
    error = simulate_point(test_point, K, world_camera)

    print(f"At 1000 mm:")
    print(f"  Mean Reprojection Error (Seperate X and Y): {error.mean_reprojection_error} Std Dev: {error.std_reprojection_error}")
    print(f"  Mean Reprojection Error (Norm): {error.mean_reprojection_error_norm} Std Dev: {error.std_reprojection_error_norm}")
    print(f"  Mean Unprojected Error: {error.mean_unprojected_error} Std Dev: {error.std_dev_unprojected_error}")
    print()

    plt.hist(error.reprojection_errors, bins=100)
    plt.xlabel('Reprojection Error (px)')
    plt.ylabel('Frequency')
    plt.title('Histogram of reprojection errors at 1000mm')
    plt.show()
    

    # simulate point on mount
    mount_position = np.array([0, 0, 1000, 1])
    point_on_mount = np.array([75, 75, 0, 1])
    error = simulate_point_on_mount(camera_dist, mount_dist, mount_position, point_on_mount, K, N_samples)

    print(f"At (75, 75) on Mount at 1000 mm:")
    print(f"  Mean Reprojection Error (Seperate X and Y): {error.mean_reprojection_error} Std Dev: {error.std_reprojection_error}")
    print(f"  Mean Reprojection Error (Norm): {error.mean_reprojection_error_norm} Std Dev: {error.std_reprojection_error_norm}")
    print()

    plt.hist(error.reprojection_errors, bins=100)
    plt.xlabel('Reprojection Error (px)')
    plt.ylabel('Frequency')
    plt.title('Histogram of reprojection errors at 1000mm on Mount (75, 75)')
    plt.show()


    # errors_by_distance = simulate_point_over_distance(100, 2000, world_camera, K, 25)
    
    # mean_by_distance = [errors.mean_reprojection_error for errors in errors_by_distance]
    # std_dev_by_distance = [errors.std_reprojection_error for errors in errors_by_distance]
    # mean_unprojection_error_by_distance = [errors.mean_unprojected_error for errors in errors_by_distance]
    # std_dev_unprojection_error_by_distance = [errors.std_dev_unprojected_error for errors in errors_by_distance]

    # plt.plot(np.linspace(100, 2000, 25), mean_by_distance, label='Mean (px)')
    # plt.plot(np.linspace(100, 2000, 25), std_dev_by_distance, label='Std Dev (px)')
    # plt.plot(np.linspace(100, 2000, 25), mean_unprojection_error_by_distance, label='Mean unrpoj error (mm)')
    # plt.plot(np.linspace(100, 2000, 25), std_dev_unprojection_error_by_distance, label='Std Dev unproj error (mm)')
    # plt.xlabel('Distance to Point')
    # plt.ylabel('Error')
    # plt.title('Error vs Distance to Point')
    # plt.legend()
    # plt.show()

    
    # simulate how many samples need be averaged to be within 0.001 of the mean
    # rad_goal = np.deg2rad(0.0036)
    # translation_goal = 0.001

    # for N in range(10, 10000, 100):
        
    #     successes = []
    #     for i in range(0, 10000):

    #       distribution = np.random.multivariate_normal(
    #           np.array([0, 0, 0, 0, 0, 0]),
    #           camera_dist.covariance,
    #           N
    #       )

    #       # find average
    #       mean = np.mean(distribution, axis=0)
    #       # print(N, mean)
    #       if np.linalg.norm(mean[0:3] - np.array([0, 0, 0])) < translation_goal and np.linalg.norm(mean[3:6]) < rad_goal:
    #           successes.append(1)
    #       else:
    #           successes.append(0)

    #     print(N, np.mean(successes))
    #     if np.mean(successes) > 0.95:
    #         print(f"Number of samples needed to be within 0.001 of the mean 95% of the time: {N}")
    #         break


    # how many samples do we need to be within 0.1 px of the ideal location 95% of the time
    point_world = np.array([0, 0, 1000, 1])
    ideal_projection = K @ point_world
    ideal_projection /= ideal_projection[2]
    for N in range(10, 100_000, 100):
        successes = []

        for i in range(0, 1000):
            distribution = np.random.multivariate_normal(
                np.array([0, 0, 0, 0, 0, 0]),
                camera_dist.covariance,
                N
            )

            mean = np.mean(distribution, axis=0)
            x, y, z, roll, pitch, yaw = mean
            R = Rotation.from_euler('zyx', [yaw, pitch, roll]).as_matrix()

            translation = np.array([
                [x],
                [y],
                [z]
            ])

            T_camera_world = np.hstack((R, translation))
            T_camera_world = np.vstack((T_camera_world, [0, 0, 0, 1]))
            T_camera_world = np.linalg.inv(T_camera_world)

            point_camera = T_camera_world @ point_world
            point_image = K @ point_camera
            point_image /= point_image[2]

            reprojection_error_norm = np.linalg.norm(ideal_projection - point_image)
            if reprojection_error_norm <= 0.01:
                successes.append(1)
            else:
                successes.append(0)
        
        mean = np.mean(successes)
        print(f"{N}: {mean}")

        if mean > 0.95:
            print(f"Number of samples needed to be within 0.1 px of the mean 95% of the time: {N}")
            break

    
      
analyze_data('camera_measurements.txt', 'mount_measurements.txt')
