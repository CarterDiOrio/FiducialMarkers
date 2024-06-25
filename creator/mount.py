import numpy as np
import json
from tags import *


def write_mount_configuration(filename: str, T_mp: np.ndarray, fiducial):

    file_text = ""

    mount_def = {}
    mount_def["Type"] = "ChessBoard"
    mount_def["T_mf"] = T_mp.tolist()
    mount_def["Height"] = fiducial.height
    mount_def["Width"] = fiducial.width
    mount_def["SquareSize"] = fiducial.square_size

    corners = []
    for corner in fiducial.get_corner_locations(centered=True):
        corners.append([corner[0], corner[1]])

    print(len(corners))

    mount_def["Corners"] = corners

    with open(filename, "w") as f:
        json.dump(mount_def, f, indent=4)
