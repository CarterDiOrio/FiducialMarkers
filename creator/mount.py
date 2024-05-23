import numpy as np

from tags import *


def write_mount_configuration(filename: str, T_mp: np.ndarray, fiducial):

    file_text = ""

    # write T_mp row by row as csv on single line
    for row in T_mp:
        file_text += ",".join([str(x) for x in row]) + ","

    file_text += "\n"

    if isinstance(fiducial, ChessBoard):
        file_text += "ChessBoard\n"
        file_text += f"{fiducial.height-1},{fiducial.width-1},{fiducial.square_size}\n"

        for corner in fiducial.get_corner_locations(centered=True):
            file_text += ",".join([str(x) for x in corner])
            file_text += "\n"

    with open(filename, "w") as f:
        f.write(file_text)
