from typing import List, Tuple
from dataclasses import dataclass
import numpy as np

type TagBits = List[List[int]]


class TagFamily:
    def __init__(self):
        self.length: int = 0
        self.tags: List[int] = []


class Tag16h5(TagFamily):
    def __init__(self):
        super().__init__()
        self.length = 4
        self.tags = [
            0x231B,
            0x2EA5,
            0x346A,
            0x45B9,
            0x79A6,
            0x7F6B,
            0xB358,
            0xE745,
            0xFE59,
            0x156D,
            0x380B,
            0xF0AB,
            0x0D84,
            0x4736,
            0x8C72,
            0xAF10,
            0x093C,
            0x93B4,
            0xA503,
            0x468F,
            0xE137,
            0x5795,
            0xDF42,
            0x1C1D,
            0xE9DC,
            0x73AD,
            0xAD5F,
            0xD530,
            0x07CA,
            0xAF2E,
        ]


class ChessBoard:
    """Models a chess board calibration pattern."""

    def __init__(
        self, width: int, height: int, square_size: float, gingham_format: bool = False
    ):
        """Creates a chess board calibartion pattern

        Args:
            width (int): the width of the chess board
            height (int): the height of the chess board
            square_size (float): the size of each square in mm
            gingham_format (bool, optional): Whether or not the pattern is in gingham format. Defaults to False.
        """
        self.width: int = width
        self.height: int = height
        self.square_size: float = square_size
        self.gingham_format: bool = gingham_format

        if self.gingham_format:
            if self.width != self.height:
                raise ValueError("Gingham format only supports square chess boards")
            if self.width % 2 != 0 or self.height % 2 != 0:
                raise ValueError("Gingham format only supports even sized chess boards")
            self.width -= 1
            self.height -= 1

    def to_bits(self) -> TagBits:
        """Creates a bit grid of the chess board pattern

        Args:
            width (int): the width of the chess board
            height (int): the height of the chess board.

        Returns:
            TagBits: The chess board pattern as a bit grid.
        """
        if self.gingham_format:
            return self._gingham_bits()

        bits = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                if (i + j) % 2 == 0:
                    row.append(1)
                else:
                    row.append(0)
            bits.append(row)
        return bits

    def _gingham_bits(self) -> TagBits:
        """Creates a gingham bit grid of the chess board pattern"""

        bits = []
        padded_width = self.width + 4
        padded_height = self.height + 4

        def is_black(r, c):
            # 4 corners
            if r < 2 and c < 2:
                return False
            elif r < 2 and c >= padded_width - 2:
                return False
            elif r >= padded_height - 2 and c < 2:
                return False
            elif r >= padded_height - 2 and c >= padded_width - 2:
                return False

            # outside non corner rows and cols
            elif r < 2:
                return c % 2 == 0
            elif c < 2:
                return r % 2 == 0
            elif r >= padded_height - 2:
                return c % 2 == 0
            elif c >= padded_width - 2:
                return r % 2 == 0

            # internal chess board
            return (r + c) % 2 != 0

        for r in range(padded_height):
            row = []
            for c in range(padded_width):
                row.append(1 if not is_black(r, c) else 0)
            bits.append(row)
        return bits

    def get_corner_locations(self, centered=False) -> np.ndarray:
        """Gets internal corner_locations in mm.

        The ordering of the corners matches what opencv would return.

        Args:
            centered (bool, optional): Whether or not the origin of the chessboard is at its center
                or at the top left corner

        Returns:
            np.ndarray: The internal corner locations in mm.
        """

        corner_locations = []

        if self.gingham_format:
            start = 0
            endh = self.height + 1
            endw = self.width + 1
        else:
            start = 1
            endh = self.height
            endw = self.width

        for r in range(start, endh):
            for c in range(start, endw):
                cx = c * self.square_size
                cy = r * self.square_size

                if centered:
                    cx -= (self.width * self.square_size) / 2
                    cy -= (self.height * self.square_size) / 2

                corner_locations.append([cx, cy])

        # transform all the corners by a transformation matrix
        T = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        for i in range(len(corner_locations)):
            corner = corner_locations[i]
            transformed = T @ np.array([corner[0], corner[1], 0, 1])
            corner_locations[i] = list(transformed[:2])

        return np.array(corner_locations)


def to_bits(tag_family: TagFamily, n: int) -> TagBits:
    """Convers the tag's internal code to a 2D array of bits.

    Args:
        tag_family (TagFamily): The tag family
        n (int): The id of the tag in the family

    Returns:
        List[List[int]]: a 2D grid of the tag's bits
    """
    shift = tag_family.length * 4 - 1
    mask = 0b1 << shift
    tag = tag_family.tags[n]

    bits = []
    for _ in range(tag_family.length):
        row = []
        for _ in range(4):
            row.append((tag & mask) >> shift)
            tag <<= 1
        bits.append(row)
    return bits


def pack_tag(tag_family: TagFamily, bits: TagBits) -> TagBits:
    """Surrounds the tag with necessary border

    Args:
        tag_family (TagFamily): The tag family of the tag
        bits (TagBits): The tag's bits

    Returns:
        TagBits: The tag with border
    """

    size = len(bits)
    packed_tag = [[1] * (size + 4), [1] + ([0] * (size + 2)) + [1]]

    for row in bits:
        packed_tag.append([1, 0] + row + [0, 1])

    packed_tag.append([1] + ([0] * (size + 2)) + [1])
    packed_tag.append([1] * (size + 4))
    return packed_tag
