from typing import List
from dataclasses import dataclass

type TagBits = List[List[int]]


class TagFamily:
    def __init__(self):
        self.length: int = 0
        self.tags: List[int] = []


class Tag16h5(TagFamily):
    def __init__(self):
        super().__init__()
        self.length = 4
        self.tags = [0x231B]


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


def create_chess_board(width: int, height: int) -> TagBits:
    """Creates a chessboard pattern

    Args:
        width (int): the width of the chess board
        height (int): the height of the chess board.

    Returns:
        TagBits: The chess board pattern as a bit grid.
    """
    bits = []
    for i in range(height):
        row = []
        for j in range(width):
            if (i + j) % 2 == 0:
                row.append(1)
            else:
                row.append(0)
        bits.append(row)
    return bits
