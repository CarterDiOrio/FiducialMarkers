import svg
from tags import *


class Renderer:
    """Handles Rendering the fiducials to SVG and a printable format."""

    def __init__(self, width: float, height: float):
        self.svg = svg.SVG()
        self.width = width
        self.height = height

    def add_fiducial(self, bits: TagBits, x: float, y: float, size: float):
        """Adds a fiducial to the SVG.

        Args:
            bits (TagBits): The tag's bits
            x (float): The x coordinate of the tag
            y (float): The y coordinate of the tag
            size (float): The size of the tag cells in (mm)
        """

        size = len(bits)

        for i, row in enumerate(bits):
            for j, bit in enumerate(row):
                if bit:
                    self.svg.add_rect(x + j * size, y + i * size, size, size, "black")

    def to_svg_str(self):
        """Converts the SVG object to a string.

        Returns:
            str: The SVG string
        """
        return self.svg.to_str()
