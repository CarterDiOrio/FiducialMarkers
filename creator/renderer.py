import svg
from tags import *


class Renderer:
    """Handles Rendering the fiducials to SVG and a printable format."""

    # px_per_mm = 96 / 25.4  # 96 px/inch / 2.54 cm/inch / 10 mm/cm
    px_per_mm = 10

    def __init__(self, width: float, height: float):
        """ """
        self.width = Renderer.to_px(width)
        self.height = Renderer.to_px(height)
        self.elements = []

    def add_fiducial(self, bits: TagBits, x: float, y: float, size: float):
        """Adds a fiducial to the SVG.

        Args:
            bits (TagBits): The tag's bits
            x (float): The x coordinate of the tag
            y (float): The y coordinate of the tag
            size (float): The size of the tag cells in (mm)
        """

        size = Renderer.to_px(size)
        for i, row in enumerate(bits):
            for j, bit in enumerate(row):
                if not bit:
                    cx = Renderer.to_px(x) + j * size
                    cy = Renderer.to_px(y) + i * size
                    self.elements.append(
                        svg.Rect(x=cx, y=cy, width=size, height=size, stroke_width=0)
                    )

    def to_svg_str(self):
        """Converts the SVG object to a string.

        Returns:
            str: The SVG string
        """
        return svg.SVG(
            width=self.width, height=self.height, elements=self.elements
        ).as_str()

    @staticmethod
    def to_px(mm: float) -> float:
        """Converts mm to px.

        Args:
            mm (float): The size in mm

        Returns:
            float: The size in px
        """
        return mm * Renderer.px_per_mm
