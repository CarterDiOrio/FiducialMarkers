import svg
from tags import *
from cairosvg import svg2pdf
import numpy as np


class Renderer:
    """Handles Rendering the fiducials to SVG and a printable format."""

    px_per_mm = 10
    dpi = 254

    def __init__(
        self,
        width: float,
        height: float,
        origin_at_center: bool = False,
    ):
        self.width = width
        self.height = height
        self.origin_at_center = origin_at_center
        self.elements = []

    def add_marker(self, x: float, y: float, radius: float):
        """Adds a circular marker to the SVG.

        Args:
            x (float): the x coordinate of the marker (mm)
            y (float): the y coordinate of the marker (mm)
            radius (float): the radius of the marker (mm)
        """
        cx = Renderer.to_px(x)
        cy = Renderer.to_px(y)
        radius = Renderer.to_px(radius)
        self.elements.append(svg.Circle(cx=cx, cy=cy, r=radius, stroke_width=0))

    def add_rectangular_fiducial(
        self, bits: TagBits, x: float, y: float, size: float, centered: bool = False
    ) -> np.ndarray:
        """Adds a fiducial to the SVG.

        Args:
            bits (TagBits): The tag's bits
            x (float): The x coordinate of the tag (mm)
            y (float): The y coordinate of the tag (mm)
            size (float): The size of the tag cells in (mm)
            centered (bool): Whether the tag should be centered at the given location.
                Default is top left corner.

        Returns:
            a Nx2 numpy array of the top left corner locations in mm relative to the top left
            corner of the paper
        """

        corner_locations = []

        if centered:
            x -= (len(bits[0]) * size) / 2
            y -= (len(bits) * size) / 2

        for i, row in enumerate(bits):
            for j, bit in enumerate(row):
                cx = x + j * size
                cy = y + i * size
                corner_locations.append([cx, cy])

                cx, cy = self.point_to_px(cx, cy)

                # render the tag if the bit is filled
                if not bit:
                    self.elements.append(
                        svg.Rect(
                            x=cx,
                            y=cy,
                            width=Renderer.to_px(size),
                            height=Renderer.to_px(size),
                            stroke_width=0,
                        )
                    )

        return np.array(corner_locations)

    def to_svg_str(self):
        """Converts the SVG object to a string.

        Returns:
            str: The SVG string
        """
        return svg.SVG(
            width=Renderer.to_px(self.width),
            height=Renderer.to_px(self.height),
            elements=self.elements,
        ).as_str()

    def to_pdf(self, filename: str):
        """Converts the SVG object to a PDF file.

        Args:
            filename (str): The filename of the PDF file
        """
        svg2pdf(bytestring=self.to_svg_str(), write_to=filename, dpi=Renderer.dpi)

    def point_to_px(self, x: float, y: float) -> Tuple[float, float]:
        """Converts a point in mm to px.

        Args:
            x (float): The x coordinate in mm
            y (float): The y coordinate in mm

        Returns:
            Tuple[float, float]: The x and y coordinates in px
        """

        if self.origin_at_center:
            x += self.width / 2
            y += self.height / 2

        return x * Renderer.px_per_mm, y * Renderer.px_per_mm

    @staticmethod
    def to_px(mm: float) -> float:
        """Converts mm to px.

        Args:
            mm (float): The size in mm

        Returns:
            float: The size in px
        """
        return mm * Renderer.px_per_mm

    @staticmethod
    def to_mm(px: int) -> float:
        """Converts svg px units to mm

        Args:
            px (int): the pixel unit

        Returns:
            float: the converted mm
        """
        return px / Renderer.px_per_mm
