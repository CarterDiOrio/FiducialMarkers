from tags import *
from renderer import *
from cairosvg import svg2pdf

paper_width = 216
paper_height = 279.4

tag_bits = to_bits(Tag16h5(), 0)
tag_bits = pack_tag(Tag16h5(), tag_bits)
render = Renderer(paper_width, paper_height)

# top left
render.add_fiducial(tag_bits, 10, 10, 10)

# top right
render.add_fiducial(tag_bits, paper_width - 10 * len(tag_bits) - 10, 10, 10)

# bottom left
render.add_fiducial(tag_bits, 10, paper_height - 10 * len(tag_bits) - 10, 10)

# bottom right
render.add_fiducial(
    tag_bits,
    paper_width - 10 * len(tag_bits) - 10,
    paper_height - 10 * len(tag_bits) - 10,
    10,
)

# chess_board = create_chess_board(6, 9)


# render.add_fiducial(chess_board, 10, 10, 25)
print(render.to_svg_str())
svg2pdf(bytestring=render.to_svg_str(), write_to="fiducial.pdf", dpi=254)
