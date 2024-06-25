from tags import *
from mount import *
from renderer import *
from cairosvg import svg2pdf

paper_width = 216
paper_height = 279.4

# Top left marker to top left corner transform (mm)
T_mp = np.array([[1, 0, 0, 0], [0, 1, 0, -5], [0, 0, 1, -6], [0, 0, 0, 1]])

tag0 = to_bits(Tag16h5(), 0)
tag0 = pack_tag(Tag16h5(), tag0)

render = Renderer(paper_height, paper_width, origin_at_center=True)

chess_board_bits = ChessBoard(10, 10, 15, gingham_format=True)

render.add_rectangular_fiducial(chess_board_bits.to_bits(), 0, 0, 15, centered=True)

# corner_locations = chess_board_bits.get_corner_locations(centered=True)

render.to_pdf("fiducial.pdf")
write_mount_configuration("./mount.txt", T_mp, chess_board_bits)
