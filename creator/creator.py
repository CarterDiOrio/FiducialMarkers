from tags import *


tag_bits = to_bits(Tag16h5(), 0)
for row in tag_bits:
    print(row)

packed_tag = pack_tag(Tag16h5(), tag_bits)
for row in packed_tag:
    print(row)
