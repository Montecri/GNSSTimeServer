"""Regenerate the embedded dashboard from src/index.html.

src/index.h.gz is gzip(src/index.html) and src/index_bytearray.h is that archive as a PROGMEM
array; main.cpp serves the array directly with Content-Encoding: gzip. All three must be
regenerated together, so run this after every edit to src/index.html:

    python scripts/build_index_assets.py [--check]

--check verifies the committed artifacts match src/index.html without writing anything, which is
what a reviewer (or a future audit) needs to confirm the served page is the page in the repository.

The gzip member keeps the field layout the previously committed archive used - FNAME "index.h",
XFL=2, OS=255 - and pins the modification time to 0 so that regenerating without a content change
produces no diff.
"""
import argparse
import struct
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HTML = ROOT / "src" / "index.html"
GZIP = ROOT / "src" / "index.h.gz"
ARRAY = ROOT / "src" / "index_bytearray.h"

GZIP_NAME = b"index.h"


def gzip_member(payload):
    compressor = zlib.compressobj(9, zlib.DEFLATED, -zlib.MAX_WBITS)
    deflated = compressor.compress(payload) + compressor.flush()
    header = b"\x1f\x8b\x08\x08" + struct.pack("<I", 0) + b"\x02\xff" + GZIP_NAME + b"\x00"
    trailer = struct.pack("<II", zlib.crc32(payload) & 0xFFFFFFFF, len(payload) & 0xFFFFFFFF)
    return header + deflated + trailer


def byte_array(archive):
    body = ", ".join("0x%02X" % b for b in archive)
    return ("#define index_gz_len %d\r\n"
            "const uint8_t index_gz[] PROGMEM = {\r\n"
            "%s\r\n"
            "};\r\n" % (len(archive), body)).encode("ascii")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="report whether the artifacts are up to date instead of writing them")
    arguments = parser.parse_args()

    payload = HTML.read_bytes()
    archive = gzip_member(payload)
    array = byte_array(archive)

    stale = [path.name for path, content in ((GZIP, archive), (ARRAY, array))
             if not path.exists() or path.read_bytes() != content]
    if arguments.check:
        if stale:
            print("Stale, rerun scripts/build_index_assets.py: " + ", ".join(stale))
            return 1
        print("src/index.h.gz and src/index_bytearray.h match src/index.html")
        return 0

    GZIP.write_bytes(archive)
    ARRAY.write_bytes(array)
    # Decompressing what was just written proves the served bytes really are this page.
    assert zlib.decompress(archive, 16 + zlib.MAX_WBITS) == payload
    print("Wrote src/index.h.gz and src/index_bytearray.h (%d bytes of gzip)" % len(archive))
    return 0


if __name__ == "__main__":
    sys.exit(main())
