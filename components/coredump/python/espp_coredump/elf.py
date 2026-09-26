"""Locating the ELF core file inside the stored flash image.

The image ``CoreDumpService`` serves is the raw core-dump partition contents:
``[flash header][ELF core file][checksum]``. The flash header is
``core_dump_header_t`` (three u32 fields: data_len, version, chip_rev = 12
bytes today), so the ELF magic normally sits at offset 12; the trailing
checksum bytes are harmless to ELF readers, which follow the file's internal
offsets. Search the first KiB rather than assume 12, so a future IDF that grows
the header still works.
"""

from __future__ import annotations

from typing import Optional

ELF_MAGIC = b"\x7fELF"
SEARCH_LIMIT = 1024


def find_elf_offset(image: bytes) -> Optional[int]:
    """The offset of the ELF magic, if it starts within the first KiB
    (offsets 0..SEARCH_LIMIT-1; the magic itself may end past it), else None."""
    pos = bytes(image).find(ELF_MAGIC, 0, SEARCH_LIMIT + len(ELF_MAGIC) - 1)
    return pos if pos >= 0 else None


def extract_elf(image: bytes) -> Optional[bytes]:
    """The ELF core file (from its magic to the end of the image), or None when
    the image holds no ELF (a device built with the binary core-dump format)."""
    pos = find_elf_offset(image)
    return None if pos is None else bytes(image[pos:])
