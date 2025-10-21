from __future__ import annotations
from .constants import STX, ETX

# ------------------ CRC8-ATM ------------------
def crc8_atm(buf: bytes, poly=0x07, init=0x00, xorout=0x00) -> int:
    c = init & 0xFF
    for b in buf:
        c ^= b
        for _ in range(8):
            c = (((c << 1) & 0xFF) ^ poly) if (c & 0x80) else ((c << 1) & 0xFF)
    return (c ^ xorout) & 0xFF

# ------------------ Paketleme ------------------
def pack(addr:int, cmd:int, data:bytes) -> bytes:
    addr &= 0xFF; cmd &= 0xFF
    ln = len(data) & 0xFF
    body = bytes([addr, cmd, ln]) + data
    crc  = crc8_atm(body)
    return bytes([STX]) + body + bytes([crc, ETX])

def hex_dump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)
