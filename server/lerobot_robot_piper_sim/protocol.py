"""Protocol-constanten van de opname-client in piper-sandwich-web (src/rec-state.js, src/recorder-client.js).

Toestand/actie: 14 waarden, volgorde vast:
  left_joint1..6.pos [rad], left_gripper.pos [0 = open .. 1 = dicht], idem rechts.
"""
from __future__ import annotations

import struct

PROTO_VERSION = 1
FPS = 30
SIDES = ("left", "right")


def state_names() -> list[str]:
    out: list[str] = []
    for s in SIDES:
        out += [f"{s}_joint{j}.pos" for j in range(1, 7)]
        out.append(f"{s}_gripper.pos")
    return out


# Gewrichtslimieten (rad): assets/scene.xml (range) == piper_sdk INTERFACE_V2 JointCtrl-tabel.
JOINT_LIMITS = {
    1: (-2.618, 2.168),   # scene.xml: range="-2.618 2.168" (SDK: +-2.6179)
    2: (0.0, 3.14),
    3: (-2.967, 0.0),
    4: (-1.745, 1.745),
    5: (-1.22, 1.22),
    6: (-2.0944, 2.0944),
}


def limits_by_name() -> dict[str, tuple[float, float]]:
    lim: dict[str, tuple[float, float]] = {}
    for s in SIDES:
        for j in range(1, 7):
            lim[f"{s}_joint{j}.pos"] = JOINT_LIMITS[j]
        lim[f"{s}_gripper.pos"] = (0.0, 1.0)
    return lim


_HDR = struct.Struct("<IBBH")  # u32 seq, u8 cam_id, u8 fmt (0 = JPEG), u16 reserved


def parse_frame(data: bytes) -> tuple[int, int, int, bytes]:
    """Binair beeldbericht -> (seq, cam_id, fmt, jpeg). Gooit ValueError bij een ongeldige header."""
    if len(data) < _HDR.size + 4:
        raise ValueError("frame te kort")
    seq, cam, fmt, reserved = _HDR.unpack_from(data, 0)
    if fmt != 0 or reserved != 0:
        raise ValueError(f"onbekend frameformaat {fmt}/{reserved}")
    return seq, cam, fmt, bytes(data[_HDR.size:])


def encode_frame(seq: int, cam: int, jpeg: bytes) -> bytes:
    return _HDR.pack(seq & 0xFFFFFFFF, cam, 0, 0) + jpeg


def fnv1a(text: str) -> str:
    """Zelfde FNV-1a (32 bit, hex) als fnv1a() in src/rec-state.js (over UTF-16 code units)."""
    h = 0x811C9DC5
    data = text.encode("utf-16-le")
    for i in range(0, len(data), 2):
        h ^= data[i] | (data[i + 1] << 8)
        h = (h * 0x01000193) & 0xFFFFFFFF
    return f"{h:08x}"
