"""Save one /camera/image frame as PNG (Phase 6b ghost-toolbox check). Usage: ghost_snap.py out.png"""
import sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main(path: str) -> None:
    rclpy.init()
    n = rclpy.create_node("ghost_snap")
    got: list = []
    n.create_subscription(Image, "/camera/image", got.append, qos_profile_sensor_data)
    for _ in range(100):
        rclpy.spin_once(n, timeout_sec=0.1)
        if got:
            break
    if not got:
        print("no frame")
        return
    m = got[0]
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
    import zlib, struct
    raw = b"".join(b"\x00" + a[i].tobytes() for i in range(m.height))
    def ch(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", m.width, m.height, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(raw)) + ch(b"IEND", b""))
    print("saved", path, m.width, m.height, m.encoding)


main(sys.argv[1])
