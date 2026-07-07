#!/usr/bin/env python3
"""
Minimal, robust LDROBOT LD06 -> sensor_msgs/LaserScan node (single-threaded).

Reads the LD06 directly with the same raw termios path proven 100% reliable on
this Jetson (the ldlidar_stl_ros2 SDK times out on the PL2303 here; raw reads
never do). One loop: read serial -> parse 47-byte packets -> accumulate bins ->
publish /scan at 10 Hz by wall clock. No executor/timer/threads => no GIL
contention on the weak Nano CPU. A publisher does not need spin() to publish.

LD06 47-byte packet:
  [0]   0x54 header
  [1]   0x2C  (VerLen; low 5 bits = point count = 12)
  [2:4] speed   uint16 LE, deg/s
  [4:6] start   uint16 LE, 0.01 deg
  [6:42] 12 x (dist uint16 LE mm, intensity uint8)
  [42:44] end   uint16 LE, 0.01 deg
  [44:46] stamp uint16 LE
  [46]  CRC8
"""
import os, termios, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

PORT = os.environ.get("LD06_PORT",
    "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0")
FRAME_ID = os.environ.get("LD06_FRAME", "lidar_link")
TOPIC = os.environ.get("LD06_TOPIC", "scan")
BINS = int(os.environ.get("LD06_BINS", "455"))
PERIOD = float(os.environ.get("LD06_PERIOD", "0.1"))   # 10 Hz publish
PKT = 47
PC = 12

_CRC = [
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8]

def crc8(data):
    c = 0
    for b in data:
        c = _CRC[(c ^ b) & 0xff]
    return c

def open_port(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[3] = 0
    a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    a[4] = termios.B230400; a[5] = termios.B230400
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

class LD06(Node):
    def __init__(self):
        super().__init__("ld06_scan_node")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, TOPIC, qos)
        self.bins = BINS
        self.ranges = [0.0] * self.bins
        self.intens = [0.0] * self.bins
        self.ainc = 2.0 * math.pi / self.bins
        self.fd = open_port(PORT)
        self.get_logger().info(f"LD06 raw node on {PORT} -> /{TOPIC} ({self.bins} bins)")

    def run(self):
        buf = bytearray()
        last_pub = time.time()
        pub_count = 0
        pkt_count = 0
        crc_fail = 0
        bytes_read = 0
        last_log = time.time()
        while rclpy.ok():
            try:
                d = os.read(self.fd, 4096)
            except BlockingIOError:
                d = b""
            except OSError:
                time.sleep(0.05); continue
            if d:
                bytes_read += len(d)
                buf.extend(d)
                i = 0
                n = len(buf)
                while i + PKT <= n:
                    if buf[i] != 0x54 or buf[i+1] != 0x2C:
                        i += 1; continue
                    pkt = buf[i:i+PKT]
                    if crc8(pkt[:PKT-1]) != pkt[PKT-1]:
                        crc_fail += 1
                        i += 1; continue
                    self._parse(pkt)
                    pkt_count += 1
                    i += PKT
                del buf[:i]
            else:
                time.sleep(0.001)
            now = time.time()
            if now - last_pub >= PERIOD:
                last_pub = now
                if self._publish():
                    pub_count += 1
            if now - last_log >= 2.0:
                dt = now - last_log
                self.get_logger().info(
                    f"pub={pub_count/dt:.1f}/s pkt={pkt_count/dt:.0f}/s crcfail={crc_fail/dt:.0f}/s bytes={bytes_read/dt:.0f}/s buf={len(buf)}")
                pub_count = 0; pkt_count = 0; crc_fail = 0; bytes_read = 0; last_log = now

    def _parse(self, p):
        start = (p[4] | (p[5] << 8)) / 100.0
        end   = (p[42] | (p[43] << 8)) / 100.0
        span = (end - start) % 360.0
        step = span / (PC - 1) if PC > 1 else 0.0
        for k in range(PC):
            dist = (p[6+k*3] | (p[7+k*3] << 8))
            ang = (start + step * k) % 360.0
            # LD06 spins CLOCKWISE; ROS LaserScan is counter-clockwise-positive
            # (REP-103). Reverse the angle or the scan comes out mirrored L-R.
            ang = (360.0 - ang) % 360.0
            b = int(ang / 360.0 * self.bins) % self.bins
            self.ranges[b] = dist / 1000.0 if dist > 0 else 0.0
            self.intens[b] = float(p[8+k*3])

    def _publish(self):
        rr = self.ranges
        ii = self.intens
        if not any(v > 0.0 for v in rr):
            return False
        self.ranges = [0.0] * self.bins
        self.intens = [0.0] * self.bins
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = FRAME_ID
        m.angle_min = 0.0
        m.angle_max = 2.0 * math.pi - self.ainc
        m.angle_increment = self.ainc
        m.scan_time = PERIOD
        m.time_increment = PERIOD / self.bins
        m.range_min = 0.02
        m.range_max = 12.0
        m.ranges = [v if v > 0.0 else float('inf') for v in rr]
        m.intensities = list(ii)
        self.pub.publish(m)
        return True

def main():
    rclpy.init()
    node = LD06()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try: os.close(node.fd)
        except Exception: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
