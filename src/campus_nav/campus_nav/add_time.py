#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class AddTime(Node):
    def __init__(self):
        super().__init__('add_time')
        self.declare_parameter('scan_period', 0.1)
        self.period = self.get_parameter('scan_period').value
        self.pub = self.create_publisher(PointCloud2, '/cloud_timed', 10)
        self.create_subscription(PointCloud2, '/cloud', self.cb, 10)

    def cb(self, msg):
        f = {p.name: p.offset for p in msg.fields}
        in_dt = np.dtype({
            'names': ['x', 'y', 'z', 'intensity', 'ring'],
            'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
            'offsets': [f['x'], f['y'], f['z'], f['intensity'], f['ring']],
            'itemsize': msg.point_step,
        })
        pts = np.frombuffer(bytes(msg.data), dtype=in_dt)

        # drop NaN/inf points (Gazebo clouds are organized and contain them)
        ok = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        pts = pts[ok]
        n = pts.shape[0]
        if n == 0:
            return

        # per-point time offset (seconds) from azimuth: 0 .. scan_period
        t = np.full(n, 1e-6, dtype='<f4')

        out_dt = np.dtype({
            'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
            'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
            'offsets': [0, 4, 8, 12, 16, 20],
            'itemsize': 24,
        })
        out = np.zeros(n, dtype=out_dt)
        for k in ('x', 'y', 'z', 'intensity', 'ring'):
            out[k] = pts[k]
        out['time'] = t

        m = PointCloud2()
        m.header = msg.header
        m.height = 1
        m.width = n
        m.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        m.is_bigendian = False
        m.point_step = 24
        m.row_step = 24 * n
        m.is_dense = True
        m.data = out.tobytes()
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(AddTime())


if __name__ == '__main__':
    main()
