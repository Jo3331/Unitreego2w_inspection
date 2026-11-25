#!/usr/bin/env python3
"""Convert native Unitree UTLIDAR PointCloud2 scans to Super-LIO's Livox-PC2 layout."""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


_FORMATS = {
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
    PointField.UINT8: 'u1',
    PointField.UINT16: 'u2',
    PointField.UINT32: 'u4',
    PointField.INT8: 'i1',
    PointField.INT16: 'i2',
    PointField.INT32: 'i4',
}
_OUT_DTYPE = np.dtype({
    'names': ('x', 'y', 'z', 'intensity', 'tag', 'timestamp'),
    'formats': ('<f4', '<f4', '<f4', '<f4', 'u1', '<f8'),
    'offsets': (0, 4, 8, 12, 16, 24),
    'itemsize': 32,
})


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _point_view(msg, fields):
    names, formats, offsets = [], [], []
    endian = '>' if msg.is_bigendian else '<'
    for name in fields:
        field = fields[name]
        if field.count != 1 or field.datatype not in _FORMATS:
            raise ValueError(f'unsupported field layout: {name}')
        code = _FORMATS[field.datatype]
        names.append(name)
        formats.append(code if code in ('u1', 'i1') else endian + code)
        offsets.append(field.offset)
    dtype = np.dtype({
        'names': names, 'formats': formats, 'offsets': offsets,
        'itemsize': msg.point_step,
    })
    count = msg.width * msg.height
    if count == 0 or len(msg.data) < count * msg.point_step:
        raise ValueError('empty or truncated UTLIDAR PointCloud2')
    return np.frombuffer(msg.data, dtype=dtype, count=count)


class UtlidarToSuperLio(Node):
    def __init__(self):
        super().__init__('utlidar_to_super_lio')
        self.declare_parameter('input_topic', '/utlidar/cloud')
        self.declare_parameter('output_topic', '/lio/utlidar_super_lio')
        self.declare_parameter('blind_range', 0.5)
        self.declare_parameter('max_point_time_sec', 0.25)
        self.declare_parameter('fallback_frame_id', 'utlidar_lidar')
        self.blind_range = max(0.0, float(self.get_parameter('blind_range').value))
        self.max_point_time = float(self.get_parameter('max_point_time_sec').value)
        self.fallback_frame = str(self.get_parameter('fallback_frame_id').value)
        input_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), output_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self.callback,
            input_qos,
        )
        self.input_count = 0
        self.output_count = 0
        self.point_count_in = 0
        self.point_count_out = 0
        self.last_report = time.monotonic()
        self.get_logger().info(
            f"UTLIDAR standalone adapter: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}; blind_range={self.blind_range:.2f} m"
        )

    def callback(self, msg):
        self.input_count += 1
        self.point_count_in += msg.width * msg.height
        try:
            fields = {field.name: field for field in msg.fields}
            for name in ('x', 'y', 'z', 'intensity', 'time'):
                if name not in fields:
                    raise ValueError(f'required field {name!r} missing')
            if any(fields[name].datatype != PointField.FLOAT32 for name in ('x', 'y', 'z')):
                raise ValueError('x/y/z must be float32')
            if fields['time'].datatype not in (PointField.FLOAT32, PointField.FLOAT64):
                raise ValueError('time must be floating-point seconds relative to scan')

            raw = _point_view(msg, {name: fields[name] for name in
                                    ('x', 'y', 'z', 'intensity', 'time')})
            xyz = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float64)
            intensity = raw['intensity'].astype(np.float32)
            point_time = raw['time'].astype(np.float64)
            valid = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity) & np.isfinite(point_time)
            valid &= (point_time >= 0.0) & (point_time <= self.max_point_time)
            valid &= np.einsum('ij,ij->i', xyz, xyz) > self.blind_range ** 2
            if not np.any(valid):
                return

            times_ns = _stamp_ns(msg.header.stamp) + np.rint(point_time[valid] * 1e9).astype(np.int64)
            order = np.argsort(times_ns, kind='stable')
            xyz = xyz[valid][order]
            intensity = intensity[valid][order]
            times_ns = times_ns[order]
            start_ns = int(times_ns[0])

            packed = np.empty(len(times_ns), dtype=_OUT_DTYPE)
            packed['x'], packed['y'], packed['z'] = xyz.T.astype(np.float32)
            packed['intensity'] = intensity
            packed['tag'] = 0x10
            packed['timestamp'] = times_ns.astype(np.float64)

            out = PointCloud2()
            out.header.stamp.sec, out.header.stamp.nanosec = divmod(start_ns, 1_000_000_000)
            out.header.frame_id = msg.header.frame_id or self.fallback_frame
            out.height = 1
            out.width = len(times_ns)
            out.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
                PointField(name='timestamp', offset=24, datatype=PointField.FLOAT64, count=1),
            ]
            out.is_bigendian = False
            out.point_step = 32
            out.row_step = out.point_step * out.width
            out.is_dense = False
            out.data = packed.tobytes()
            self.publisher.publish(out)
            self.output_count += 1
            self.point_count_out += out.width
        except (KeyError, ValueError, TypeError) as exc:
            self.get_logger().error(f'Rejecting UTLIDAR scan: {exc}', throttle_duration_sec=2.0)

        now = time.monotonic()
        if now - self.last_report >= 5.0:
            self.get_logger().info(
                f'UTLIDAR adapter totals: scans={self.input_count}, '
                f'published={self.output_count}, points_in={self.point_count_in}, '
                f'points_out={self.point_count_out}'
            )
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = UtlidarToSuperLio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
