#!/usr/bin/env python3
"""
camera_watchdog_node — Monitor camera topic health and detect stalls.

Subscribes to configured camera image topics with BEST_EFFORT QoS so it
never back-pressures publishers. Publishes a JSON health report to
/camera/health every check_interval_s seconds. Logs WARN on stall
transitions and INFO on recovery. Does not restart camera processes
(leave that to a systemd watchdog or bringup-level node).
"""

import json
import time
import threading
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import String
from sensor_msgs.msg import Image


# BEST_EFFORT + VOLATILE: never block a publisher, never queue stale frames.
_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class _CamState:
    """Per-topic state, thread-safe."""

    def __init__(self, topic: str, stall_sec: float):
        self.topic = topic
        self.stall_sec = stall_sec
        self._lock = threading.Lock()
        self._last_ts: Optional[float] = None
        self._frames = 0
        self._stall_count = 0
        self._was_stalled = True   # start as "stalled until proven otherwise"

    # Called from ROS subscription callback (executor thread)
    def on_frame(self):
        with self._lock:
            self._last_ts = time.monotonic()
            self._frames += 1

    # Called from timer callback (executor thread)
    def check(self) -> tuple[bool, bool]:
        """Return (is_stalled, newly_changed)."""
        now = time.monotonic()
        with self._lock:
            if self._last_ts is None:
                stalled = True
            else:
                stalled = (now - self._last_ts) > self.stall_sec
            changed = stalled != self._was_stalled
            if changed:
                if stalled:
                    self._stall_count += 1
                self._was_stalled = stalled
            return stalled, changed

    def to_dict(self) -> dict:
        now = time.monotonic()
        with self._lock:
            age = round(now - self._last_ts, 2) if self._last_ts else None
            return {
                'topic': self.topic,
                'frames': self._frames,
                'stalls': self._stall_count,
                'last_frame_age_s': age,
                'stalled': self._was_stalled,
            }


class CameraWatchdogNode(Node):
    """Watch a configurable list of camera topics for frame stalls."""

    # Covers Kinect2 (hd/sd), RealSense (color/depth), and generic USB cams.
    _DEFAULT_TOPICS = [
        '/kinect2/hd/image_color_rect',
        '/kinect2/sd/image_depth_rect',
        '/camera/color/image_raw',
        '/camera/depth/image_rect_raw',
        '/usb_cam/image_raw',
        '/camera/image_raw',
    ]

    def __init__(self):
        super().__init__('camera_watchdog_node')

        self.declare_parameter('stall_timeout_s',  5.0)
        self.declare_parameter('check_interval_s', 2.0)
        self.declare_parameter('topics',           self._DEFAULT_TOPICS)

        stall_sec      = self.get_parameter('stall_timeout_s').value
        check_interval = self.get_parameter('check_interval_s').value
        topics         = self.get_parameter('topics').value

        self._cameras: Dict[str, _CamState] = {}
        self._health_pub = self.create_publisher(String, '/camera/health', 10)

        for topic in topics:
            state = _CamState(topic, stall_sec)
            self._cameras[topic] = state
            # Lambda captures topic by value (default arg trick)
            self.create_subscription(
                Image, topic,
                lambda _msg, t=topic: self._cameras[t].on_frame(),
                _BE_QOS,
            )

        self.create_timer(check_interval, self._tick)

        self.get_logger().info(
            f'camera_watchdog_node ready — monitoring {len(topics)} topics, '
            f'stall timeout {stall_sec}s'
        )

    def _tick(self):
        all_dicts = []
        for state in self._cameras.values():
            stalled, changed = state.check()
            d = state.to_dict()
            all_dicts.append(d)

            if changed:
                if stalled:
                    self.get_logger().warn(
                        f'Camera STALL: {state.topic} '
                        f'(total stalls: {d["stalls"]}, '
                        f'frames received: {d["frames"]})'
                    )
                else:
                    self.get_logger().info(
                        f'Camera RECOVERED: {state.topic} '
                        f'(frames: {d["frames"]})'
                    )

        active     = sum(1 for d in all_dicts if not d['stalled'] and d['frames'] > 0)
        stalled    = sum(1 for d in all_dicts if d['stalled'] and d['frames'] > 0)
        never_seen = sum(1 for d in all_dicts if d['frames'] == 0)

        report = {
            'ts':         time.time(),
            'active':     active,
            'stalled':    stalled,
            'never_seen': never_seen,
            'cameras':    all_dicts,
        }
        msg = String()
        msg.data = json.dumps(report)
        self._health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
