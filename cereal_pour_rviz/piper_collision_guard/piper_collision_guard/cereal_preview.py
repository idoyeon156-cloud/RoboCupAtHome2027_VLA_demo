"""Animate cereal waypoints on the RViz demo joint-state topic."""
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from .cereal_pour import POUR_ORDER, SIDES, TICK_S, read_sequence


class CerealPreview(Node):
    def __init__(self):
        super().__init__('cereal_preview')
        share = Path(get_package_share_directory('piper_collision_guard'))
        self.declare_parameter('sequence_path', str(share/'config/cereal_pour.yaml'))
        config = yaml.safe_load(Path(self.get_parameter('sequence_path').value).read_text())
        poses, rates, pour_hold = read_sequence(config)
        self.names = [f'{side}_joint{i}' for side in SIDES for i in range(1, 9)]
        home = np.zeros_like(poses['ready'])
        left_ready = np.stack([poses['ready'][0], home[1]])
        playback = {'home': home, 'left_ready': left_ready, **poses}
        order = ('home', 'left_ready', *POUR_ORDER, 'left_ready', 'home')
        self.segments = [(home, home, 2.)]
        for previous, current in zip(order, order[1:]):
            first, last = playback[previous], playback[current]
            duration = float(np.max(np.abs(last-first)/rates))
            if duration > 0:
                self.segments.append((first, last, duration))
            if current == 'tilt_65' and pour_hold > 0:
                self.segments.append((last, last, pour_hold))
        self.duration = sum(duration for _, _, duration in self.segments)
        self.started = time.monotonic()
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.create_timer(TICK_S, self.tick)

    def tick(self):
        elapsed = (time.monotonic()-self.started) % self.duration
        for first, last, duration in self.segments:
            if elapsed < duration:
                q = first + (last-first)*(elapsed/duration)
                break
            elapsed -= duration
        else:
            q = self.segments[0][0]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = np.column_stack((q[:, :6], q[:, 6]/2., -q[:, 6]/2.)).flatten().tolist()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = CerealPreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
