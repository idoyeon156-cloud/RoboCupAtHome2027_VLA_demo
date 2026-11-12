"""Animate cereal waypoints on the RViz demo joint-state topic."""
import json
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
        self.timeline = None
        self.scene = None
        if config.get('version') == 2:
            from .cereal_timeline import Timeline
            from .cereal_planning import object_definitions
            from .distance import DistanceModel
            from visualization_msgs.msg import MarkerArray
            from std_msgs.msg import String
            import pinocchio as pin
            self.pin = pin
            self.timeline = Timeline(config)
            self.task = config
            self.objects = object_definitions(config['settings'])
            self.tcp = np.asarray(config['settings']['tool']['tcp_in_gripper_base_m'])
            model_share = Path(get_package_share_directory('piper_dual_description'))
            self.fk = DistanceModel(model_share/'urdf/dual_piper.urdf')
            self.marker_publisher = self.create_publisher(MarkerArray, '/cereal_task/objects', 1)
            self.phase_publisher = self.create_publisher(String, '/cereal_task/status', 1)
            self.names = [f'{side}_joint{i}' for side in SIDES for i in range(1, 9)]
            self.duration = self.timeline.duration
            self.started = time.monotonic()
            self.publisher = self.create_publisher(JointState, 'joint_states', 1)
            self.create_timer(TICK_S, self.tick)
            self.get_logger().info(f'Planned alpha/pick/pour loop: {self.duration:.1f}s, {len(config["waypoints"])} waypoints')
            return
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
        if self.timeline is not None:
            q, phase, held = self.timeline.sample(time.monotonic()-self.started)
            self.publish_objects(q, phase, held)
            self.publish_joints(q)
            return
        elapsed = (time.monotonic()-self.started) % self.duration
        for first, last, duration in self.segments:
            if elapsed < duration:
                q = first + (last-first)*(elapsed/duration)
                break
            elapsed -= duration
        else:
            q = self.segments[0][0]
        self.publish_joints(q)

    def publish_joints(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = np.column_stack((q[:, :6], q[:, 6]/2., -q[:, 6]/2.)).flatten().tolist()
        self.publisher.publish(msg)



    def publish_objects(self, q, phase, held):
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import String
        from scipy.spatial.transform import Rotation
        state = self.fk.configuration(q[0,:6],q[1,:6])
        self.pin.framesForwardKinematics(self.fk.model,self.fk.data,state)
        array = MarkerArray()
        for index, (name,item) in enumerate(self.objects.items()):
            pose=item['pose'].copy()
            if name in held:
                frame=self.fk.data.oMf[self.fk.model.getFrameId(item['side']+'_gripper_base')]
                pose=frame.homogeneous@np.asarray(self.task['object_attachments'][name])
            def marker(number,kind):
                m=Marker();m.header.frame_id='world';m.header.stamp=self.get_clock().now().to_msg()
                m.ns=name;m.id=number;m.type=kind;m.action=Marker.ADD
                m.pose.orientation.w=1.;m.color.a=.9
                m.color.r,m.color.g,m.color.b=(.25,.65,.95) if name=='bowl' else (.95,.65,.2)
                return m
            if name=='box':
                m=marker(0,Marker.CUBE);centre=pose[:3,3]+pose[:3,:3]@np.array([0,0,item['size'][2]/2])
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,centre)
                quat=Rotation.from_matrix(pose[:3,:3]).as_quat();m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,quat)
                m.scale.x,m.scale.y,m.scale.z=map(float,item['size']);array.markers.append(m)
            else:
                # Hollow bowl for viewing; collision checking uses a conservative solid cylinder.
                m=marker(0,Marker.TRIANGLE_LIST);m.scale.x=m.scale.y=m.scale.z=1.
                r=item['size'][0]/2;h=item['size'][2];thickness=.003
                for a,b in zip(np.linspace(0,2*np.pi,65)[:-1],np.linspace(0,2*np.pi,65)[1:]):
                    corners=[np.array([r*np.cos(a),r*np.sin(a),0]),np.array([r*np.cos(b),r*np.sin(b),0]),np.array([r*np.cos(a),r*np.sin(a),h]),np.array([r*np.cos(b),r*np.sin(b),h])]
                    for ids in ((0,1,2),(1,3,2)):
                        for i in ids:
                            p=pose[:3,:3]@corners[i]+pose[:3,3];m.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
                    for p in (np.array([0,0,thickness]),corners[0]+[0,0,thickness],corners[1]+[0,0,thickness]):
                        p=pose[:3,:3]@p+pose[:3,3];m.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
                array.markers.append(m)
            label=marker(1,Marker.TEXT_VIEW_FACING);label.color.r=label.color.g=label.color.b=1.;label.scale.z=.025
            label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,pose[:3,3]+[0,0,item['size'][2]+.04])
            label.text=name+(' (held)' if name in held else ' (placed)');array.markers.append(label)
        status=Marker();status.header.frame_id='world';status.ns='task_phase';status.id=0;status.type=Marker.TEXT_VIEW_FACING;status.action=Marker.ADD
        status.pose.orientation.w=1.;status.pose.position.z=.8;status.scale.z=.03;status.color.r=status.color.g=status.color.b=status.color.a=1.
        status.text='Cereal: '+phase;array.markers.append(status)
        self.marker_publisher.publish(array)
        self.phase_publisher.publish(String(data=json.dumps(dict(phase=phase,held_objects=list(held),duration_s=self.duration))))


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
