"""Spawns the hole block (fixed) and peg (random stable pose) into Gazebo.

Random peg pose
---------------
  XY   – uniform inside a reachable workspace footprint
  Yaw  – uniform in [0, 2π]
  Face – one of three stable ground-contact orientations (chosen uniformly):
           0  upright  : long axis vertical,   80 mm tall  (RPY = [0, 0, θ])
           1  narrow   : 10×80 mm face down,   28 mm tall  (RPY = [0, π/2, θ])
           2  flat     : 28×80 mm face down,   10 mm tall  (RPY = [π/2, 0, θ])

After both objects are spawned the node publishes the peg pose to /peg_pose
(geometry_msgs/PoseStamped, frame: world) so grasp_and_insert.py can use it.
"""

import math
import os
import random
import time

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped


# ── Peg stable faces ─────────────────────────────────────────────────────────
# Peg SDF box:  X=0.028 m  Y=0.010 m  Z=0.080 m
# Each tuple: (roll, pitch, z_center_above_ground)
PEG_FACES = [
    (0.0,         0.0,         0.040),   # 0 – upright:  Z up,  h=80 mm
    (0.0,         math.pi / 2, 0.014),   # 1 – narrow:   X up,  h=28 mm
    (math.pi / 2, 0.0,         0.005),   # 2 – flat:     Y up,  h=10 mm
]

# Reachable workspace for random peg XY (robot base at world origin)
PEG_X_RANGE = (0.30, 0.55)
PEG_Y_RANGE = (-0.30, 0.30)


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    """Convert RPY (rad) to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


class TaskSpawner(Node):

    def __init__(self):
        super().__init__('peg_in_hole_spawner')

        self._pub = self.create_publisher(PoseStamped, '/peg_pose', 10)
        self._cli = self.create_client(SpawnEntity, '/spawn_entity')

        pkg = get_package_share_directory('peg_in_hole_task')
        params_path = os.path.join(pkg, 'config', 'task_params.yaml')
        with open(params_path) as f:
            params = yaml.safe_load(f)

        with open(os.path.join(pkg, 'models', 'hole_block', 'model.sdf')) as f:
            hole_sdf = f.read()
        with open(os.path.join(pkg, 'models', 'peg', 'model.sdf')) as f:
            peg_sdf = f.read()

        # Hole: fixed pose from config
        hole_pose = self._pose_from_cfg(params['hole'])

        # Peg: randomised
        peg_pose, self._peg_stamped = self._random_peg_pose()
        self.get_logger().info(
            f'Peg random pose → '
            f'x={peg_pose.position.x:.3f}  '
            f'y={peg_pose.position.y:.3f}  '
            f'z={peg_pose.position.z:.3f}'
        )

        self.get_logger().info('Waiting for /spawn_entity ...')
        self._cli.wait_for_service(timeout_sec=30.0)

        ok = self._spawn('hole_block', hole_sdf, hole_pose)
        ok = self._spawn('peg', peg_sdf, peg_pose) and ok

        if ok:
            time.sleep(0.5)   # let Gazebo settle peg onto ground
            self._peg_stamped.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._peg_stamped)
            self.get_logger().info('Spawned all objects. Peg pose → /peg_pose')
        else:
            self.get_logger().error('Failed to spawn one or more objects.')

        self.create_timer(0.5, lambda: rclpy.shutdown())

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _random_peg_pose():
        x    = random.uniform(*PEG_X_RANGE)
        y    = random.uniform(*PEG_Y_RANGE)
        yaw  = random.uniform(0.0, 2.0 * math.pi)
        face = random.randint(0, len(PEG_FACES) - 1)
        roll, pitch, z = PEG_FACES[face]

        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        stamped = PoseStamped()
        stamped.header.frame_id = 'world'
        stamped.pose = pose
        return pose, stamped

    @staticmethod
    def _pose_from_cfg(cfg: dict) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in cfg['xyz']]
        qx, qy, qz, qw = rpy_to_quat(*[float(v) for v in cfg['rpy']])
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _spawn(self, name: str, xml: str, pose: Pose) -> bool:
        req = SpawnEntity.Request()
        req.name           = name
        req.xml            = xml
        req.initial_pose   = pose
        req.robot_namespace = ''
        req.reference_frame = 'world'

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error(f"Spawn service call timed out for '{name}'.")
            return False
        if not future.result().success:
            self.get_logger().error(
                f"Spawn failed for '{name}': {future.result().status_message}")
            return False
        self.get_logger().info(f"Spawned '{name}'.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TaskSpawner()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
