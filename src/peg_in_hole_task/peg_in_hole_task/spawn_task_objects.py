import os
import time
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    """Convert RPY (rad) to quaternion (x,y,z,w)."""
    import math
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class TaskSpawner(Node):
    def __init__(self):
        super().__init__("peg_in_hole_spawner")

        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Waiting for /spawn_entity service...")
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("...still waiting for /spawn_entity")

        pkg_share = get_package_share_directory("peg_in_hole_task")

        params_path = os.path.join(pkg_share, "config", "task_params.yaml")
        with open(params_path, "r") as f:
            params = yaml.safe_load(f)

        hole_sdf_path = os.path.join(pkg_share, "models", "hole_block", "model.sdf")
        peg_sdf_path = os.path.join(pkg_share, "models", "peg", "model.sdf")
        hole_xml = self._read_text(hole_sdf_path)
        peg_xml = self._read_text(peg_sdf_path)

        hole_pose = self._pose_from_cfg(params["hole"])
        peg_pose = self._pose_from_cfg(params["peg"])

        # Spawn fixture first, then peg
        ok_hole = self._spawn(name="hole_block", xml=hole_xml, pose=hole_pose)
        ok_peg = self._spawn(name="peg", xml=peg_xml, pose=peg_pose)

        if ok_hole and ok_peg:
            self.get_logger().info("✅ Spawned hole_block and peg.")
        else:
            self.get_logger().error("❌ Failed to spawn one or more entities (see logs).")

        time.sleep(0.2)
        rclpy.shutdown()

    @staticmethod
    def _read_text(path: str) -> str:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Missing file: {path}")
        with open(path, "r") as f:
            return f.read()

    @staticmethod
    def _pose_from_cfg(cfg: dict) -> Pose:
        pose = Pose()
        x, y, z = cfg["xyz"]
        r, p, yw = cfg["rpy"]
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        qx, qy, qz, qw = rpy_to_quat(float(r), float(p), float(yw))
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _spawn(self, name: str, xml: str, pose: Pose) -> bool:
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.initial_pose = pose
        req.robot_namespace = ""

        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error(f"Spawn service call failed for '{name}'.")
            return False

        res = future.result()
        if not res.success:
            self.get_logger().error(f"Spawn failed for '{name}': {res.status_message}")
            return False

        self.get_logger().info(f"Spawned '{name}': {res.status_message}")
        return True


def main():
    rclpy.init()
    TaskSpawner()


if __name__ == "__main__":
    main()
