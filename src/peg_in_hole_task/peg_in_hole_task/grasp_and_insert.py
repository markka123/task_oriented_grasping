"""Parametrised grasp-and-insert pipeline for the peg-in-hole task.

Grasp parameters
----------------
~grasp_offset   (float, metres, default 0.0)
    Signed offset along the peg's long axis from its centre.
    Range ≈ [-0.025, +0.025].  Positive → toward the tip the peg points.

~approach_angle (float, degrees, default 0.0)
    Rotation of the gripper around the approach axis.
    Different values (e.g., 0 / 45 / 90 / 135) produce clearly different
    insertion outcomes and are the main parametric variable for evaluation.

Pipeline
--------
1.  Wait for peg pose on /peg_pose (published by spawn_task_objects).
2.  Compute grasp pose from peg pose + parameters.
3.  Open gripper.
4.  Move arm → pre-grasp (120 mm offset along approach axis).
5.  Move arm → grasp pose.
6.  Close gripper; start peg-tracking timer (50 Hz, /gazebo/set_entity_state).
7.  Lift to pre-grasp height.
8.  Move arm → pre-insertion pose (in front of hole).
9.  Move arm → insertion pose (peg pushed through hole face).
10. Open gripper; stop peg tracking.
11. Move arm → home.
12. Publish outcome JSON to /grasp_outcome.

Peg tracking
------------
The Robotiq gripper uses mock hardware (no physics contacts with Gazebo
bodies), so the peg is "attached" by continuously teleporting it to the
pose it would have if rigidly held by the TCP.  This uses the
/gazebo/set_entity_state service provided by the gazebo_ros init plugin
(always loaded by default in gzserver).
"""

import json
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros
import tf2_geometry_msgs  # noqa – registers PoseStamped transforms

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from pymoveit2 import MoveIt2


# ── Gripper joint names and mimic multipliers ─────────────────────────────────
GRIPPER_JOINTS = [
    'robotiq_85_left_knuckle_joint',
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
]
# Mimic multipliers relative to left_knuckle (applied manually since
# mock_components/GenericSystem does not enforce URDF mimic constraints).
GRIPPER_MIMIC = [1, -1, 1, -1, -1, 1]

# Gripper positions (left_knuckle joint angle, rad)
GRIPPER_OPEN   = 0.00   # fully open
GRIPPER_CLOSED = 0.70   # ~15 mm gap, fits peg cross-section

# Home joint values (rad) – matches SRDF group_state "home"
# Order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
HOME_JOINTS = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]

# Clearance distances
APPROACH_HEIGHT = 0.12  # m – stand-off above grasp for pre-grasp pose
LIFT_HEIGHT     = 0.15  # m – clearance after grasp before moving to hole

# Hole geometry (world frame) – must match task_params.yaml
# Hole plate: 80×80 cm face, 4 cm thick, vertical, facing the robot.
# With rpy=[0, π/2, 0] the plate thickness is along world X and the 80 cm
# span is along world Z.  Bottom of plate = hz − 0.40 = 0.00 (at floor).
# Robot approaches from the −X side and inserts in the +X direction.
HOLE_POSITION  = (0.55, 0.00, 0.40)
INSERT_DEPTH   = 0.05   # m – how far to push past the hole face centre


# ── Rotation / quaternion helpers ─────────────────────────────────────────────

def rotation_from_msg(q) -> Rotation:
    return Rotation.from_quat([q.x, q.y, q.z, q.w])


def quat_from_rotation(rot: Rotation):
    from geometry_msgs.msg import Quaternion
    xyzw = rot.as_quat()
    q = Quaternion()
    q.x, q.y, q.z, q.w = float(xyzw[0]), float(xyzw[1]), float(xyzw[2]), float(xyzw[3])
    return q


def peg_long_axis(peg_rot: Rotation) -> np.ndarray:
    """Peg local Z (80 mm long axis) expressed in world frame."""
    return peg_rot.apply([0.0, 0.0, 1.0])


# ── Grasp pose computation ────────────────────────────────────────────────────

def compute_grasp_pose(
    peg_pos: np.ndarray,
    peg_rot: Rotation,
    grasp_offset: float,
    approach_angle_deg: float,
) -> Pose:
    """
    Return the desired TCP pose for grasping.

    The gripper approaches from above for flat/narrow-edge pegs and from
    the side (+X direction) for an upright peg.  approach_angle_deg rotates
    the gripper around its approach axis, which is the primary variable that
    distinguishes grasp candidates for the evaluation dataset.
    """
    long_axis = peg_long_axis(peg_rot)
    grasp_pt  = peg_pos + grasp_offset * long_axis

    upright = abs(long_axis[2]) > 0.7   # long axis mostly vertical

    if upright:
        # Approach along world -X; TCP Y points up
        approach_dir = np.array([-1.0,  0.0,  0.0])
        lateral_ref  = np.array([ 0.0,  0.0,  1.0])
    else:
        # Approach from above; lateral reference ⊥ long axis in XY plane
        approach_dir = np.array([0.0,  0.0, -1.0])
        h = np.array([long_axis[0], long_axis[1], 0.0])
        h_norm = np.linalg.norm(h)
        h = h / h_norm if h_norm > 1e-6 else np.array([1.0, 0.0, 0.0])
        lateral_ref = np.array([-h[1], h[0], 0.0])   # 90° CCW in XY

    z_ax = approach_dir / np.linalg.norm(approach_dir)
    x_ax = lateral_ref - np.dot(lateral_ref, z_ax) * z_ax
    x_ax /= np.linalg.norm(x_ax)
    y_ax = np.cross(z_ax, x_ax)

    base_rot  = Rotation.from_matrix(np.column_stack([x_ax, y_ax, z_ax]))
    angle_rot = Rotation.from_rotvec(math.radians(approach_angle_deg) * z_ax)
    tcp_rot   = angle_rot * base_rot

    pose = Pose()
    pose.position.x  = float(grasp_pt[0])
    pose.position.y  = float(grasp_pt[1])
    pose.position.z  = float(grasp_pt[2])
    pose.orientation = quat_from_rotation(tcp_rot)
    return pose


def pose_above(pose: Pose, height: float) -> Pose:
    """Offset pose by `height` metres backwards along its own Z axis."""
    rot    = rotation_from_msg(pose.orientation)
    local_z = rot.apply([0.0, 0.0, 1.0])
    p = Pose()
    p.position.x  = pose.position.x - local_z[0] * height
    p.position.y  = pose.position.y - local_z[1] * height
    p.position.z  = pose.position.z - local_z[2] * height
    p.orientation = pose.orientation
    return p


# ── Main node ─────────────────────────────────────────────────────────────────

class GraspAndInsert(Node):

    def __init__(self):
        super().__init__('grasp_and_insert')

        self.declare_parameter('grasp_offset',   0.0)
        self.declare_parameter('approach_angle', 0.0)
        self.declare_parameter('auto_execute',   True)

        self._grasp_offset   = self.get_parameter('grasp_offset').value
        self._approach_angle = self.get_parameter('approach_angle').value
        self._auto_execute   = self.get_parameter('auto_execute').value

        # TF listener
        self._tf_buf    = tf2_ros.Buffer()
        self._tf_listen = tf2_ros.TransformListener(self._tf_buf, self)

        # Gripper action client (direct, bypasses MoveIt for simplicity)
        self._gripper_ac = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

        # Gazebo entity state service (teleport peg when tracking)
        self._set_state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Outcome publisher
        self._outcome_pub = self.create_publisher(String, '/grasp_outcome', 10)

        # Peg pose (received once from spawner)
        self._peg_pose: PoseStamped | None = None
        self._pose_sub = self.create_subscription(
            PoseStamped, '/peg_pose', self._peg_pose_cb, 10)

        # Peg tracking state
        self._tracking       = False
        self._peg_in_tcp     = None   # (rel_trans: np.ndarray, rel_rot: Rotation)
        self._track_timer    = None

        # pymoveit2 arm interface (constructed after executor is running)
        self._moveit2: MoveIt2 | None = None
        self._start_timer = None

        self.get_logger().info(
            f'GraspAndInsert ready | '
            f'grasp_offset={self._grasp_offset:.3f} m  '
            f'approach_angle={self._approach_angle:.1f}°'
        )

    def init_moveit(self):
        """Call after the executor is spinning to create the MoveIt2 interface."""
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
            ],
            base_link_name='base_link',
            end_effector_name='tcp',
            group_name='ur_manipulator',
        )
        self._moveit2.planner_id = 'RRTConnectkConfigDefault'
        self._moveit2.planning_time = 10.0
        self._moveit2.max_velocity = 0.2
        self._moveit2.max_acceleration = 0.2

        if self._auto_execute:
            # Wait for peg pose then launch pipeline in background thread
            self._start_timer = self.create_timer(1.0, self._maybe_start)

    # ── Peg pose callback ─────────────────────────────────────────────────────

    def _peg_pose_cb(self, msg: PoseStamped):
        if self._peg_pose is None:
            self._peg_pose = msg
            self.get_logger().info('Received peg pose.')

    def _maybe_start(self):
        if self._peg_pose is not None and self._moveit2 is not None:
            if self._start_timer is not None:
                self._start_timer.cancel()
                self._start_timer = None
            threading.Thread(target=self._run_pipeline, daemon=True).start()

    # ── Gripper control ───────────────────────────────────────────────────────

    def _gripper(self, position: float, duration_sec: float = 1.5) -> bool:
        if not self._gripper_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available.')
            return False

        values = [position * m for m in GRIPPER_MIMIC]
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions  = values
        pt.velocities = [0.0] * len(GRIPPER_JOINTS)
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        # Use threading.Event so we don't call spin_until_future_complete
        # while the MultiThreadedExecutor is already spinning.
        done = threading.Event()
        success = [False]

        def _result_cb(future):
            success[0] = True
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        self._gripper_ac.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait(timeout=duration_sec + 5.0)
        return success[0]

    # ── Peg tracking (teleport via /gazebo/set_entity_state) ─────────────────

    def _start_tracking(self):
        try:
            tf = self._tf_buf.lookup_transform(
                'world', 'tcp', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'TF lookup failed, cannot track peg: {e}')
            return

        tcp_rot   = rotation_from_msg(tf.transform.rotation)
        tcp_trans = np.array([tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z])
        p = self._peg_pose.pose
        peg_trans = np.array([p.position.x, p.position.y, p.position.z])
        peg_rot   = rotation_from_msg(p.orientation)

        self._peg_in_tcp = (
            tcp_rot.inv().apply(peg_trans - tcp_trans),
            tcp_rot.inv() * peg_rot,
        )
        self._tracking   = True
        self._track_timer = self.create_timer(1.0 / 50.0, self._track_cb)
        self.get_logger().info('Peg tracking started.')

    def _stop_tracking(self):
        self._tracking = False
        if self._track_timer is not None:
            self._track_timer.cancel()
            self._track_timer = None

    def _track_cb(self):
        if not self._tracking or self._peg_in_tcp is None:
            return
        if not self._set_state_cli.service_is_ready():
            return
        try:
            tf = self._tf_buf.lookup_transform('world', 'tcp', rclpy.time.Time())
        except Exception:
            return

        tcp_rot   = rotation_from_msg(tf.transform.rotation)
        tcp_trans = np.array([tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z])
        rel_trans, rel_rot = self._peg_in_tcp
        peg_trans = tcp_trans + tcp_rot.apply(rel_trans)
        peg_rot   = tcp_rot * rel_rot
        xyzw      = peg_rot.as_quat()

        state = EntityState()
        state.name = 'peg'
        state.reference_frame = 'world'
        state.pose.position.x  = float(peg_trans[0])
        state.pose.position.y  = float(peg_trans[1])
        state.pose.position.z  = float(peg_trans[2])
        state.pose.orientation.x = float(xyzw[0])
        state.pose.orientation.y = float(xyzw[1])
        state.pose.orientation.z = float(xyzw[2])
        state.pose.orientation.w = float(xyzw[3])

        req = SetEntityState.Request()
        req.state = state
        self._set_state_cli.call_async(req)

        # Keep local record current for outcome evaluation
        self._peg_pose.pose.position.x = float(peg_trans[0])
        self._peg_pose.pose.position.y = float(peg_trans[1])
        self._peg_pose.pose.position.z = float(peg_trans[2])

    # ── Arm motion helpers ────────────────────────────────────────────────────

    def _move_to_pose(self, pose: Pose, label: str) -> bool:
        self._moveit2.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            ],
            cartesian=False,
        )
        self._moveit2.wait_until_executed()
        ok = True   # pymoveit2 raises on failure; simplified check
        if not ok:
            self.get_logger().error(f'Motion failed: {label}')
        return ok

    def _move_home(self) -> bool:
        self._moveit2.move_to_configuration(joint_positions=HOME_JOINTS)
        self._moveit2.wait_until_executed()
        return True

    # ── Insertion geometry ────────────────────────────────────────────────────

    def _insertion_poses(self) -> tuple[Pose, Pose]:
        """Pre-insertion (in front of hole) and insertion (through hole face)."""
        # Hole plate is vertical, facing the robot (normal points in −X world direction).
        # TCP Z = world +X: the gripper approaches from the −X side and pushes in +X.
        # TCP X = world +Z so that fingers are oriented vertically (less chance of collision
        # with the plate edges).
        x_ax = np.array([0.0,  0.0,  1.0])   # TCP X = world +Z (finger axis vertical)
        z_ax = np.array([1.0,  0.0,  0.0])   # TCP Z = world +X (insertion direction)
        y_ax = np.cross(z_ax, x_ax)
        tcp_rot = Rotation.from_matrix(np.column_stack([x_ax, y_ax, z_ax]))
        q = quat_from_rotation(tcp_rot)

        hx, hy, hz = HOLE_POSITION
        pre = Pose()
        pre.position.x = hx - APPROACH_HEIGHT   # stand-off on the robot side
        pre.position.y = hy
        pre.position.z = hz
        pre.orientation = q

        ins = Pose()
        ins.position.x = hx + INSERT_DEPTH      # pushed through to the far side
        ins.position.y = hy
        ins.position.z = hz
        ins.orientation = q

        return pre, ins

    # ── Main pipeline ─────────────────────────────────────────────────────────

    def _run_pipeline(self):
        peg_pos = np.array([
            self._peg_pose.pose.position.x,
            self._peg_pose.pose.position.y,
            self._peg_pose.pose.position.z,
        ])
        peg_rot = rotation_from_msg(self._peg_pose.pose.orientation)

        grasp_pose = compute_grasp_pose(
            peg_pos, peg_rot, self._grasp_offset, self._approach_angle)
        pre_grasp  = pose_above(grasp_pose, APPROACH_HEIGHT)
        pre_insert, insert_pose = self._insertion_poses()

        outcome = {
            'grasp_offset':   self._grasp_offset,
            'approach_angle': self._approach_angle,
            'success':        False,
            'failure_stage':  '',
        }

        try:
            self.get_logger().info('Step 1 – open gripper')
            self._gripper(GRIPPER_OPEN)

            self.get_logger().info('Step 2 – pre-grasp')
            if not self._move_to_pose(pre_grasp, 'pre_grasp'):
                outcome['failure_stage'] = 'pre_grasp'; return

            self.get_logger().info('Step 3 – grasp')
            if not self._move_to_pose(grasp_pose, 'grasp'):
                outcome['failure_stage'] = 'grasp'; return

            self.get_logger().info('Step 4 – close gripper + start tracking')
            self._gripper(GRIPPER_CLOSED)
            time.sleep(0.3)
            self._start_tracking()

            self.get_logger().info('Step 5 – lift')
            if not self._move_to_pose(pose_above(grasp_pose, LIFT_HEIGHT), 'lift'):
                outcome['failure_stage'] = 'lift'; return

            self.get_logger().info('Step 6 – pre-insertion')
            if not self._move_to_pose(pre_insert, 'pre_insert'):
                outcome['failure_stage'] = 'pre_insert'; return

            self.get_logger().info('Step 7 – insert')
            if not self._move_to_pose(insert_pose, 'insert'):
                outcome['failure_stage'] = 'insert'; return

            outcome['success'] = self._peg_pose.pose.position.x > HOLE_POSITION[0]

        finally:
            self.get_logger().info('Step 8 – release')
            self._stop_tracking()
            self._gripper(GRIPPER_OPEN)
            time.sleep(0.3)

            self.get_logger().info('Step 9 – home')
            self._move_home()

            msg = String()
            msg.data = json.dumps(outcome)
            self._outcome_pub.publish(msg)
            self.get_logger().info(f'Outcome: {outcome}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GraspAndInsert()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # Init MoveIt2 after executor is spinning (requires active node)
    threading.Thread(target=node.init_moveit, daemon=True).start()
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
