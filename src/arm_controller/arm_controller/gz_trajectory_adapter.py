import math
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
JOINT_LIMIT = math.pi / 2.0  # +/- 1.5708 rad


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class GzTrajectoryAdapter(Node):
    """Subscribes to /arm_controller/joint_trajectory and interpolates smooth
    position setpoints to Gazebo's JointPositionController topics."""

    def __init__(self) -> None:
        super().__init__('gz_trajectory_adapter')

        self.declare_parameter('publish_joint_states_fallback', False)
        self.publish_fallback = bool(
            self.get_parameter('publish_joint_states_fallback').value
        )

        self.cmd_pubs: Dict[str, any] = {
            name: self.create_publisher(Float64, f'/arm_gz/{name}/cmd_pos', 10)
            for name in JOINT_NAMES
        }

        self.js_fallback_pub = None
        if self.publish_fallback:
            self.js_fallback_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.current_positions: Dict[str, float] = {name: 0.0 for name in JOINT_NAMES}
        self.commanded_positions: Dict[str, float] = {name: 0.0 for name in JOINT_NAMES}

        # Active trajectory: (start_wall_time, list of (t_sec, {joint: pos}))
        self.traj_start_time: float = 0.0
        self.traj_start_positions: Dict[str, float] = dict(self.current_positions)
        self.traj_waypoints: List[Tuple[float, Dict[str, float]]] = []

        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._on_trajectory,
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            10,
        )

        # 50 Hz interpolation loop
        self.create_timer(0.02, self._on_timer)
        self.get_logger().info(
            'gz_trajectory_adapter ready: listening on /arm_controller/joint_trajectory'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = float(pos)

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warning('Received empty JointTrajectory; ignoring.')
            return

        waypoints: List[Tuple[float, Dict[str, float]]] = []
        for pt in msg.points:
            t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            targets = dict(self.commanded_positions)
            for name, pos in zip(msg.joint_names, pt.positions):
                if name in targets:
                    targets[name] = clamp(float(pos), -JOINT_LIMIT, JOINT_LIMIT)
            waypoints.append((max(0.0, t_sec), targets))

        waypoints.sort(key=lambda item: item[0])
        self.traj_start_time = time.monotonic()
        self.traj_start_positions = dict(self.commanded_positions)
        self.traj_waypoints = waypoints

        last_t, last_pos = waypoints[-1]
        self.get_logger().info(
            f'Executing trajectory with {len(waypoints)} point(s) '
            f'(duration={last_t:.2f}s, final={[round(last_pos[j], 3) for j in JOINT_NAMES]})'
        )

    def _on_timer(self) -> None:
        if self.traj_waypoints:
            elapsed = time.monotonic() - self.traj_start_time
            prev_t = 0.0
            prev_pos = self.traj_start_positions
            done = True

            for wp_t, wp_pos in self.traj_waypoints:
                if elapsed <= wp_t:
                    done = False
                    if wp_t <= prev_t:
                        alpha = 1.0
                    else:
                        alpha = clamp((elapsed - prev_t) / (wp_t - prev_t), 0.0, 1.0)
                    for name in JOINT_NAMES:
                        self.commanded_positions[name] = (
                            prev_pos[name] + alpha * (wp_pos[name] - prev_pos[name])
                        )
                    break
                prev_t = wp_t
                prev_pos = wp_pos

            if done:
                self.commanded_positions = dict(self.traj_waypoints[-1][1])
                self.traj_waypoints = []

        for name in JOINT_NAMES:
            out = Float64()
            out.data = self.commanded_positions[name]
            self.cmd_pubs[name].publish(out)

        if self.js_fallback_pub is not None:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(JOINT_NAMES)
            js.position = [self.commanded_positions[n] for n in JOINT_NAMES]
            self.js_fallback_pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzTrajectoryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
