import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# Demo poses: (label, [joint1, joint2, joint3, joint4] in radians, duration_sec)
DEMO_SEQUENCE = [
    ('Home', [0.0, 0.0, 0.0, 0.0], 2.0),
    ('Reach Right', [0.8, 0.45, -0.55, 0.30], 2.5),
    ('Lift & Inspect', [0.8, -0.25, 0.60, -0.35], 2.5),
    ('Sweep Left', [-0.8, 0.40, -0.50, 0.25], 3.0),
    ('Wave End-Effector', [-0.8, 0.20, 0.40, -0.60], 2.0),
    ('Return Home', [0.0, 0.0, 0.0, 0.0], 2.5),
]


class TrajectoryPublisher(Node):
    """Publishes JointTrajectory commands to /arm_controller/joint_trajectory."""

    def __init__(self) -> None:
        super().__init__('trajectory_publisher')
        self.declare_parameter('period_sec', 3.5)
        self.declare_parameter('loop', True)

        self.period_sec = float(self.get_parameter('period_sec').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.step_index = 0
        self.timer = self.create_timer(self.period_sec, self._publish_next)
        self.get_logger().info(
            'trajectory_publisher started: publishing to /arm_controller/joint_trajectory'
        )

    def _publish_next(self) -> None:
        if self.step_index >= len(DEMO_SEQUENCE):
            if self.loop:
                self.step_index = 0
            else:
                self.get_logger().info('Demo sequence complete.')
                self.timer.cancel()
                return

        label, positions, duration_sec = DEMO_SEQUENCE[self.step_index]
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points = [point]

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published [{label}]: positions={positions}, time_from_start={duration_sec:.1f}s'
        )
        self.step_index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
