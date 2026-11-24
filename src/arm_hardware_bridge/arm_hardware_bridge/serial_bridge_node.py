import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
except ImportError:
    serial = None


def rad_to_clamped_deg(
    rad: float,
    rad_min: float = -math.pi / 2.0,
    rad_max: float = math.pi / 2.0,
    deg_min: int = 0,
    deg_max: int = 180,
) -> int:
    """Convert joint angle in radians to a clamped integer servo angle [0, 180]."""
    clamped_rad = max(rad_min, min(rad_max, rad))
    ratio = (clamped_rad - rad_min) / (rad_max - rad_min)
    deg = deg_min + ratio * (deg_max - deg_min)
    return int(round(max(deg_min, min(deg_max, deg))))


class SerialBridgeNode(Node):
    """Subscribes to /arm_controller/joint_trajectory, converts joint angles from
    radians to clamped 0-180 degree servo commands, and sends CSV frames over USB
    serial to an Arduino:

        S,<servo1_deg>,<servo2_deg>\\n
    """

    def __init__(self) -> None:
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('servo_joints', ['joint1', 'joint2'])

        self.serial_port: str = str(self.get_parameter('serial_port').value)
        self.baudrate: int = int(self.get_parameter('baudrate').value)
        self.dry_run: bool = bool(self.get_parameter('dry_run').value)
        self.servo_joints: List[str] = list(self.get_parameter('servo_joints').value)

        self.ser: Optional['serial.Serial'] = None
        self.current_deg: Dict[str, int] = {j: 90 for j in self.servo_joints}

        # Scheduled waypoints: list of (target_monotonic_time, {joint: deg})
        self.pending_points: List[Tuple[float, Dict[str, int]]] = []

        if not self.dry_run:
            self._connect_serial()
        else:
            self.get_logger().info(
                'Running in dry_run mode (no physical serial port opened).'
            )

        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._on_trajectory,
            10,
        )
        self.create_timer(0.02, self._process_queue)

        self.get_logger().info(
            f'serial_bridge_node ready: mapping {self.servo_joints} -> '
            f'{self.serial_port} @ {self.baudrate} baud (dry_run={self.dry_run})'
        )

    def _connect_serial(self) -> None:
        if serial is None:
            self.get_logger().error(
                'pyserial is not installed! Install with: sudo apt install python3-serial'
            )
            return
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            # Arduino Uno/Nano/Mega resets when serial opens; wait briefly for bootloader
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(
                f'Connected to Arduino on {self.serial_port} @ {self.baudrate} baud.'
            )
        except Exception as exc:
            self.ser = None
            self.get_logger().warning(
                f'Could not open {self.serial_port}: {exc}. '
                'Will retry automatically when commands arrive.'
            )

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        now = time.monotonic()
        schedule: List[Tuple[float, Dict[str, int]]] = []

        for pt in msg.points:
            t_offset = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            deg_map = dict(self.current_deg)
            for name, rad in zip(msg.joint_names, pt.positions):
                if name in deg_map:
                    deg_map[name] = rad_to_clamped_deg(float(rad))
            # For single-point GUI/controller commands, send immediately so servos respond live
            if len(msg.points) == 1:
                schedule.append((now, deg_map))
            else:
                schedule.append((now + max(0.0, t_offset), deg_map))

        schedule.sort(key=lambda item: item[0])
        self.pending_points = schedule

    def _process_queue(self) -> None:
        if not self.pending_points:
            return

        now = time.monotonic()
        due: Optional[Dict[str, int]] = None
        while self.pending_points and self.pending_points[0][0] <= now:
            _, due = self.pending_points.pop(0)

        if due is not None:
            self.current_deg = due
            self._send_frame([due[j] for j in self.servo_joints])

    def _send_frame(self, angles_deg: List[int]) -> None:
        frame = 'S,' + ','.join(str(a) for a in angles_deg) + '\n'

        if self.dry_run:
            self.get_logger().info(f'[DRY RUN] TX -> {frame.strip()}')
            return

        if self.ser is None or not self.ser.is_open:
            self._connect_serial()
            if self.ser is None:
                return

        try:
            self.ser.write(frame.encode('ascii'))
            self.ser.flush()
            self.get_logger().info(f'TX -> {frame.strip()}')
        except Exception as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
