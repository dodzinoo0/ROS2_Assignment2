import math
import time
import tkinter as tk
from tkinter import ttk
from typing import Dict, List

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
JOINT_LABELS = {
    'joint1': 'Joint 1 (Base Yaw — Servo 1)',
    'joint2': 'Joint 2 (Shoulder Pitch — Servo 2)',
    'joint3': 'Joint 3 (Elbow Pitch)',
    'joint4': 'Joint 4 (Wrist Pitch)',
}
JOINT_MIN = -1.5708
JOINT_MAX = 1.5708


def rad_to_servo_deg(rad: float) -> int:
    deg = ((rad - JOINT_MIN) / (JOINT_MAX - JOINT_MIN)) * 180.0
    return int(round(max(0.0, min(180.0, deg))))


class ArmGuiNode(Node):
    """Tkinter slider GUI that publishes JointTrajectory commands to
    /arm_controller/joint_trajectory so Gazebo, RViz2, and Arduino servos move together."""

    def __init__(self) -> None:
        super().__init__('arm_gui')
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.last_live_publish = 0.0

        self.root = tk.Tk()
        self.root.title('ROS 2 Robotic Arm Controller (/arm_controller/joint_trajectory)')
        self.root.geometry('540x430')
        self.root.resizable(False, False)

        self.vars: Dict[str, tk.DoubleVar] = {}
        self.readout_labels: Dict[str, ttk.Label] = {}
        self.live_var = tk.BooleanVar(value=True)
        self.duration_var = tk.DoubleVar(value=0.4)

        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.running = True
        self.get_logger().info('arm_gui ready.')

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=16)
        container.pack(fill=tk.BOTH, expand=True)

        header = ttk.Label(
            container,
            text='Robotic Arm Joint Trajectory Control',
            font=('TkDefaultFont', 13, 'bold'),
        )
        header.pack(anchor=tk.W, pady=(0, 10))

        for name in JOINT_NAMES:
            row = ttk.Frame(container)
            row.pack(fill=tk.X, pady=6)

            top = ttk.Frame(row)
            top.pack(fill=tk.X)
            ttk.Label(top, text=JOINT_LABELS[name]).pack(side=tk.LEFT)
            readout = ttk.Label(top, text='0.00 rad  (90° servo)', width=22, anchor=tk.E)
            readout.pack(side=tk.RIGHT)
            self.readout_labels[name] = readout

            var = tk.DoubleVar(value=0.0)
            self.vars[name] = var
            scale = ttk.Scale(
                row,
                from_=JOINT_MIN,
                to=JOINT_MAX,
                orient=tk.HORIZONTAL,
                variable=var,
                command=lambda _val, j=name: self._on_slider_change(j),
            )
            scale.pack(fill=tk.X)

        controls = ttk.Frame(container)
        controls.pack(fill=tk.X, pady=(14, 8))

        ttk.Checkbutton(
            controls,
            text='Live publish while dragging',
            variable=self.live_var,
        ).pack(side=tk.LEFT)

        ttk.Label(controls, text='Duration (s):').pack(side=tk.LEFT, padx=(20, 4))
        ttk.Spinbox(
            controls,
            from_=0.1,
            to=5.0,
            increment=0.1,
            width=5,
            textvariable=self.duration_var,
        ).pack(side=tk.LEFT)

        ttk.Button(
            controls,
            text='Send Trajectory',
            command=lambda: self.publish_current(force=True),
        ).pack(side=tk.RIGHT)

        presets = ttk.LabelFrame(container, text='Preset Poses', padding=8)
        presets.pack(fill=tk.X, pady=(8, 6))

        preset_defs = [
            ('Home (0°)', [0.0, 0.0, 0.0, 0.0]),
            ('Reach Forward', [0.0, 0.6, -0.6, 0.3]),
            ('Swing Right', [1.0, 0.4, -0.4, 0.2]),
            ('Swing Left', [-1.0, 0.4, -0.4, -0.2]),
        ]
        for label, angles in preset_defs:
            ttk.Button(
                presets,
                text=label,
                command=lambda a=angles: self._apply_preset(a),
            ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)

        self.status_label = ttk.Label(
            container,
            text='Topic: /arm_controller/joint_trajectory (ready)',
            foreground='#444444',
        )
        self.status_label.pack(anchor=tk.W, pady=(8, 0))

    def _update_readouts(self) -> None:
        for name in JOINT_NAMES:
            rad = self.vars[name].get()
            deg = rad_to_servo_deg(rad)
            self.readout_labels[name].config(text=f'{rad:+.2f} rad  ({deg:3d}° servo)')

    def _on_slider_change(self, _joint: str) -> None:
        self._update_readouts()
        if self.live_var.get():
            now = time.monotonic()
            if now - self.last_live_publish >= 0.06:  # ~16 Hz cap
                self.last_live_publish = now
                self.publish_current(duration_override=0.15)

    def _apply_preset(self, angles: List[float]) -> None:
        for name, val in zip(JOINT_NAMES, angles):
            self.vars[name].set(val)
        self._update_readouts()
        self.publish_current(duration_override=max(1.0, self.duration_var.get()), force=True)

    def publish_current(self, duration_override: float = None, force: bool = False) -> None:
        duration_sec = (
            duration_override if duration_override is not None else float(self.duration_var.get())
        )
        positions = [round(self.vars[name].get(), 4) for name in JOINT_NAMES]

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = positions
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points = [pt]

        self.pub.publish(msg)
        self.status_label.config(
            text=f'Published: {[round(p, 2) for p in positions]} rad (t={duration_sec:.2f}s)'
        )
        if force:
            self.get_logger().info(f'GUI published trajectory: {positions} (t={duration_sec:.2f}s)')

    def _spin_step(self) -> None:
        if not self.running:
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(20, self._spin_step)

    def _on_close(self) -> None:
        self.running = False
        self.root.destroy()

    def run(self) -> None:
        self.root.after(20, self._spin_step)
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGuiNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
