import sys
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from PyQt5 import QtCore, QtWidgets

import tf2_ros
from pymoveit2 import MoveIt2


@dataclass
class TargetPoint:
    x: float
    y: float
    z: float


class RosSpinThread(QtCore.QThread):
    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        self._running = True

    def run(self):
        while rclpy.ok() and self._running:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def stop(self):
        self._running = False


class MotionWorker(QtCore.QThread):
    log_sig = QtCore.pyqtSignal(str)
    state_sig = QtCore.pyqtSignal(str)
    done_sig = QtCore.pyqtSignal(bool, str)

    def __init__(
        self,
        node: Node,
        moveit2: MoveIt2,
        tf_buffer: tf2_ros.Buffer,
        base_link: str,
        ee_link: str,
        targets: List[TargetPoint],
        relative: bool,
        vel_scale: float,
        acc_scale: float,
        parent=None,
    ):
        super().__init__(parent)
        self.node = node
        self.moveit2 = moveit2
        self.tf_buffer = tf_buffer
        self.base_link = base_link
        self.ee_link = ee_link
        self.targets = targets
        self.relative = relative
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self._stop = False

    def request_stop(self):
        self._stop = True

    def _get_current_ee_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.base_link,
            self.ee_link,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        z = float(tf.transform.translation.z)
        q = tf.transform.rotation
        quat_xyzw = (float(q.x), float(q.y), float(q.z), float(q.w))
        return (x, y, z), quat_xyzw

    def run(self):
        try:
            if not self.targets:
                self.done_sig.emit(False, "No target points.")
                return

            self.state_sig.emit("RUNNING")
            self.log_sig.emit(
                f"targets={len(self.targets)} relative={self.relative} "
                f"vel={self.vel_scale:.2f} acc={self.acc_scale:.2f}"
            )

            self.moveit2.max_velocity = self.vel_scale
            self.moveit2.max_acceleration = self.acc_scale

            try:
                (base_x, base_y, base_z), quat_xyzw = self._get_current_ee_pose()
                self.log_sig.emit(
                    f"current_pose: x={base_x:.3f} y={base_y:.3f} z={base_z:.3f} "
                    f"q=({quat_xyzw[0]:.3f},{quat_xyzw[1]:.3f},{quat_xyzw[2]:.3f},{quat_xyzw[3]:.3f})"
                )
            except Exception as e:
                base_x, base_y, base_z = 0.0, 0.0, 0.0
                quat_xyzw = (0.0, 0.0, 0.0, 1.0)
                self.log_sig.emit(f"warn: cannot read EE TF, fallback used. ({e})")

            for i, t in enumerate(self.targets, start=1):
                if self._stop:
                    self.done_sig.emit(False, "Stopped by user.")
                    return

                if self.relative:
                    base_x += t.x
                    base_y += t.y
                    base_z += t.z
                    goal = (base_x, base_y, base_z)goal =
                else:
                    goal = (t.x, t.y, t.z)

                self.log_sig.emit(
                    f"[{i}/{len(self.targets)}] move_to_pose: "
                    f"x={goal[0]:.3f} y={goal[1]:.3f} z={goal[2]:.3f}"
                )

                self.moveit2.move_to_pose(position=goal) # , quat_xyzw=quat_xyzw)
                ok = self.moveit2.wait_until_executed()

                if not ok:
                    self.done_sig.emit(False, f"Execution failed at target #{i}.")
                    return

                self.log_sig.emit(f"done: target #{i}")

            self.done_sig.emit(True, "All targets completed.")
        except Exception as e:
            self.done_sig.emit(False, f"Exception: {e}")
        finally:
            self.state_sig.emit("IDLE")


class AssignmentGui(QtWidgets.QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

        self.base_link = "base_link"
        self.ee_link = "link_6"  # change if your EE frame differs

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name=self.base_link,
            end_effector_name=self.ee_link,
            group_name="manipulator",
        )

        self.worker = None
        self._build_ui()

    def _build_ui(self):
        self.setWindowTitle("ROS2 Assignment - E0509 GUI")
        self.resize(1000, 600)

        self.chk_relative = QtWidgets.QCheckBox("Relative mode (unchecked = absolute)")
        self.ed_vel = QtWidgets.QLineEdit("0.2")
        self.ed_acc = QtWidgets.QLineEdit("0.2")

        form = QtWidgets.QFormLayout()
        form.addRow(self.chk_relative)
        form.addRow("Velocity scale (0~1)", self.ed_vel)
        form.addRow("Acceleration scale (0~1)", self.ed_acc)

        self.tbl = QtWidgets.QTableWidget(1, 3)
        self.tbl.setHorizontalHeaderLabels(["x", "y", "z"])
        self.tbl.horizontalHeader().setStretchLastSection(True)

        btn_add = QtWidgets.QPushButton("Add target")
        btn_del = QtWidgets.QPushButton("Remove selected")
        btn_run = QtWidgets.QPushButton("Run")
        btn_stop = QtWidgets.QPushButton("Stop")

        btn_add.clicked.connect(self._add_row)
        btn_del.clicked.connect(self._del_row)
        btn_run.clicked.connect(self._run)
        btn_stop.clicked.connect(self._stop)

        row1 = QtWidgets.QHBoxLayout()
        row1.addWidget(btn_add)
        row1.addWidget(btn_del)

        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(btn_run)
        row2.addWidget(btn_stop)

        left = QtWidgets.QVBoxLayout()
        left.addLayout(form)
        left.addWidget(QtWidgets.QLabel("Targets (x,y,z)"))
        left.addWidget(self.tbl)
        left.addLayout(row1)
        left.addLayout(row2)

        self.lb_conn = QtWidgets.QLabel("Connection: (unknown)")
        self.lb_state = QtWidgets.QLabel("State: IDLE")
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self.lb_conn)
        right.addWidget(self.lb_state)
        right.addWidget(QtWidgets.QLabel("Log"))
        right.addWidget(self.log)

        root = QtWidgets.QHBoxLayout()
        root.addLayout(left, 2)
        root.addLayout(right, 1)
        self.setLayout(root)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick_status)
        self.timer.start(1000)

    def _tick_status(self):
        self.lb_conn.setText("Connection: ROS OK" if rclpy.ok() else "Connection: ROS DOWN")

    def _add_row(self):
        self.tbl.insertRow(self.tbl.rowCount())

    def _del_row(self):
        rows = sorted({i.row() for i in self.tbl.selectedIndexes()}, reverse=True)
        for r in rows:
            self.tbl.removeRow(r)

    def _parse_targets(self) -> List[TargetPoint]:
        targets: List[TargetPoint] = []
        for r in range(self.tbl.rowCount()):
            def read_float(c: int) -> float:
                item = self.tbl.item(r, c)
                if item is None:
                    return 0.0
                s = item.text().strip()
                return float(s) if s else 0.0

            targets.append(TargetPoint(read_float(0), read_float(1), read_float(2)))
        return targets

    def _run(self):
        if self.worker and self.worker.isRunning():
            self._append("Already running.")
            return

        try:
            vel = float(self.ed_vel.text().strip())
            acc = float(self.ed_acc.text().strip())
        except Exception:
            self._append("Invalid velocity/acceleration.")
            return

        vel = max(0.0, min(1.0, vel))
        acc = max(0.0, min(1.0, acc))

        targets = self._parse_targets()

        self.worker = MotionWorker(
            node=self.node,
            moveit2=self.moveit2,
            tf_buffer=self.tf_buffer,
            base_link=self.base_link,
            ee_link=self.ee_link,
            targets=targets,
            relative=self.chk_relative.isChecked(),
            vel_scale=vel,
            acc_scale=acc,
        )
        self.worker.log_sig.connect(self._append)
        self.worker.state_sig.connect(self._set_state)
        self.worker.done_sig.connect(self._done)
        self.worker.start()

    def _stop(self):
        if self.worker and self.worker.isRunning():
            self.worker.request_stop()
            self._append("Stop requested.")

    def _set_state(self, s: str):
        self.lb_state.setText(f"State: {s}")

    def _done(self, ok: bool, msg: str):
        self._append(("SUCCESS: " if ok else "FAIL: ") + msg)

    def _append(self, msg: str):
        self.log.append(msg)


def main():
    rclpy.init()
    node = Node("my_ros2_assignment_gui")

    app = QtWidgets.QApplication(sys.argv)
    gui = AssignmentGui(node)
    gui.show()

    spin_thread = RosSpinThread(node)
    spin_thread.start()

    ret = app.exec_()

    spin_thread.stop()
    spin_thread.wait()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

