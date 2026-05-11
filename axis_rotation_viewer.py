"""
Axis Rotation Viewer - 用于理解不同轴的旋转

功能：
- 显示一个可旋转的几何体（箭头）
- 显示坐标系（X=红，Y=绿，Z=蓝）
- 滑条控制旋转
- 4x4矩阵输入
- 固定透明pose对比
- Home/Reset Camera
"""

import math
import sys

import numpy as np
import vtk
from PyQt5 import QtCore, QtWidgets
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

axis_x_color = "#ff4d5a"
axis_y_color = "#2ecc71"
axis_z_color = "#3b82ff"


def _hex_color(value: str) -> tuple[float, float, float]:
    text = value.strip().lstrip("#")
    if len(text) != 6:
        raise ValueError(f"expected #RRGGBB color, got {value!r}")
    return tuple(int(text[i : i + 2], 16) / 255.0 for i in (0, 2, 4))


def _vtk_matrix_from_numpy(matrix: np.ndarray) -> vtk.vtkMatrix4x4:
    vtk_matrix = vtk.vtkMatrix4x4()
    for row in range(4):
        for col in range(4):
            vtk_matrix.SetElement(row, col, float(matrix[row, col]))
    return vtk_matrix


def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    R = np.array([
        [cy * cz, -cy * sz, sy],
        [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
        [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy]
    ], dtype=float)
    return R


def _matrix_to_euler_deg(R: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(R[2, 0], sy)
        rz = math.atan2(-R[0, 2], R[0, 0])
    else:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(R[2, 0], sy)
        rz = 0

    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


class AxisRotationViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Axis Rotation Viewer")
        self.resize(1200, 800)

        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.fixed_pose = np.eye(4, dtype=float)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        self.main_layout = QtWidgets.QHBoxLayout(central)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        self.vtk_widget = QVTKRenderWindowInteractor(central)
        self.main_layout.addWidget(self.vtk_widget, 1)

        self.renderer = vtk.vtkRenderer()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.vtk_widget.GetRenderWindow().SetWindowName("Axis Rotation Viewer")
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

        self.renderer.SetBackground(0.96, 0.97, 0.98)
        self.renderer.SetBackground2(0.88, 0.91, 0.95)
        self.renderer.GradientBackgroundOn()
        self._setup_scene()
        self._setup_sidebar()

        self.vtk_widget.Initialize()
        self._update_scene()

    def _setup_scene(self):
        self.axis_length = 0.12

        self.x_axis = self._create_arrow(axis_x_color, self.axis_length)
        self.y_axis = self._create_arrow(axis_y_color, self.axis_length)
        self.z_axis = self._create_arrow(axis_z_color, self.axis_length)

        self.fixed_x_axis = self._create_fixed_axis(axis_x_color, self.axis_length)
        self.fixed_y_axis = self._create_fixed_axis(axis_y_color, self.axis_length)
        self.fixed_z_axis = self._create_fixed_axis(axis_z_color, self.axis_length)

        self.mesh_actor = self._create_mesh_actor()
        self.fixed_mesh_actor = self._create_fixed_mesh_actor()

        self._setup_camera()

    def _create_arrow(self, color: str, length: float) -> vtk.vtkActor:
        source = vtk.vtkArrowSource()
        source.SetShaftRadius(0.004)
        source.SetTipRadius(0.012)
        source.SetTipLength(0.25)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*_hex_color(color))
        actor.GetProperty().SetAmbient(0.3)
        actor.GetProperty().SetDiffuse(0.7)
        actor.GetProperty().SetSpecular(0.2)
        actor.GetProperty().SetSpecularPower(10)

        self.renderer.AddActor(actor)
        return actor

    def _create_fixed_axis(self, color: str, length: float) -> vtk.vtkActor:
        source = vtk.vtkArrowSource()
        source.SetShaftRadius(0.003)
        source.SetTipRadius(0.01)
        source.SetTipLength(0.25)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*_hex_color(color))
        actor.GetProperty().SetOpacity(0.2)

        self.renderer.AddActor(actor)
        return actor

    def _create_mesh_actor(self) -> vtk.vtkActor:
        source = vtk.vtkSphereSource()
        source.SetRadius(0.006)
        source.SetThetaResolution(48)
        source.SetPhiResolution(48)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.2, 0.4, 0.8)
        actor.GetProperty().SetAmbient(0.3)
        actor.GetProperty().SetDiffuse(0.7)
        actor.GetProperty().SetSpecular(0.5)

        self.renderer.AddActor(actor)
        return actor

    def _create_fixed_mesh_actor(self) -> vtk.vtkActor:
        source = vtk.vtkSphereSource()
        source.SetRadius(0.006)
        source.SetThetaResolution(48)
        source.SetPhiResolution(48)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.2, 0.4, 0.8)
        actor.GetProperty().SetOpacity(0.2)

        self.renderer.AddActor(actor)
        return actor

    def _setup_camera(self):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0.5, -0.5, 0.4)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()

    def _setup_sidebar(self):
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(360)
        panel.setStyleSheet("""
            QWidget#sidePanel { background: #ffffff; border-left: 1px solid #e2e8f0; }
            QGroupBox { font-weight: 700; border: 1px solid #e2e8f0; border-radius: 8px; margin-top: 15px; padding: 12px; background: #fcfdfe; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 8px; color: #334155; }
            QLabel { color: #64748b; font-size: 13px; font-family: "Segoe UI", sans-serif; }
            QDoubleSpinBox { min-width: 65px; border: 1px solid #cbd5e1; border-radius: 4px; padding: 3px; background: white; selection-background-color: #3b82f6; }
            QPushButton { background: #3b82f6; color: white; border: none; border-radius: 6px; padding: 8px 16px; font-weight: 600; font-size: 13px; }
            QPushButton:hover { background: #2563eb; }
            QPushButton#homeBtn { background: #64748b; }
            QPushButton#homeBtn:hover { background: #475569; }
            QPushButton#resetBtn { background: #94a3b8; }
            QPushButton#resetBtn:hover { background: #64748b; }
            QCheckBox { color: #475569; spacing: 10px; font-weight: 500; }
            QSlider::groove:horizontal { border: 1px solid #e2e8f0; height: 4px; background: #f1f5f9; border-radius: 2px; }
            QSlider::handle:horizontal { background: #3b82f6; border: 1px solid #2563eb; width: 16px; height: 16px; margin: -7px 0; border-radius: 8px; }
        """)

        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        header = QtWidgets.QLabel("Rotation Control")
        header.setStyleSheet("font-size: 18px; font-weight: 800; color: #0f172a; margin-bottom: 4px;")
        layout.addWidget(header)

        sub_header = QtWidgets.QLabel("X:Red | Y:Green | Z:Blue")
        sub_header.setStyleSheet("color: #94a3b8; font-weight: 500; margin-bottom: 10px;")
        layout.addWidget(sub_header)

        button_layout = QtWidgets.QHBoxLayout()
        self.home_btn = QtWidgets.QPushButton("Home Position")
        self.home_btn.setObjectName("homeBtn")
        self.reset_btn = QtWidgets.QPushButton("Reset View")
        self.reset_btn.setObjectName("resetBtn")
        self.home_btn.clicked.connect(self._on_home)
        self.reset_btn.clicked.connect(self._on_reset_camera)
        button_layout.addWidget(self.home_btn)
        button_layout.addWidget(self.reset_btn)
        layout.addLayout(button_layout)

        rotation_group = QtWidgets.QGroupBox("Rotation (degrees)")
        rot_layout = QtWidgets.QVBoxLayout(rotation_group)

        self.rot_x_slider, self.rot_x_spin = self._create_slider_spin("RX:", -180, 180, 0)
        self.rot_y_slider, self.rot_y_spin = self._create_slider_spin("RY:", -180, 180, 0)
        self.rot_z_slider, self.rot_z_spin = self._create_slider_spin("RZ:", -180, 180, 0)

        self.rot_x_slider.valueChanged.connect(lambda v: self._on_slider_changed("rx", v))
        self.rot_x_spin.valueChanged.connect(lambda v: self._on_spin_changed("rx", v))
        self.rot_y_slider.valueChanged.connect(lambda v: self._on_slider_changed("ry", v))
        self.rot_y_spin.valueChanged.connect(lambda v: self._on_spin_changed("ry", v))
        self.rot_z_slider.valueChanged.connect(lambda v: self._on_slider_changed("rz", v))
        self.rot_z_spin.valueChanged.connect(lambda v: self._on_spin_changed("rz", v))

        rot_layout.addLayout(self._make_row("RX:", self.rot_x_slider, self.rot_x_spin))
        rot_layout.addLayout(self._make_row("RY:", self.rot_y_slider, self.rot_y_spin))
        rot_layout.addLayout(self._make_row("RZ:", self.rot_z_slider, self.rot_z_spin))
        layout.addWidget(rotation_group)

        translation_group = QtWidgets.QGroupBox("Translation (meters)")
        trans_layout = QtWidgets.QVBoxLayout(translation_group)

        self.pos_x_slider, self.pos_x_spin = self._create_slider_spin("X:", -5, 5, 0, factor=1)
        self.pos_y_slider, self.pos_y_spin = self._create_slider_spin("Y:", -5, 5, 0, factor=1)
        self.pos_z_slider, self.pos_z_spin = self._create_slider_spin("Z:", -5, 5, 0, factor=1)

        self.pos_x_slider.valueChanged.connect(lambda v: self._on_slider_changed("x", v))
        self.pos_x_spin.valueChanged.connect(lambda v: self._on_spin_changed("x", v))
        self.pos_y_slider.valueChanged.connect(lambda v: self._on_slider_changed("y", v))
        self.pos_y_spin.valueChanged.connect(lambda v: self._on_spin_changed("y", v))
        self.pos_z_slider.valueChanged.connect(lambda v: self._on_slider_changed("z", v))
        self.pos_z_spin.valueChanged.connect(lambda v: self._on_spin_changed("z", v))

        trans_layout.addLayout(self._make_row("X:", self.pos_x_slider, self.pos_x_spin))
        trans_layout.addLayout(self._make_row("Y:", self.pos_y_slider, self.pos_y_spin))
        trans_layout.addLayout(self._make_row("Z:", self.pos_z_slider, self.pos_z_spin))
        layout.addWidget(translation_group)

        self.show_fixed_check = QtWidgets.QCheckBox("Show Fixed Pose (opacity 0.3)")
        self.show_fixed_check.setChecked(True)
        self.show_fixed_check.toggled.connect(self._update_scene)
        layout.addWidget(self.show_fixed_check)

        fixed_layout = QtWidgets.QHBoxLayout()
        fixed_layout.addWidget(self.show_fixed_check)

        set_home_btn = QtWidgets.QPushButton("Set as Fixed Pose")
        set_home_btn.setStyleSheet("background: #0f172a; margin-left: 10px;")
        set_home_btn.clicked.connect(self._set_as_home)
        fixed_layout.addWidget(set_home_btn)
        layout.addLayout(fixed_layout)

        matrix_group = QtWidgets.QGroupBox("Homogeneous Matrix (4x4)")
        matrix_layout = QtWidgets.QGridLayout(matrix_group)
        matrix_layout.setSpacing(4)

        self.matrix_inputs = []
        for row in range(4):
            row_inputs = []
            for col in range(4):
                spin = QtWidgets.QDoubleSpinBox()
                spin.setDecimals(3)
                spin.setRange(-999.0, 999.0)
                spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
                spin.setAlignment(QtCore.Qt.AlignCenter)
                spin.setStyleSheet("border: 1px solid #e2e8f0; border-radius: 3px; font-family: 'Consolas'; font-size: 11px;")
                if row == col:
                    spin.setValue(1.0)
                else:
                    spin.setValue(0.0)
                spin.valueChanged.connect(self._on_matrix_changed)
                row_inputs.append(spin)
                matrix_layout.addWidget(spin, row, col)
            self.matrix_inputs.append(row_inputs)

        btn_container = QtWidgets.QHBoxLayout()
        self.apply_matrix_btn = QtWidgets.QPushButton("Apply Matrix")
        self.apply_matrix_btn.setStyleSheet("background: #10b981;") # Green for action
        self.apply_matrix_btn.clicked.connect(self._apply_matrix_to_rotation)
        
        self.set_from_rot_btn = QtWidgets.QPushButton("Fetch from Scene")
        self.set_from_rot_btn.setStyleSheet("background: #f59e0b;") # Orange
        self.set_from_rot_btn.clicked.connect(self._update_matrix_from_rotation)
        
        btn_container.addWidget(self.apply_matrix_btn)
        btn_container.addWidget(self.set_from_rot_btn)
        matrix_layout.addLayout(btn_container, 4, 0, 1, 4)

        layout.addWidget(matrix_group)

        pose_info_group = QtWidgets.QGroupBox("Current State")
        pose_info_layout = QtWidgets.QVBoxLayout(pose_info_group)
        self.pose_label = QtWidgets.QLabel()
        self.pose_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.pose_label.setStyleSheet("""
            font-family: 'Consolas', 'Courier New', monospace; 
            font-size: 11px; 
            color: #1e293b; 
            background: #f1f5f9; 
            padding: 10px; 
            border-radius: 6px;
            line-height: 1.4;
        """)
        pose_info_layout.addWidget(self.pose_label)
        layout.addWidget(pose_info_group)

        layout.addStretch()

        self.main_layout.addWidget(panel)

    def _create_slider_spin(self, label: str, min_val: int, max_val: int, default: int, factor: float = 1.0):
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(int(min_val * factor), int(max_val * factor))
        slider.setValue(int(default * factor))

        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(float(min_val), float(max_val))
        spin.setDecimals(2 if factor > 1 else 1)
        spin.setSingleStep(1.0 / factor)
        spin.setValue(float(default))

        return slider, spin

    def _make_row(self, label: str, slider, spin):
        row = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(label)
        lbl.setFixedWidth(30)
        row.addWidget(lbl)
        row.addWidget(slider, 1)
        row.addWidget(spin)
        return row

    def _on_slider_changed(self, axis: str, value: int):
        if axis == "rx":
            self.rot_x = float(value)
            self.rot_x_spin.blockSignals(True)
            self.rot_x_spin.setValue(self.rot_x)
            self.rot_x_spin.blockSignals(False)
        elif axis == "ry":
            self.rot_y = float(value)
            self.rot_y_spin.blockSignals(True)
            self.rot_y_spin.setValue(self.rot_y)
            self.rot_y_spin.blockSignals(False)
        elif axis == "rz":
            self.rot_z = float(value)
            self.rot_z_spin.blockSignals(True)
            self.rot_z_spin.setValue(self.rot_z)
            self.rot_z_spin.blockSignals(False)
        elif axis == "x":
            self.pos_x = float(value) / 100.0
            self.pos_x_spin.blockSignals(True)
            self.pos_x_spin.setValue(self.pos_x)
            self.pos_x_spin.blockSignals(False)
        elif axis == "y":
            self.pos_y = float(value) / 100.0
            self.pos_y_spin.blockSignals(True)
            self.pos_y_spin.setValue(self.pos_y)
            self.pos_y_spin.blockSignals(False)
        elif axis == "z":
            self.pos_z = float(value) / 100.0
            self.pos_z_spin.blockSignals(True)
            self.pos_z_spin.setValue(self.pos_z)
            self.pos_z_spin.blockSignals(False)
        self._update_scene()

    def _on_spin_changed(self, axis: str, value: float):
        if axis == "rx":
            self.rot_x = value
            self.rot_x_slider.blockSignals(True)
            self.rot_x_slider.setValue(int(round(value)))
            self.rot_x_slider.blockSignals(False)
        elif axis == "ry":
            self.rot_y = value
            self.rot_y_slider.blockSignals(True)
            self.rot_y_slider.setValue(int(round(value)))
            self.rot_y_slider.blockSignals(False)
        elif axis == "rz":
            self.rot_z = value
            self.rot_z_slider.blockSignals(True)
            self.rot_z_slider.setValue(int(round(value)))
            self.rot_z_slider.blockSignals(False)
        elif axis == "x":
            self.pos_x = value
            self.pos_x_slider.blockSignals(True)
            self.pos_x_slider.setValue(int(round(value * 100.0)))
            self.pos_x_slider.blockSignals(False)
        elif axis == "y":
            self.pos_y = value
            self.pos_y_slider.blockSignals(True)
            self.pos_y_slider.setValue(int(round(value * 100.0)))
            self.pos_y_slider.blockSignals(False)
        elif axis == "z":
            self.pos_z = value
            self.pos_z_slider.blockSignals(True)
            self.pos_z_slider.setValue(int(round(value * 100.0)))
            self.pos_z_slider.blockSignals(False)
        self._update_scene()

    def _on_home(self):
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        for slider in [self.rot_x_slider, self.rot_y_slider, self.rot_z_slider, 
                      self.pos_x_slider, self.pos_y_slider, self.pos_z_slider]:
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            
        for spin in [self.rot_x_spin, self.rot_y_spin, self.rot_z_spin, 
                    self.pos_x_spin, self.pos_y_spin, self.pos_z_spin]:
            spin.blockSignals(True)
            spin.setValue(0.0)
            spin.blockSignals(False)

        self._update_scene()

    def _on_reset_camera(self):
        self._setup_camera()
        self.vtk_widget.GetRenderWindow().Render()

    def _set_as_home(self):
        self.fixed_pose = self._get_current_pose()
        self._update_scene()

    def _get_current_pose(self) -> np.ndarray:
        R = _euler_to_rotation_matrix(self.rot_x, self.rot_y, self.rot_z)
        pose = np.eye(4, dtype=float)
        pose[:3, :3] = R
        pose[0, 3] = self.pos_x
        pose[1, 3] = self.pos_y
        pose[2, 3] = self.pos_z
        return pose

    def _on_matrix_changed(self):
        pass

    def _apply_matrix_to_rotation(self):
        matrix = np.eye(4, dtype=float)
        for row in range(4):
            for col in range(4):
                matrix[row, col] = self.matrix_inputs[row][col].value()

        if abs(np.linalg.det(matrix[:3, :3]) - 1.0) > 0.01:
            QtWidgets.QMessageBox.warning(self, "Warning", "Matrix rotation part is not a valid rotation matrix (determinant != 1)")

        rx, ry, rz = _matrix_to_euler_deg(matrix[:3, :3])

        self.rot_x = rx
        self.rot_y = ry
        self.rot_z = rz
        self.pos_x = matrix[0, 3]
        self.pos_y = matrix[1, 3]
        self.pos_z = matrix[2, 3]

        # Update RX, RY, RZ
        self.rot_x_slider.blockSignals(True)
        self.rot_x_spin.blockSignals(True)
        self.rot_x_slider.setValue(int(round(rx)))
        self.rot_x_spin.setValue(rx)
        self.rot_x_slider.blockSignals(False)
        self.rot_x_spin.blockSignals(False)

        self.rot_y_slider.blockSignals(True)
        self.rot_y_spin.blockSignals(True)
        self.rot_y_slider.setValue(int(round(ry)))
        self.rot_y_spin.setValue(ry)
        self.rot_y_slider.blockSignals(False)
        self.rot_y_spin.blockSignals(False)

        self.rot_z_slider.blockSignals(True)
        self.rot_z_spin.blockSignals(True)
        self.rot_z_slider.setValue(int(round(rz)))
        self.rot_z_spin.setValue(rz)
        self.rot_z_slider.blockSignals(False)
        self.rot_z_spin.blockSignals(False)

        # Update X, Y, Z
        self.pos_x_slider.blockSignals(True)
        self.pos_x_spin.blockSignals(True)
        self.pos_x_slider.setValue(int(round(self.pos_x * 100.0)))
        self.pos_x_spin.setValue(self.pos_x)
        self.pos_x_slider.blockSignals(False)
        self.pos_x_spin.blockSignals(False)

        self.pos_y_slider.blockSignals(True)
        self.pos_y_spin.blockSignals(True)
        self.pos_y_slider.setValue(int(round(self.pos_y * 100.0)))
        self.pos_y_spin.setValue(self.pos_y)
        self.pos_y_slider.blockSignals(False)
        self.pos_y_spin.blockSignals(False)

        self.pos_z_slider.blockSignals(True)
        self.pos_z_spin.blockSignals(True)
        self.pos_z_slider.setValue(int(round(self.pos_z * 100.0)))
        self.pos_z_spin.setValue(self.pos_z)
        self.pos_z_slider.blockSignals(False)
        self.pos_z_spin.blockSignals(False)

        self._update_scene()

    def _update_matrix_from_rotation(self):
        pose = self._get_current_pose()
        for row in range(4):
            for col in range(4):
                self.matrix_inputs[row][col].blockSignals(True)
                self.matrix_inputs[row][col].setValue(float(pose[row, col]))
                self.matrix_inputs[row][col].blockSignals(False)

    def _update_scene(self):
        R = _euler_to_rotation_matrix(self.rot_x, self.rot_y, self.rot_z)
        pos = np.array([self.pos_x, self.pos_y, self.pos_z])
        R_fixed = self.fixed_pose[:3, :3]
        pos_fixed = self.fixed_pose[:3, 3]

        self._set_arrow_transform(self.x_axis, np.array([self.axis_length, 0, 0]), R, pos)
        self._set_arrow_transform(self.y_axis, np.array([0, self.axis_length, 0]), R, pos)
        self._set_arrow_transform(self.z_axis, np.array([0, 0, self.axis_length]), R, pos)

        self._set_arrow_transform(self.fixed_x_axis, np.array([self.axis_length, 0, 0]), R_fixed, pos_fixed)
        self._set_arrow_transform(self.fixed_y_axis, np.array([0, self.axis_length, 0]), R_fixed, pos_fixed)
        self._set_arrow_transform(self.fixed_z_axis, np.array([0, 0, self.axis_length]), R_fixed, pos_fixed)

        pose_4x4 = np.eye(4, dtype=float)
        pose_4x4[:3, :3] = R
        pose_4x4[:3, 3] = pos

        transform = vtk.vtkTransform()
        transform.SetMatrix(_vtk_matrix_from_numpy(pose_4x4))
        self.mesh_actor.SetUserMatrix(transform.GetMatrix())

        fixed_transform = vtk.vtkTransform()
        fixed_transform.SetMatrix(_vtk_matrix_from_numpy(self.fixed_pose))
        self.fixed_mesh_actor.SetUserMatrix(fixed_transform.GetMatrix())

        show_fixed = self.show_fixed_check.isChecked()
        self.fixed_mesh_actor.SetVisibility(show_fixed)
        self.fixed_x_axis.SetVisibility(show_fixed)
        self.fixed_y_axis.SetVisibility(show_fixed)
        self.fixed_z_axis.SetVisibility(show_fixed)

        self._update_pose_label()

        self.vtk_widget.GetRenderWindow().Render()

    def _set_arrow_transform(self, actor: vtk.vtkActor, local_dir: np.ndarray, R: np.ndarray, pos: np.ndarray):
        world_dir = R @ local_dir
        length = np.linalg.norm(world_dir)
        if length < 1e-6:
            actor.SetVisibility(False)
            return
        actor.SetVisibility(True)
        
        direction = world_dir / length
        up = np.array([0, 0, 1])
        if abs(np.dot(direction, up)) > 0.999:
            up = np.array([1, 0, 0])
        
        right = np.cross(up, direction)
        right /= np.linalg.norm(right)
        up = np.cross(direction, right)
        
        rot_mat = np.eye(4)
        rot_mat[:3, 0] = direction
        rot_mat[:3, 1] = right
        rot_mat[:3, 2] = up
        rot_mat[:3, 3] = pos
        
        actor.SetUserMatrix(_vtk_matrix_from_numpy(rot_mat))
        actor.SetScale(length, 1, 1)

    def _update_pose_label(self):
        pose = self._get_current_pose()
        fmt = "{:.4f}"
        text = "Current Pose (Euler XYZ):\n"
        text += f"X: {fmt.format(self.rot_x)} deg\n"
        text += f"Y: {fmt.format(self.rot_y)} deg\n"
        text += f"Z: {fmt.format(self.rot_z)} deg\n\n"
        text += "Matrix:\n"
        for row in range(4):
            text += "[" + " ".join(fmt.format(pose[row, col]) for col in range(4)) + "]\n"
        self.pose_label.setText(text)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = AxisRotationViewer()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()