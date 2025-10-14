'''
SmartHand - Robot Delta Phone Interaction System

Features:
- Camera view for phone monitoring
- Chessboard detection and perspective transformation
- Coordinate mapping from phone screen to robot workspace
- Robot control and positioning
- Touch simulation with stylus
- Safety zone configuration
- Auto-detection of phone plane
'''

import sys
import time
import threading
import cv2
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QLineEdit, 
                            QGroupBox, QGridLayout, QSpinBox, QMessageBox,
                            QFileDialog, QComboBox, QCheckBox, QSlider,
                            QTextEdit, QTabWidget, QDoubleSpinBox, QSplitter)
from PyQt5.QtCore import QTimer, Qt, QPoint, pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont


class ClickableLabel(QLabel):
    """Custom QLabel that emits click signals"""
    clicked = pyqtSignal(QPoint)
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(event.pos())


class RobotController:
    """Simple serial controller for the Delta robot."""

    def __init__(self, handshake_command="IsDelta", handshake_response="YesDelta",
                 default_feedrate=2000, home_z=-291.28):
        self.handshake_command = handshake_command
        self.handshake_response = handshake_response
        self.default_feedrate = default_feedrate
        self.home_z = float(home_z)
        self.conn = None
        self.lock = threading.Lock()
        self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)

    def list_ports(self):
        """Return available serial ports."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def is_connected(self):
        return self.conn is not None and self.conn.is_open

    def connect(self, port, baudrate):
        """Connect to robot and perform handshake."""
        if self.is_connected():
            self.disconnect()

        try:
            conn = serial.Serial(port, baudrate, timeout=2)
            # Allow device time to boot after opening port
            time.sleep(0.5)

            if self.handshake_command:
                conn.write((self.handshake_command + "\n").encode())
                conn.flush()
                response = conn.readline().decode(errors="ignore").strip()
                if response != self.handshake_response:
                    conn.close()
                    if response:
                        return False, f"Unexpected handshake response: {response}"
                    return False, "No handshake response from device"

            self.conn = conn
            self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)
            self.set_absolute_mode()
            return True, ""
        except Exception as exc:
            return False, str(exc)

    def disconnect(self):
        if self.conn:
            try:
                if self.conn.is_open:
                    self.conn.close()
            finally:
                self.conn = None
                self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)

    def set_absolute_mode(self):
        if self.is_connected():
            self.send_command("G90", wait_for_ok=False)

    def set_relative_mode(self):
        if self.is_connected():
            self.send_command("G91", wait_for_ok=False)

    def send_command(self, command, wait_for_ok=True, timeout=5.0):
        """Send raw G-code to robot and return responses."""
        if not self.is_connected():
            raise RuntimeError("Robot not connected")

        command = command.strip()
        responses = []

        with self.lock:
            self._drain_input()
            self.conn.write((command + "\n").encode())
            self.conn.flush()

            if wait_for_ok:
                end_time = time.time() + timeout
                while time.time() < end_time:
                    line = self.conn.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    responses.append(line)
                    lower = line.lower()
                    if lower.startswith("ok") or lower.startswith("error"):
                        break
            return responses

    def _drain_input(self):
        """Clear any buffered data to keep responses in sync."""
        if not self.is_connected():
            return

        try:
            waiting = self.conn.in_waiting
        except AttributeError:
            waiting = 0

        if waiting:
            try:
                self.conn.read(waiting)
            except Exception:
                pass

    def home(self):
        responses = self.send_command("G28")
        self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)
        return responses

    def move_linear_absolute(self, x=None, y=None, z=None, feedrate=None):
        if not self.is_connected():
            raise RuntimeError("Robot not connected")

        if feedrate is None:
            feedrate = self.default_feedrate

        parts = ["G01"]
        new_position = self.current_position.copy()

        if x is not None:
            parts.append(f"X{x:.3f}")
            new_position[0] = x
        if y is not None:
            parts.append(f"Y{y:.3f}")
            new_position[1] = y
        if z is not None:
            parts.append(f"Z{z:.3f}")
            new_position[2] = z

        parts.append(f"F{float(feedrate):.0f}")
        responses = self.send_command(" ".join(parts))
        self.current_position = new_position
        return responses

    def move_linear_relative(self, dx=0.0, dy=0.0, dz=0.0, feedrate=None):
        if not self.is_connected():
            raise RuntimeError("Robot not connected")

        if feedrate is None:
            feedrate = self.default_feedrate

        delta = np.array([dx, dy, dz], dtype=float)
        if np.allclose(delta, 0):
            return []

        self.set_relative_mode()
        try:
            parts = ["G01"]
            if dx:
                parts.append(f"X{dx:.3f}")
            if dy:
                parts.append(f"Y{dy:.3f}")
            if dz:
                parts.append(f"Z{dz:.3f}")
            parts.append(f"F{float(feedrate):.0f}")
            responses = self.send_command(" ".join(parts))
            self.current_position += delta
        finally:
            self.set_absolute_mode()

        return responses

    def dwell(self, seconds):
        milliseconds = max(int(seconds * 1000), 1)
        return self.send_command(f"G4 P{milliseconds}")

    def get_position(self):
        return self.current_position.copy()

    def set_position(self, x=None, y=None, z=None):
        if x is not None:
            self.current_position[0] = x
        if y is not None:
            self.current_position[1] = y
        if z is not None:
            self.current_position[2] = z

class SmartHandApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SmartHand - Robot Delta Phone Interaction System")
        self.setGeometry(50, 50, 1600, 900)
        
        # Camera properties
        self.camera = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.current_frame = None
        self.transformed_frame = None
        
        # Chessboard detection
        self.chessboard_size = (7, 7)
        self.chessboard_found = False
        self.corners = None
        self.transformation_matrix = None
        self.transformed_size = (800, 600)
        
        # Phone detection
        self.phone_corners = []  # 4 corners of phone screen
        self.phone_plane_equation = None  # ax + by + cz + d = 0
        self.phone_z_height = 0.0  # Z height of phone surface
        
        # Coordinate mapping
        self.calibration_points = []
        self.mapping_matrix = None
        self.click_mode = None
        self.temp_click_point = None
        
        # Robot connection
        self.robot_controller = RobotController()
        self.robot_home_z = self.robot_controller.home_z
        self.robot_connected = False
        self.robot_position = [0.0, 0.0, self.robot_home_z]  # Current XYZ position
        self.robot_safe_z = self.robot_home_z + 40.0  # Safe Z height (above phone)
        
        # Touch settings
        self.touch_force = 5.0  # Force when touching (mm down from surface)
        self.touch_duration = 0.1  # Touch duration in seconds
        
        self.init_ui()
        
    def init_ui(self):
        # Main widget with splitter
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        
        # Left panel - Camera views
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        left_widget.setLayout(left_layout)
        
        # Camera view
        camera_container = QGroupBox("Live Camera View")
        camera_layout = QVBoxLayout()
        self.camera_label = ClickableLabel("Camera Feed")
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid #2196F3; background-color: #222;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.clicked.connect(self.on_camera_click)
        camera_layout.addWidget(self.camera_label)
        camera_container.setLayout(camera_layout)
        left_layout.addWidget(camera_container)
        
        # Transformed view
        transformed_container = QGroupBox("Top-Down View (Phone Screen)")
        transformed_layout = QVBoxLayout()
        self.transformed_label = ClickableLabel("Transformed View")
        self.transformed_label.setFixedSize(640, 480)
        self.transformed_label.setStyleSheet("border: 2px solid #4CAF50; background-color: #222;")
        self.transformed_label.setAlignment(Qt.AlignCenter)
        self.transformed_label.clicked.connect(self.on_transformed_click)
        transformed_layout.addWidget(self.transformed_label)
        transformed_container.setLayout(transformed_layout)
        left_layout.addWidget(transformed_container)
        
        splitter.addWidget(left_widget)
        
        # Right panel - Control tabs
        right_widget = QWidget()
        right_layout = QVBoxLayout()
        right_widget.setLayout(right_layout)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        
        # Tab 1: Camera & Detection
        tab_camera = QWidget()
        self.create_camera_tab(tab_camera)
        self.tab_widget.addTab(tab_camera, "📷 Camera")
        
        # Tab 2: Calibration
        tab_calibration = QWidget()
        self.create_calibration_tab(tab_calibration)
        self.tab_widget.addTab(tab_calibration, "🎯 Calibration")
        
        # Tab 3: Robot Control
        tab_robot = QWidget()
        self.create_robot_tab(tab_robot)
        self.tab_widget.addTab(tab_robot, "🤖 Robot")
        
        # Tab 4: Touch Control
        tab_touch = QWidget()
        self.create_touch_tab(tab_touch)
        self.tab_widget.addTab(tab_touch, "👆 Touch")
        
        right_layout.addWidget(self.tab_widget)
        
        # Status bar at bottom
        status_container = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(120)
        self.status_text.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;")
        status_layout.addWidget(self.status_text)
        status_container.setLayout(status_layout)
        right_layout.addWidget(status_container)
        
        splitter.addWidget(right_widget)
        
        # Set splitter sizes
        splitter.setSizes([800, 800])
        main_layout.addWidget(splitter)
        
        self.log_status("System initialized. Ready to start.")
        
    def create_camera_tab(self, parent):
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        # Camera controls
        camera_group = QGroupBox("Camera Setup")
        camera_layout = QVBoxLayout()
        
        # Camera selection
        cam_layout = QHBoxLayout()
        cam_layout.addWidget(QLabel("Camera ID:"))
        self.camera_id_spin = QSpinBox()
        self.camera_id_spin.setMinimum(0)
        self.camera_id_spin.setMaximum(10)
        self.camera_id_spin.setValue(0)
        cam_layout.addWidget(self.camera_id_spin)
        cam_layout.addStretch()
        camera_layout.addLayout(cam_layout)
        
        # Camera buttons
        btn_layout = QHBoxLayout()
        self.btn_start_camera = QPushButton("▶ Start Camera")
        self.btn_start_camera.clicked.connect(self.start_camera)
        self.btn_start_camera.setStyleSheet("background-color: #4CAF50; color: white; padding: 8px;")
        btn_layout.addWidget(self.btn_start_camera)
        
        self.btn_stop_camera = QPushButton("⏹ Stop Camera")
        self.btn_stop_camera.clicked.connect(self.stop_camera)
        self.btn_stop_camera.setEnabled(False)
        self.btn_stop_camera.setStyleSheet("background-color: #f44336; color: white; padding: 8px;")
        btn_layout.addWidget(self.btn_stop_camera)
        camera_layout.addLayout(btn_layout)
        
        self.camera_status = QLabel("Status: Camera not started")
        camera_layout.addWidget(self.camera_status)
        
        camera_group.setLayout(camera_layout)
        layout.addWidget(camera_group)
        
        # Chessboard detection
        chess_group = QGroupBox("Phone Detection (Chessboard Method)")
        chess_layout = QVBoxLayout()
        
        info_label = QLabel("Place a chessboard pattern on or near the phone to detect the plane.")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #888; font-style: italic;")
        chess_layout.addWidget(info_label)
        
        # Chessboard size
        size_layout = QHBoxLayout()
        size_layout.addWidget(QLabel("Columns:"))
        self.chess_cols = QSpinBox()
        self.chess_cols.setMinimum(3)
        self.chess_cols.setMaximum(20)
        self.chess_cols.setValue(7)
        size_layout.addWidget(self.chess_cols)
        size_layout.addWidget(QLabel("Rows:"))
        self.chess_rows = QSpinBox()
        self.chess_rows.setMinimum(3)
        self.chess_rows.setMaximum(20)
        self.chess_rows.setValue(7)
        size_layout.addWidget(self.chess_rows)
        chess_layout.addLayout(size_layout)
        
        self.btn_detect_chess = QPushButton("🔍 Detect Chessboard")
        self.btn_detect_chess.clicked.connect(self.detect_chessboard)
        chess_layout.addWidget(self.btn_detect_chess)
        
        self.chk_full_image = QCheckBox("Transform Full Image (no crop)")
        self.chk_full_image.setChecked(True)
        chess_layout.addWidget(self.chk_full_image)
        
        self.chess_status = QLabel("Status: Not detected")
        chess_layout.addWidget(self.chess_status)
        
        chess_group.setLayout(chess_layout)
        layout.addWidget(chess_group)
        
        # Manual phone corner selection
        corner_group = QGroupBox("Manual Phone Corner Selection")
        corner_layout = QVBoxLayout()
        
        info_label2 = QLabel("Or manually select 4 corners of the phone screen on the camera view.")
        info_label2.setWordWrap(True)
        info_label2.setStyleSheet("color: #888; font-style: italic;")
        corner_layout.addWidget(info_label2)
        
        self.btn_select_corners = QPushButton("📍 Select Phone Corners (4 points)")
        self.btn_select_corners.clicked.connect(self.enable_corner_selection)
        corner_layout.addWidget(self.btn_select_corners)
        
        self.btn_clear_corners = QPushButton("🗑 Clear Corners")
        self.btn_clear_corners.clicked.connect(self.clear_corners)
        corner_layout.addWidget(self.btn_clear_corners)
        
        self.corner_status = QLabel("Corners selected: 0/4")
        corner_layout.addWidget(self.corner_status)
        
        corner_group.setLayout(corner_layout)
        layout.addWidget(corner_group)
        
        layout.addStretch()
        
    def create_calibration_tab(self, parent):
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        info_label = QLabel("Calibrate coordinate mapping from phone screen to robot workspace.")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("background-color: #e3f2fd; padding: 10px; border-radius: 5px;")
        layout.addWidget(info_label)
        
        # Coordinate mapping
        mapping_group = QGroupBox("Coordinate Mapping Calibration")
        mapping_layout = QVBoxLayout()
        
        mapping_layout.addWidget(QLabel("<b>Reference Point 1</b>"))
        
        # Point 1
        p1_img_layout = QHBoxLayout()
        p1_img_layout.addWidget(QLabel("Screen (x,y):"))
        self.p1_img_x = QLineEdit()
        self.p1_img_x.setPlaceholderText("x")
        p1_img_layout.addWidget(self.p1_img_x)
        self.p1_img_y = QLineEdit()
        self.p1_img_y.setPlaceholderText("y")
        p1_img_layout.addWidget(self.p1_img_y)
        self.btn_click_p1 = QPushButton("Click")
        self.btn_click_p1.clicked.connect(self.enable_point1_selection)
        p1_img_layout.addWidget(self.btn_click_p1)
        mapping_layout.addLayout(p1_img_layout)
        
        p1_real_layout = QHBoxLayout()
        p1_real_layout.addWidget(QLabel("Robot (x,y):"))
        self.p1_real_x = QLineEdit()
        self.p1_real_x.setPlaceholderText("x (mm)")
        p1_real_layout.addWidget(self.p1_real_x)
        self.p1_real_y = QLineEdit()
        self.p1_real_y.setPlaceholderText("y (mm)")
        p1_real_layout.addWidget(self.p1_real_y)
        self.btn_go_p1 = QPushButton("Go")
        self.btn_go_p1.clicked.connect(lambda: self.go_to_current_position(1))
        p1_real_layout.addWidget(self.btn_go_p1)
        mapping_layout.addLayout(p1_real_layout)
        
        mapping_layout.addWidget(QLabel("<b>Reference Point 2</b>"))
        
        # Point 2
        p2_img_layout = QHBoxLayout()
        p2_img_layout.addWidget(QLabel("Screen (x,y):"))
        self.p2_img_x = QLineEdit()
        self.p2_img_x.setPlaceholderText("x")
        p2_img_layout.addWidget(self.p2_img_x)
        self.p2_img_y = QLineEdit()
        self.p2_img_y.setPlaceholderText("y")
        p2_img_layout.addWidget(self.p2_img_y)
        self.btn_click_p2 = QPushButton("Click")
        self.btn_click_p2.clicked.connect(self.enable_point2_selection)
        p2_img_layout.addWidget(self.btn_click_p2)
        mapping_layout.addLayout(p2_img_layout)
        
        p2_real_layout = QHBoxLayout()
        p2_real_layout.addWidget(QLabel("Robot (x,y):"))
        self.p2_real_x = QLineEdit()
        self.p2_real_x.setPlaceholderText("x (mm)")
        p2_real_layout.addWidget(self.p2_real_x)
        self.p2_real_y = QLineEdit()
        self.p2_real_y.setPlaceholderText("y (mm)")
        p2_real_layout.addWidget(self.p2_real_y)
        self.btn_go_p2 = QPushButton("Go")
        self.btn_go_p2.clicked.connect(lambda: self.go_to_current_position(2))
        p2_real_layout.addWidget(self.btn_go_p2)
        mapping_layout.addLayout(p2_real_layout)
        
        self.btn_calibrate = QPushButton("✓ Calculate Mapping Matrix")
        self.btn_calibrate.clicked.connect(self.calibrate_mapping)
        self.btn_calibrate.setStyleSheet("background-color: #2196F3; color: white; padding: 10px; font-weight: bold;")
        mapping_layout.addWidget(self.btn_calibrate)
        
        self.mapping_status = QLabel("Status: Not calibrated")
        mapping_layout.addWidget(self.mapping_status)
        
        mapping_group.setLayout(mapping_layout)
        layout.addWidget(mapping_group)

        # Mapping test
        test_group = QGroupBox("Mapping Test")
        test_layout = QVBoxLayout()

        test_info = QLabel("Enter a screen coordinate to verify its mapped robot position.")
        test_info.setWordWrap(True)
        test_info.setStyleSheet("color: #888; font-style: italic;")
        test_layout.addWidget(test_info)

        test_input_layout = QHBoxLayout()
        test_input_layout.addWidget(QLabel("Screen (x,y):"))
        self.calib_test_x = QLineEdit()
        self.calib_test_x.setPlaceholderText("x")
        test_input_layout.addWidget(self.calib_test_x)
        self.calib_test_y = QLineEdit()
        self.calib_test_y.setPlaceholderText("y")
        test_input_layout.addWidget(self.calib_test_y)
        self.btn_calib_click = QPushButton("Pick")
        self.btn_calib_click.clicked.connect(self.enable_calibration_test_selection)
        test_input_layout.addWidget(self.btn_calib_click)
        self.btn_test_mapping = QPushButton("Test Mapping")
        self.btn_test_mapping.clicked.connect(self.test_mapping_point)
        test_input_layout.addWidget(self.btn_test_mapping)
        test_layout.addLayout(test_input_layout)

        self.calib_test_result = QLabel("Result: -")
        self.calib_test_result.setStyleSheet("font-weight: bold;")
        test_layout.addWidget(self.calib_test_result)

        test_group.setLayout(test_layout)
        layout.addWidget(test_group)
        
        # Phone surface Z height
        z_group = QGroupBox("Phone Surface Height")
        z_layout = QVBoxLayout()
        
        z_info = QLabel("Set the Z height of the phone surface (where the stylus should touch).")
        z_info.setWordWrap(True)
        z_info.setStyleSheet("color: #888; font-style: italic;")
        z_layout.addWidget(z_info)
        
        z_input_layout = QHBoxLayout()
        z_input_layout.addWidget(QLabel("Z Height (mm):"))
        self.phone_z_input = QDoubleSpinBox()
        self.phone_z_input.setMinimum(-400)
        self.phone_z_input.setMaximum(-291.28)
        self.phone_z_input.setValue(0)
        self.phone_z_input.setSingleStep(0.1)
        z_input_layout.addWidget(self.phone_z_input)
        
        self.btn_measure_z = QPushButton("📏 Measure Current Z")
        self.btn_measure_z.clicked.connect(self.measure_current_z)
        z_input_layout.addWidget(self.btn_measure_z)
        z_layout.addLayout(z_input_layout)
        
        z_group.setLayout(z_layout)
        layout.addWidget(z_group)
        
        # Save/Load
        file_group = QGroupBox("Calibration Data")
        file_layout = QHBoxLayout()
        
        self.btn_save_calib = QPushButton("💾 Save")
        self.btn_save_calib.clicked.connect(self.save_calibration)
        file_layout.addWidget(self.btn_save_calib)
        
        self.btn_load_calib = QPushButton("📁 Load")
        self.btn_load_calib.clicked.connect(self.load_calibration)
        file_layout.addWidget(self.btn_load_calib)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        layout.addStretch()
        
    def refresh_robot_ports(self):
        """Populate COM port combo with currently available ports."""
        if not hasattr(self, "port_combo"):
            return

        ports = self.robot_controller.list_ports()
        current_text = self.port_combo.currentText().strip() if self.port_combo.count() else ""

        self.port_combo.blockSignals(True)
        try:
            self.port_combo.clear()
            if ports:
                self.port_combo.addItems(ports)
                if current_text in ports:
                    self.port_combo.setCurrentText(current_text)
            else:
                # Preserve ability to type a custom port manually
                self.port_combo.addItem("COM1")
                self.port_combo.setEditable(True)
        finally:
            self.port_combo.blockSignals(False)

    def create_robot_tab(self, parent):
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        # Connection
        conn_group = QGroupBox("Robot Connection")
        conn_layout = QVBoxLayout()
        
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.refresh_robot_ports()
        port_layout.addWidget(self.port_combo)
        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self.refresh_robot_ports)
        port_layout.addWidget(self.refresh_ports_btn)
        port_layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baudrate_combo.setCurrentText("115200")
        port_layout.addWidget(self.baudrate_combo)
        conn_layout.addLayout(port_layout)
        
        conn_btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("🔌 Connect")
        self.btn_connect.clicked.connect(self.connect_robot)
        self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white; padding: 8px;")
        conn_btn_layout.addWidget(self.btn_connect)
        
        self.btn_disconnect = QPushButton("❌ Disconnect")
        self.btn_disconnect.clicked.connect(self.disconnect_robot)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.setStyleSheet("background-color: #f44336; color: white; padding: 8px;")
        conn_btn_layout.addWidget(self.btn_disconnect)
        conn_layout.addLayout(conn_btn_layout)
        
        self.robot_status = QLabel("Status: Not connected")
        conn_layout.addWidget(self.robot_status)
        
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)
        
        # Position display
        pos_group = QGroupBox("Current Position")
        pos_layout = QGridLayout()
        
        pos_layout.addWidget(QLabel("X:"), 0, 0)
        self.pos_x_label = QLabel("0.0 mm")
        self.pos_x_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        pos_layout.addWidget(self.pos_x_label, 0, 1)
        
        pos_layout.addWidget(QLabel("Y:"), 1, 0)
        self.pos_y_label = QLabel("0.0 mm")
        self.pos_y_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        pos_layout.addWidget(self.pos_y_label, 1, 1)
        
        pos_layout.addWidget(QLabel("Z:"), 2, 0)
        self.pos_z_label = QLabel("0.0 mm")
        self.pos_z_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        pos_layout.addWidget(self.pos_z_label, 2, 1)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # Basic controls
        control_group = QGroupBox("Basic Controls")
        control_layout = QVBoxLayout()
        
        self.btn_home = QPushButton("🏠 Home Robot")
        self.btn_home.clicked.connect(self.home_robot)
        control_layout.addWidget(self.btn_home)
        
        self.btn_safe_height = QPushButton("⬆ Move to Safe Height")
        self.btn_safe_height.clicked.connect(self.move_to_safe_height)
        control_layout.addWidget(self.btn_safe_height)
        
        safe_z_layout = QHBoxLayout()
        safe_z_layout.addWidget(QLabel("Safe Z Height:"))
        self.safe_z_input = QDoubleSpinBox()
        self.safe_z_input.setMinimum(self.robot_home_z - 200.0)
        self.safe_z_input.setMaximum(-291.28)
        self.safe_z_input.setValue(-350)
        self.safe_z_input.setSingleStep(1)
        safe_z_layout.addWidget(self.safe_z_input)
        safe_z_layout.addWidget(QLabel("mm"))
        control_layout.addLayout(safe_z_layout)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Manual jog
        jog_group = QGroupBox("Manual Jog Control")
        jog_layout = QGridLayout()
        
        jog_layout.addWidget(QLabel("Step Size:"), 0, 0)
        self.jog_step = QDoubleSpinBox()
        self.jog_step.setMinimum(0.1)
        self.jog_step.setMaximum(50)
        self.jog_step.setValue(10)
        self.jog_step.setSingleStep(1)
        jog_layout.addWidget(self.jog_step, 0, 1)
        jog_layout.addWidget(QLabel("mm"), 0, 2)
        
        # XY jog buttons
        self.btn_y_plus = QPushButton("↑ Y+")
        self.btn_y_plus.clicked.connect(lambda: self.jog_robot('Y', 1))
        jog_layout.addWidget(self.btn_y_plus, 1, 1)
        
        self.btn_x_minus = QPushButton("← X-")
        self.btn_x_minus.clicked.connect(lambda: self.jog_robot('X', -1))
        jog_layout.addWidget(self.btn_x_minus, 2, 0)
        
        self.btn_x_plus = QPushButton("→ X+")
        self.btn_x_plus.clicked.connect(lambda: self.jog_robot('X', 1))
        jog_layout.addWidget(self.btn_x_plus, 2, 2)
        
        self.btn_y_minus = QPushButton("↓ Y-")
        self.btn_y_minus.clicked.connect(lambda: self.jog_robot('Y', -1))
        jog_layout.addWidget(self.btn_y_minus, 3, 1)
        
        # Z jog buttons
        self.btn_z_plus = QPushButton("⬆ Z+")
        self.btn_z_plus.clicked.connect(lambda: self.jog_robot('Z', 1))
        jog_layout.addWidget(self.btn_z_plus, 1, 4)
        
        self.btn_z_minus = QPushButton("⬇ Z-")
        self.btn_z_minus.clicked.connect(lambda: self.jog_robot('Z', -1))
        jog_layout.addWidget(self.btn_z_minus, 2, 4)
        
        jog_group.setLayout(jog_layout)
        layout.addWidget(jog_group)
        
        layout.addStretch()
        
    def create_touch_tab(self, parent):
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        info_label = QLabel("Control touch interactions with the phone screen.")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("background-color: #fff3e0; padding: 10px; border-radius: 5px;")
        layout.addWidget(info_label)
        
        # Touch settings
        touch_group = QGroupBox("Touch Settings")
        touch_layout = QVBoxLayout()
        
        # Touch force
        force_layout = QHBoxLayout()
        force_layout.addWidget(QLabel("Touch Force:"))
        self.touch_force_spin = QDoubleSpinBox()
        self.touch_force_spin.setMinimum(0.1)
        self.touch_force_spin.setMaximum(20)
        self.touch_force_spin.setValue(1.0)
        self.touch_force_spin.setSingleStep(0.5)
        force_layout.addWidget(self.touch_force_spin)
        force_layout.addWidget(QLabel("mm (down from surface)"))
        touch_layout.addLayout(force_layout)
        
        # Touch duration
        duration_layout = QHBoxLayout()
        duration_layout.addWidget(QLabel("Touch Duration:"))
        self.touch_duration_spin = QDoubleSpinBox()
        self.touch_duration_spin.setMinimum(0.01)
        self.touch_duration_spin.setMaximum(5.0)
        self.touch_duration_spin.setValue(0.1)
        self.touch_duration_spin.setSingleStep(0.01)
        duration_layout.addWidget(self.touch_duration_spin)
        duration_layout.addWidget(QLabel("seconds"))
        touch_layout.addLayout(duration_layout)
        
        # Movement speed
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Movement Speed:"))
        self.move_speed_spin = QSpinBox()
        self.move_speed_spin.setMinimum(10)
        self.move_speed_spin.setMaximum(1000)
        self.move_speed_spin.setValue(100)
        self.move_speed_spin.setSingleStep(10)
        speed_layout.addWidget(self.move_speed_spin)
        speed_layout.addWidget(QLabel("mm/s"))
        touch_layout.addLayout(speed_layout)
        
        touch_group.setLayout(touch_layout)
        layout.addWidget(touch_group)
        
        # Touch modes
        mode_group = QGroupBox("Touch Modes")
        mode_layout = QVBoxLayout()
        
        self.chk_click_to_touch = QCheckBox("Click to Touch (click on transformed view)")
        self.chk_click_to_touch.setChecked(False)
        self.chk_click_to_touch.stateChanged.connect(self.toggle_click_to_touch)
        mode_layout.addWidget(self.chk_click_to_touch)
        
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # Quick test
        test_group = QGroupBox("Test Touch")
        test_layout = QVBoxLayout()
        
        test_coord_layout = QHBoxLayout()
        test_coord_layout.addWidget(QLabel("Test Position (x,y):"))
        self.test_x_input = QLineEdit()
        self.test_x_input.setPlaceholderText("x")
        test_coord_layout.addWidget(self.test_x_input)
        self.test_y_input = QLineEdit()
        self.test_y_input.setPlaceholderText("y")
        test_coord_layout.addWidget(self.test_y_input)
        self.btn_click_test = QPushButton("Click")
        self.btn_click_test.clicked.connect(self.enable_test_selection)
        test_coord_layout.addWidget(self.btn_click_test)
        test_layout.addLayout(test_coord_layout)
        
        self.test_result_label = QLabel("Robot coordinates: -")
        self.test_result_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        test_layout.addWidget(self.test_result_label)
        
        test_btn_layout = QHBoxLayout()
        self.btn_test_touch = QPushButton("👆 Execute Touch")
        self.btn_test_touch.clicked.connect(self.execute_test_touch)
        self.btn_test_touch.setStyleSheet("background-color: #FF9800; color: white; padding: 10px; font-weight: bold;")
        test_btn_layout.addWidget(self.btn_test_touch)
        
        self.btn_goto_test = QPushButton("➜ Go To (Safe Height)")
        self.btn_goto_test.clicked.connect(self.goto_test_position)
        test_btn_layout.addWidget(self.btn_goto_test)
        test_layout.addLayout(test_btn_layout)
        
        test_group.setLayout(test_layout)
        layout.addWidget(test_group)
        
        # Gesture controls
        gesture_group = QGroupBox("Gesture Recording (Future)")
        gesture_layout = QVBoxLayout()
        
        gesture_info = QLabel("Record and playback touch gestures (swipe, drag, multi-touch).")
        gesture_info.setWordWrap(True)
        gesture_info.setStyleSheet("color: #888; font-style: italic;")
        gesture_layout.addWidget(gesture_info)
        
        gesture_btn_layout = QHBoxLayout()
        self.btn_record_gesture = QPushButton("⏺ Record Gesture")
        self.btn_record_gesture.setEnabled(False)
        gesture_btn_layout.addWidget(self.btn_record_gesture)
        
        self.btn_play_gesture = QPushButton("▶ Play Gesture")
        self.btn_play_gesture.setEnabled(False)
        gesture_btn_layout.addWidget(self.btn_play_gesture)
        gesture_layout.addLayout(gesture_btn_layout)
        
        gesture_group.setLayout(gesture_layout)
        layout.addWidget(gesture_group)
        
        layout.addStretch()
        
    # ========== Camera Functions ==========
    
    def start_camera(self):
        camera_id = self.camera_id_spin.value()
        self.camera = cv2.VideoCapture(camera_id)
        if self.camera.isOpened():
            self.timer.start(30)
            self.btn_start_camera.setEnabled(False)
            self.btn_stop_camera.setEnabled(True)
            self.camera_status.setText("Status: Camera running")
            self.camera_status.setStyleSheet("color: green;")
            self.log_status(f"Camera {camera_id} started successfully.")
        else:
            QMessageBox.warning(self, "Error", f"Cannot open camera {camera_id}")
            self.log_status(f"ERROR: Cannot open camera {camera_id}")
            
    def stop_camera(self):
        if self.camera:
            self.timer.stop()
            self.camera.release()
            self.camera = None
            self.btn_start_camera.setEnabled(True)
            self.btn_stop_camera.setEnabled(False)
            self.camera_status.setText("Status: Camera stopped")
            self.camera_status.setStyleSheet("color: red;")
            self.log_status("Camera stopped.")
            
    def update_frame(self):
        if self.camera and self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                self.current_frame = frame.copy()
                display_frame = frame.copy()
                
                # Draw phone corners if selected
                if len(self.phone_corners) > 0:
                    for i, corner in enumerate(self.phone_corners):
                        cv2.circle(display_frame, tuple(corner), 8, (0, 255, 0), -1)
                        cv2.putText(display_frame, f"C{i+1}", 
                                   (corner[0] + 10, corner[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    if len(self.phone_corners) == 4:
                        # Draw phone outline
                        pts = np.array(self.phone_corners, np.int32)
                        cv2.polylines(display_frame, [pts], True, (0, 255, 0), 2)
                
                # Convert and display
                rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                    self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.camera_label.setPixmap(scaled_pixmap)
                
                # Update transformed view if available
                if self.transformation_matrix is not None:
                    self.update_transformed_view()
    
    # ========== Detection Functions ==========
    
    def detect_chessboard(self):
        if self.current_frame is None:
            QMessageBox.warning(self, "Error", "No camera frame available")
            return
            
        self.chessboard_size = (self.chess_cols.value(), self.chess_rows.value())
        gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            self.corners = corners
            self.chessboard_found = True
            
            self.calculate_transformation_matrix()
            
            self.chess_status.setText("Status: Chessboard detected!")
            self.chess_status.setStyleSheet("color: green;")
            self.log_status("Chessboard detected and transformation matrix calculated.")
            QMessageBox.information(self, "Success", "Chessboard detected successfully!")
        else:
            self.chessboard_found = False
            self.chess_status.setText("Status: Chessboard not found")
            self.chess_status.setStyleSheet("color: red;")
            self.log_status("ERROR: Chessboard not found.")
            QMessageBox.warning(self, "Error", "Chessboard not found.")
            
    def calculate_transformation_matrix(self):
        if not self.chessboard_found or self.corners is None or self.current_frame is None:
            return
            
        # Get four corner points
        top_left = self.corners[0][0]
        top_right = self.corners[self.chessboard_size[0] - 1][0]
        bottom_left = self.corners[-self.chessboard_size[0]][0]
        bottom_right = self.corners[-1][0]
        
        chessboard_src = np.float32([top_left, top_right, bottom_left, bottom_right])
        
        # Calculate output size
        width_top = np.linalg.norm(top_right - top_left)
        width_bottom = np.linalg.norm(bottom_right - bottom_left)
        chess_width = max(width_top, width_bottom)
        
        height_left = np.linalg.norm(bottom_left - top_left)
        height_right = np.linalg.norm(bottom_right - top_right)
        chess_height = max(height_left, height_right)
        
        max_dimension = 800
        scale = max_dimension / max(chess_width, chess_height)
        chess_output_width = int(chess_width * scale)
        chess_output_height = int(chess_height * scale)
        
        chessboard_dst = np.float32([
            [0, 0],
            [chess_output_width, 0],
            [0, chess_output_height],
            [chess_output_width, chess_output_height]
        ])
        
        M = cv2.getPerspectiveTransform(chessboard_src, chessboard_dst)
        
        if self.chk_full_image.isChecked():
            h, w = self.current_frame.shape[:2]
            image_corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
            transformed_corners = cv2.perspectiveTransform(image_corners, M)
            
            x_coords = transformed_corners[:, 0, 0]
            y_coords = transformed_corners[:, 0, 1]
            
            min_x, max_x = x_coords.min(), x_coords.max()
            min_y, max_y = y_coords.min(), y_coords.max()
            
            translation_matrix = np.array([
                [1, 0, -min_x],
                [0, 1, -min_y],
                [0, 0, 1]
            ], dtype=np.float32)
            
            self.transformation_matrix = translation_matrix @ M
            
            output_width = int(np.ceil(max_x - min_x))
            output_height = int(np.ceil(max_y - min_y))
            self.transformed_size = (output_width, output_height)
        else:
            self.transformation_matrix = M
            self.transformed_size = (chess_output_width, chess_output_height)
            
    def update_transformed_view(self):
        if self.current_frame is None or self.transformation_matrix is None:
            return
            
        self.transformed_frame = cv2.warpPerspective(
            self.current_frame,
            self.transformation_matrix,
            self.transformed_size
        )
        
        display_frame = self.transformed_frame.copy()
        
        # Draw calibration points
        if self.calibration_points:
            for i, (img_pt, real_pt) in enumerate(self.calibration_points):
                cv2.circle(display_frame, (int(img_pt[0]), int(img_pt[1])), 8, (0, 255, 0), -1)
                cv2.putText(display_frame, f"P{i+1}",
                           (int(img_pt[0]) + 10, int(img_pt[1]) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw temp click point
        if self.temp_click_point:
            cv2.circle(display_frame, self.temp_click_point, 5, (255, 0, 0), -1)
            cv2.drawMarker(display_frame, self.temp_click_point, (255, 0, 0),
                          cv2.MARKER_CROSS, 20, 2)
        
        rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
            self.transformed_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.transformed_label.setPixmap(scaled_pixmap)
        
    def enable_corner_selection(self):
        self.phone_corners = []
        self.click_mode = 'corners'
        self.corner_status.setText("Corners selected: 0/4 - Click on camera view")
        self.log_status("Corner selection mode activated. Click 4 corners of the phone.")
        
    def clear_corners(self):
        self.phone_corners = []
        self.corner_status.setText("Corners selected: 0/4")
        self.transformation_matrix = None
        self.log_status("Phone corners cleared.")
        
    def on_camera_click(self, pos):
        if self.click_mode == 'corners' and len(self.phone_corners) < 4:
            # Get click position in image coordinates
            label_size = self.camera_label.size()
            pixmap = self.camera_label.pixmap()
            if pixmap is None or self.current_frame is None:
                return
                
            pixmap_size = pixmap.size()
            x_offset = (label_size.width() - pixmap_size.width()) / 2
            y_offset = (label_size.height() - pixmap_size.height()) / 2
            
            click_x = pos.x() - x_offset
            click_y = pos.y() - y_offset
            
            if click_x < 0 or click_y < 0 or click_x >= pixmap_size.width() or click_y >= pixmap_size.height():
                return
                
            scale_x = self.current_frame.shape[1] / pixmap_size.width()
            scale_y = self.current_frame.shape[0] / pixmap_size.height()
            
            img_x = int(click_x * scale_x)
            img_y = int(click_y * scale_y)
            
            self.phone_corners.append([img_x, img_y])
            self.corner_status.setText(f"Corners selected: {len(self.phone_corners)}/4")
            self.log_status(f"Corner {len(self.phone_corners)} selected: ({img_x}, {img_y})")
            
            if len(self.phone_corners) == 4:
                self.click_mode = None
                self.calculate_transformation_from_corners()
                
    def calculate_transformation_from_corners(self):
        if len(self.phone_corners) != 4:
            return
            
        # Calculate output size based on corner distances
        corners = np.array(self.phone_corners, dtype=np.float32)
        
        width_top = np.linalg.norm(corners[1] - corners[0])
        width_bottom = np.linalg.norm(corners[3] - corners[2])
        width = max(width_top, width_bottom)
        
        height_left = np.linalg.norm(corners[2] - corners[0])
        height_right = np.linalg.norm(corners[3] - corners[1])
        height = max(height_left, height_right)
        
        output_width = int(width)
        output_height = int(height)
        
        dst_points = np.float32([
            [0, 0],
            [output_width, 0],
            [0, output_height],
            [output_width, output_height]
        ])
        
        self.transformation_matrix = cv2.getPerspectiveTransform(corners, dst_points)
        self.transformed_size = (output_width, output_height)
        
        self.log_status(f"Transformation matrix calculated from manual corners. Output size: {output_width}x{output_height}")
        QMessageBox.information(self, "Success", "Phone transformation calculated!")
        
    # ========== Calibration Functions ==========
    
    def enable_point1_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", "Please detect phone first!")
            return
        self.click_mode = 'point1'
        self.btn_click_p1.setStyleSheet("background-color: yellow;")
        self.log_status("Point 1 selection mode. Click on transformed view.")
        
    def enable_point2_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", "Please detect phone first!")
            return
        self.click_mode = 'point2'
        self.btn_click_p2.setStyleSheet("background-color: yellow;")
        self.log_status("Point 2 selection mode. Click on transformed view.")
        
    def enable_test_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", "Please detect phone first!")
            return
        self.click_mode = 'test'
        self.btn_click_test.setStyleSheet("background-color: yellow;")
        self.log_status("Test point selection mode. Click on transformed view.")

    def enable_calibration_test_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", "Please detect phone first!")
            return
        self.click_mode = 'calib_test'
        if hasattr(self, "btn_calib_click"):
            self.btn_calib_click.setStyleSheet("background-color: yellow;")
        self.log_status("Calibration test mode. Click on transformed view to map coordinates.")
        
    def on_transformed_click(self, pos):
        if self.transformed_frame is None:
            return
            
        label_size = self.transformed_label.size()
        pixmap = self.transformed_label.pixmap()
        if pixmap is None:
            return
            
        pixmap_size = pixmap.size()
        x_offset = (label_size.width() - pixmap_size.width()) / 2
        y_offset = (label_size.height() - pixmap_size.height()) / 2
        
        click_x = pos.x() - x_offset
        click_y = pos.y() - y_offset
        
        if click_x < 0 or click_y < 0 or click_x >= pixmap_size.width() or click_y >= pixmap_size.height():
            return
            
        scale_x = self.transformed_frame.shape[1] / pixmap_size.width()
        scale_y = self.transformed_frame.shape[0] / pixmap_size.height()
        
        img_x = int(click_x * scale_x)
        img_y = int(click_y * scale_y)
        
        self.temp_click_point = (img_x, img_y)
        
        handled_selection = False

        if self.click_mode == 'point1':
            self.p1_img_x.setText(str(img_x))
            self.p1_img_y.setText(str(img_y))
            self.btn_click_p1.setStyleSheet("")
            self.log_status(f"Point 1 selected: ({img_x}, {img_y})")
            handled_selection = True
        elif self.click_mode == 'point2':
            self.p2_img_x.setText(str(img_x))
            self.p2_img_y.setText(str(img_y))
            self.btn_click_p2.setStyleSheet("")
            self.log_status(f"Point 2 selected: ({img_x}, {img_y})")
            handled_selection = True
        elif self.click_mode == 'test':
            self.test_x_input.setText(str(img_x))
            self.test_y_input.setText(str(img_y))
            self.btn_click_test.setStyleSheet("")
            if self.mapping_matrix is not None:
                self.calculate_test_coordinates()
            self.log_status(f"Test point selected: ({img_x}, {img_y})")
            handled_selection = True
        elif self.click_mode == 'calib_test':
            self.calib_test_x.setText(str(img_x))
            self.calib_test_y.setText(str(img_y))
            if hasattr(self, "btn_calib_click"):
                self.btn_calib_click.setStyleSheet("")
            self.test_mapping_point()
            self.log_status(f"Calibration test point: ({img_x}, {img_y})")
            handled_selection = True

        if handled_selection:
            self.click_mode = None
            return

        # No explicit selection in progress – handle click-to-touch when enabled
        if self.chk_click_to_touch.isChecked():
            self.handle_click_to_touch(img_x, img_y)
        
    def go_to_current_position(self, point_num):
        """Move robot to current position and update the coordinate field"""
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        # In real implementation, get current position from robot
        x, y = self.robot_position[0], self.robot_position[1]
        
        if point_num == 1:
            self.p1_real_x.setText(f"{x:.2f}")
            self.p1_real_y.setText(f"{y:.2f}")
        else:
            self.p2_real_x.setText(f"{x:.2f}")
            self.p2_real_y.setText(f"{y:.2f}")
        
        self.log_status(f"Point {point_num} set to current robot position: ({x:.2f}, {y:.2f})")

    def image_point_to_mapping_space(self, img_x, img_y):
        """Convert image point to a Cartesian mapping space (Y axis pointing up)."""
        height = None
        if self.transformed_frame is not None:
            height = self.transformed_frame.shape[0]
        elif self.transformed_size is not None:
            height = self.transformed_size[1]

        if not height:
            return np.array([img_x, img_y], dtype=float)

        flipped_y = height - img_y
        return np.array([img_x, flipped_y], dtype=float)
    
    def get_motion_feedrate(self):
        """Return motion feedrate (mm/min) derived from UI speed control."""
        speed_mm_per_s = float(self.move_speed_spin.value())
        # G-code feedrates expect mm/min; clamp to a minimal positive value
        feedrate = max(speed_mm_per_s * 60.0, 1.0)
        return feedrate
        
    def calibrate_mapping(self):
        try:
            p1_img_x = float(self.p1_img_x.text())
            p1_img_y = float(self.p1_img_y.text())
            p1_real_x = float(self.p1_real_x.text())
            p1_real_y = float(self.p1_real_y.text())
            
            p2_img_x = float(self.p2_img_x.text())
            p2_img_y = float(self.p2_img_y.text())
            p2_real_x = float(self.p2_real_x.text())
            p2_real_y = float(self.p2_real_y.text())
            
            self.calibration_points = [
                ((p1_img_x, p1_img_y), (p1_real_x, p1_real_y)),
                ((p2_img_x, p2_img_y), (p2_real_x, p2_real_y))
            ]
            
            p1_img_vec = self.image_point_to_mapping_space(p1_img_x, p1_img_y)
            p2_img_vec = self.image_point_to_mapping_space(p2_img_x, p2_img_y)
            p1_img_vec = self.image_point_to_mapping_space(p1_img_x, p1_img_y)
            p2_img_vec = self.image_point_to_mapping_space(p2_img_x, p2_img_y)

            img_dx = p2_img_vec[0] - p1_img_vec[0]
            img_dy = p2_img_vec[1] - p1_img_vec[1]
            real_dx = p2_real_x - p1_real_x
            real_dy = p2_real_y - p1_real_y

            if abs(img_dx) < 1e-6 or abs(img_dy) < 1e-6:
                QMessageBox.warning(self, "Error", "Calibration points must differ in both X and Y to define the mapping axes.")
                return

            scale_x = real_dx / img_dx
            scale_y = real_dy / img_dy

            rot_scale_matrix = np.array([[scale_x, 0.0],
                                         [0.0, scale_y]])
            p1_real_vec = np.array([p1_real_x, p1_real_y])
            translation = p1_real_vec - rot_scale_matrix @ p1_img_vec

            self.mapping_matrix = {
                'rotation_scale': rot_scale_matrix,
                'translation': translation
            }

            self.mapping_status.setText("Status: Calibrated successfully!")
            self.mapping_status.setStyleSheet("color: green;")
            self.log_status(f"Mapping calibrated! ScaleX: {scale_x:.4f} mm/pixel, ScaleY: {scale_y:.4f} mm/pixel")
            QMessageBox.information(self, "Success", 
                f"Mapping calibrated!\nScaleX: {scale_x:.4f} mm/pixel\nScaleY: {scale_y:.4f} mm/pixel")
            
        except ValueError:
            QMessageBox.warning(self, "Error", "Please enter valid numbers")
            
    def image_to_real_coordinates(self, img_x, img_y):
        """Convert screen coordinates to robot coordinates"""
        if self.mapping_matrix is None:
            return None
            
        img_point = self.image_point_to_mapping_space(img_x, img_y)
        real_point = self.mapping_matrix['rotation_scale'] @ img_point + self.mapping_matrix['translation']
        
        return real_point[0], real_point[1]
    
    def run_touch_sequence(self, real_x, real_y, phone_z, touch_force, touch_duration, safe_z, feedrate, show_errors=True):
        """Execute the touch motion sequence. Returns True on success."""
        touch_z = phone_z - touch_force
        try:
            self.log_status("  1. Moving to safe Z height before travel")
            self.log_robot_responses(self.robot_controller.move_linear_absolute(z=safe_z, feedrate=feedrate))

            self.log_status(f"  2. Moving in XY to target ({real_x:.2f}, {real_y:.2f})")
            self.log_robot_responses(self.robot_controller.move_linear_absolute(x=real_x, y=real_y, feedrate=feedrate))

            self.log_status(f"  3. Lowering to touch surface Z={touch_z:.2f}")
            self.log_robot_responses(self.robot_controller.move_linear_absolute(z=touch_z, feedrate=feedrate))

            self.log_status(f"  4. Holding for {touch_duration:.2f}s")
            self.log_robot_responses(self.robot_controller.dwell(touch_duration))

            self.log_status(f"  5. Lifting back to safe height Z={safe_z:.2f}")
            self.log_robot_responses(self.robot_controller.move_linear_absolute(z=safe_z, feedrate=feedrate))
        except Exception as exc:
            self.log_status(f"Touch sequence failed: {exc}")
            if show_errors:
                QMessageBox.critical(self, "Error", f"Touch sequence failed:\n{exc}")
            return False

        self.robot_position = self.robot_controller.get_position().tolist()
        self.update_position_display()
        self.log_status("Touch completed successfully!")
        return True
        
    def calculate_test_coordinates(self):
        try:
            img_x = float(self.test_x_input.text())
            img_y = float(self.test_y_input.text())
            
            if self.mapping_matrix:
                real_x, real_y = self.image_to_real_coordinates(img_x, img_y)
                self.test_result_label.setText(f"Robot coordinates: ({real_x:.2f}, {real_y:.2f}) mm")
            else:
                self.test_result_label.setText("Robot coordinates: Not calibrated")
        except:
            pass

    def test_mapping_point(self):
        """Test mapping for a manually entered screen coordinate."""
        if self.mapping_matrix is None:
            QMessageBox.warning(self, "Error", "Please calibrate mapping first!")
            return

        try:
            img_x = float(self.calib_test_x.text())
            img_y = float(self.calib_test_y.text())
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid screen coordinates")
            return

        result = self.image_to_real_coordinates(img_x, img_y)
        if result is None:
            QMessageBox.warning(self, "Error", "Mapping matrix not available")
            return

        real_x, real_y = result
        self.calib_test_result.setText(f"Result: Robot ({real_x:.2f}, {real_y:.2f}) mm")
        self.log_status(f"Mapping test -> Screen ({img_x:.2f}, {img_y:.2f}) -> Robot ({real_x:.2f}, {real_y:.2f})")
            
    def measure_current_z(self):
        """Set phone Z height to current robot Z position"""
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        if self.robot_controller.is_connected():
            self.robot_position = self.robot_controller.get_position().tolist()
        current_z = self.robot_position[2]
        self.phone_z_input.setValue(current_z)
        self.log_status(f"Phone Z height set to current position: {current_z:.2f} mm")
        
    def save_calibration(self):
        if self.mapping_matrix is None:
            QMessageBox.warning(self, "Error", "No calibration data to save")
            return
            
        filename, _ = QFileDialog.getSaveFileName(self, "Save Calibration", "", "NumPy Files (*.npz)")
        if filename:
            np.savez(filename,
                    rotation_scale=self.mapping_matrix['rotation_scale'],
                    translation=self.mapping_matrix['translation'],
                    calibration_points=np.array(self.calibration_points, dtype=object),
                    transformation_matrix=self.transformation_matrix if self.transformation_matrix is not None else np.array([]),
                    phone_z=self.phone_z_input.value())
            self.log_status(f"Calibration saved to {filename}")
            QMessageBox.information(self, "Success", "Calibration saved!")
            
    def load_calibration(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Calibration", "", "NumPy Files (*.npz)")
        if filename:
            data = np.load(filename, allow_pickle=True)
            self.mapping_matrix = {
                'rotation_scale': data['rotation_scale'],
                'translation': data['translation']
            }
            
            if 'calibration_points' in data:
                self.calibration_points = data['calibration_points'].tolist()
                if len(self.calibration_points) >= 2:
                    p1_img, p1_real = self.calibration_points[0]
                    p2_img, p2_real = self.calibration_points[1]
                    
                    self.p1_img_x.setText(str(p1_img[0]))
                    self.p1_img_y.setText(str(p1_img[1]))
                    self.p1_real_x.setText(str(p1_real[0]))
                    self.p1_real_y.setText(str(p1_real[1]))
                    
                    self.p2_img_x.setText(str(p2_img[0]))
                    self.p2_img_y.setText(str(p2_img[1]))
                    self.p2_real_x.setText(str(p2_real[0]))
                    self.p2_real_y.setText(str(p2_real[1]))
            
            if 'transformation_matrix' in data and data['transformation_matrix'].size > 0:
                self.transformation_matrix = data['transformation_matrix']
                
            if 'phone_z' in data:
                self.phone_z_input.setValue(float(data['phone_z']))
                
            self.mapping_status.setText("Status: Calibration loaded!")
            self.mapping_status.setStyleSheet("color: green;")
            self.log_status(f"Calibration loaded from {filename}")
            QMessageBox.information(self, "Success", "Calibration loaded!")
            
    # ========== Robot Control Functions ==========
    
    def connect_robot(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "Error", "Please select a COM port")
            return

        try:
            baudrate = int(self.baudrate_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid baudrate selected")
            return

        self.log_status(f"Connecting to robot on {port} @ {baudrate} baud...")
        success, error_message = self.robot_controller.connect(port, baudrate)

        if not success:
            self.log_status(f"Robot connection failed: {error_message}")
            QMessageBox.critical(self, "Connection Failed", f"Unable to connect to robot on {port}.\n{error_message}")
            self.refresh_robot_ports()
            return

        self.robot_connected = True
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.robot_status.setText(f"Status: Connected to {port}")
        self.robot_status.setStyleSheet("color: green;")
        self.robot_position = self.robot_controller.get_position().tolist()
        self.update_position_display()
        self.log_status("Robot connection established successfully")
        QMessageBox.information(self, "Connected", f"Connected to Delta robot on {port}")
        
    def disconnect_robot(self):
        if self.robot_connected:
            self.robot_controller.disconnect()
            self.robot_connected = False

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.robot_status.setText("Status: Disconnected")
        self.robot_status.setStyleSheet("color: red;")
        self.robot_position = [0.0, 0.0, self.robot_home_z]
        self.update_position_display()
        self.log_status("Robot disconnected")
        self.refresh_robot_ports()
        
    def home_robot(self):
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        self.log_status("Sending home command (G28)")
        try:
            responses = self.robot_controller.home()
        except Exception as exc:
            self.log_status(f"Homing failed: {exc}")
            QMessageBox.critical(self, "Error", f"Failed to home robot:\n{exc}")
            return
        self.log_robot_responses(responses)

        # Assume home position is origin and move to configured safe Z afterwards
        safe_z = -350
        try:
            move_responses = self.robot_controller.move_linear_absolute(z=safe_z)
            self.log_robot_responses(move_responses)
        except Exception as exc:
            self.log_status(f"Failed to move to safe height after homing: {exc}")

        self.robot_position = self.robot_controller.get_position().tolist()
        self.update_position_display()
        self.log_status("Robot homed successfully")
        
    def move_to_safe_height(self):
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        safe_z = self.safe_z_input.value()
        self.log_status(f"Moving to safe Z height: {safe_z:.2f} mm")
        try:
            responses = self.robot_controller.move_linear_absolute(z=safe_z)
            self.log_robot_responses(responses)
        except Exception as exc:
            self.log_status(f"Failed to move to safe height: {exc}")
            QMessageBox.critical(self, "Error", f"Failed to move to safe height:\n{exc}")
            return

        self.robot_position = self.robot_controller.get_position().tolist()
        self.update_position_display()
        self.log_status(f"Reached safe height at Z={safe_z:.2f} mm")
        
    def jog_robot(self, axis, direction):
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        step = self.jog_step.value() * direction
        dx = dy = dz = 0.0
        
        if axis == 'X':
            dx = step
        elif axis == 'Y':
            dy = step
        elif axis == 'Z':
            dz = step
        else:
            return

        self.log_status(f"Jogging {axis} axis by {step:.2f} mm")
        try:
            responses = self.robot_controller.move_linear_relative(dx=dx, dy=dy, dz=dz)
            self.log_robot_responses(responses)
        except Exception as exc:
            self.log_status(f"Jog command failed: {exc}")
            QMessageBox.critical(self, "Error", f"Failed to jog robot:\n{exc}")
            return

        self.robot_position = self.robot_controller.get_position().tolist()
        self.update_position_display()
        
    def update_position_display(self):
        if self.robot_connected and self.robot_controller.is_connected():
            position = self.robot_controller.get_position()
            self.robot_position = position.tolist()
        else:
            position = np.array(self.robot_position, dtype=float)

        self.pos_x_label.setText(f"{position[0]:.2f} mm")
        self.pos_y_label.setText(f"{position[1]:.2f} mm")
        self.pos_z_label.setText(f"{position[2]:.2f} mm")
        
    # ========== Touch Functions ==========
    
    def toggle_click_to_touch(self, state):
        if state == Qt.Checked:
            if self.mapping_matrix is None:
                QMessageBox.warning(self, "Error", "Please calibrate mapping before enabling click-to-touch.")
                self.chk_click_to_touch.blockSignals(True)
                self.chk_click_to_touch.setChecked(False)
                self.chk_click_to_touch.blockSignals(False)
                return
            if not self.robot_connected:
                QMessageBox.warning(self, "Error", "Connect the robot before enabling click-to-touch.")
                self.chk_click_to_touch.blockSignals(True)
                self.chk_click_to_touch.setChecked(False)
                self.chk_click_to_touch.blockSignals(False)
                return
            self.log_status("Click-to-touch enabled. Click the transformed view to execute touches.")
        else:
            self.log_status("Click-to-touch disabled")
        self.click_mode = None
    
    def handle_click_to_touch(self, img_x, img_y):
        """Execute touch sequence triggered by clicking the transformed view."""
        if not self.chk_click_to_touch.isChecked():
            return
        if self.mapping_matrix is None:
            self.log_status("Click-to-touch ignored: mapping not calibrated.")
            return
        if not self.robot_connected:
            self.log_status("Click-to-touch ignored: robot not connected.")
            return

        coords = self.image_to_real_coordinates(img_x, img_y)
        if coords is None:
            self.log_status("Click-to-touch failed: unable to map coordinates.")
            return

        real_x, real_y = coords
        self.test_x_input.setText(str(img_x))
        self.test_y_input.setText(str(img_y))
        self.calculate_test_coordinates()

        phone_z = self.phone_z_input.value()
        touch_force = self.touch_force_spin.value()
        touch_duration = self.touch_duration_spin.value()
        safe_z = self.safe_z_input.value()
        feedrate = self.get_motion_feedrate()

        self.log_status(f"Click-to-touch executing at screen ({img_x}, {img_y}) -> robot ({real_x:.2f}, {real_y:.2f})")
        self.run_touch_sequence(real_x, real_y, phone_z, touch_force, touch_duration, safe_z, feedrate, show_errors=False)
            
    def execute_test_touch(self):
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        if self.mapping_matrix is None:
            QMessageBox.warning(self, "Error", "Please calibrate mapping first!")
            return
            
        try:
            img_x = float(self.test_x_input.text())
            img_y = float(self.test_y_input.text())
            
            real_x, real_y = self.image_to_real_coordinates(img_x, img_y)
            phone_z = self.phone_z_input.value()
            touch_force = self.touch_force_spin.value()
            touch_duration = self.touch_duration_spin.value()
            safe_z = self.safe_z_input.value()
            feedrate = self.get_motion_feedrate()
            
            self.log_status(f"Executing touch at screen ({img_x:.0f}, {img_y:.0f}) -> robot ({real_x:.2f}, {real_y:.2f})")
            if not self.run_touch_sequence(real_x, real_y, phone_z, touch_force, touch_duration, safe_z, feedrate, show_errors=True):
                return
            
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid coordinates")
            
    def goto_test_position(self):
        if not self.robot_connected:
            QMessageBox.warning(self, "Error", "Robot not connected!")
            return
            
        if self.mapping_matrix is None:
            QMessageBox.warning(self, "Error", "Please calibrate mapping first!")
            return
            
        try:
            img_x = float(self.test_x_input.text())
            img_y = float(self.test_y_input.text())
            
            real_x, real_y = self.image_to_real_coordinates(img_x, img_y)
            safe_z = self.safe_z_input.value()
            feedrate = self.get_motion_feedrate()
            self.log_status("Moving to target position with safe travel sequence")

            try:
                self.log_status(f"  1. Raising to safe Z={safe_z:.2f}")
                self.log_robot_responses(self.robot_controller.move_linear_absolute(z=safe_z, feedrate=feedrate))

                self.log_status(f"  2. Moving in XY to ({real_x:.2f}, {real_y:.2f})")
                self.log_robot_responses(self.robot_controller.move_linear_absolute(x=real_x, y=real_y, feedrate=feedrate))
            except Exception as exc:
                self.log_status(f"Failed to move to target: {exc}")
                QMessageBox.critical(self, "Error", f"Failed to move to target:\n{exc}")
                return

            self.robot_position = self.robot_controller.get_position().tolist()
            self.update_position_display()
            self.log_status(f"Arrived at ({real_x:.2f}, {real_y:.2f}, {self.robot_position[2]:.2f}) mm")
            
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid coordinates")
            
    # ========== Utility Functions ==========
    
    def log_status(self, message):
        """Add message to status log"""
        self.status_text.append(message)
        # Auto scroll to bottom
        scrollbar = self.status_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def log_robot_responses(self, responses):
        """Helper to log each response line from robot controller."""
        for line in responses or []:
            if line:
                self.log_status(f"  {line}")
        
    def closeEvent(self, event):
        self.stop_camera()
        if self.robot_controller.is_connected():
            self.robot_controller.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    window = SmartHandApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

