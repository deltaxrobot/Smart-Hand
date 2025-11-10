"""SmartHand desktop application UI."""

import math
import json
import os
import subprocess
import sys
import tempfile
import time
from io import BytesIO
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
from PyQt5.QtCore import QEvent, QPoint, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .robot import RobotController
from .utils import MAX_TRANSFORM_DIMENSION, get_local_ip
from .widgets import ClickableLabel, ZoomScrollArea

try:
    import qrcode
except ImportError:  # pragma: no cover - optional dependency
    qrcode = None

ROOT_DIR = Path(__file__).resolve().parents[1]

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
        self.transformed_zoom = 1.0
        self.min_transformed_zoom = 0.05
        self.max_transformed_zoom = 6.0
        self.zoom_slider = None
        self.zoom_value_label = None
        self.transformed_scroll = None
        
        # Chessboard detection
        self.chessboard_size = (7, 7)
        self.chessboard_found = False
        self.corners = None
        self.transformation_matrix = None
        self.transformed_size = (800, 600)
        
        # Phone detection
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

        # Chessboard web server
        self.server_process = None
        self.server_port = 8080
        self.server_local_ip = get_local_ip()
        self.server_base_url = f"http://{self.server_local_ip}:{self.server_port}"
        self.server_status_file = None
        self.server_url_label = None
        self.server_qr_label = None
        self.server_active = False

        # Image crop handling
        self.btn_crop_mode = None
        self.crop_mode = False
        self.crop_start_label_pos = None
        self.crop_rect_image = None
        
        self.init_ui()
        self.start_server()
        
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
        self.transformed_label.setStyleSheet("border: 2px solid #4CAF50; background-color: #222;")
        self.transformed_label.setAlignment(Qt.AlignCenter)
        self.transformed_label.clicked.connect(self.on_transformed_click)
        self.transformed_label.setMouseTracking(True)
        self.transformed_label.installEventFilter(self)
        self.transformed_scroll = ZoomScrollArea()
        self.transformed_scroll.setWidgetResizable(False)
        self.transformed_scroll.setAlignment(Qt.AlignCenter)
        self.transformed_scroll.setStyleSheet("border: none;")
        self.transformed_scroll.setMinimumSize(640, 480)
        self.transformed_scroll.setWidget(self.transformed_label)
        self.transformed_scroll.wheel_zoom.connect(self.on_transformed_wheel)
        transformed_layout.addWidget(self.transformed_scroll)
        zoom_layout = QHBoxLayout()
        zoom_layout.addWidget(QLabel("Zoom:"))
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(
            int(self.min_transformed_zoom * 100),
            int(self.max_transformed_zoom * 100),
        )
        self.zoom_slider.setSingleStep(5)
        self.zoom_slider.setPageStep(25)
        self.zoom_slider.setValue(int(self.transformed_zoom * 100))
        self.zoom_slider.valueChanged.connect(self.on_zoom_slider_changed)
        zoom_layout.addWidget(self.zoom_slider)
        self.zoom_value_label = QLabel(f"{int(self.transformed_zoom * 100)}%")
        zoom_layout.addWidget(self.zoom_value_label)
        zoom_reset_btn = QPushButton("Reset")
        zoom_reset_btn.clicked.connect(self.reset_transformed_zoom)
        zoom_layout.addWidget(zoom_reset_btn)
        zoom_layout.addStretch()
        transformed_layout.addLayout(zoom_layout)

        crop_layout = QHBoxLayout()
        crop_layout.addStretch()
        self.btn_crop_mode = QPushButton("Crop Image")
        self.btn_crop_mode.clicked.connect(self.on_crop_button_clicked)
        crop_layout.addWidget(self.btn_crop_mode)
        transformed_layout.addLayout(crop_layout)
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
        self.server_url_label = QLabel(f"Web server: {self.server_base_url}")
        self.server_url_label.setStyleSheet("color: #00bcd4;")
        self.server_url_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        status_layout.addWidget(self.server_url_label)
        self.server_qr_label = QLabel("QR code unavailable")
        self.server_qr_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.server_qr_label)
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
    
    def detect_chessboard(self, checked=False):
        if self.current_frame is None:
            QMessageBox.warning(self, "Error", "No camera frame available")
            self.log_status("ERROR: No camera frame available for chessboard detection.")
            return False

        self.chessboard_size = (self.chess_cols.value(), self.chess_rows.value())
        gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            self.corners = corners
            self.chessboard_found = True

            self.calculate_transformation_matrix()
            self.update_transformed_view()
            self.auto_center_transformed_view()

            self.chess_status.setText("Status: Chessboard detected!")
            self.chess_status.setStyleSheet("color: green;")
            self.log_status("Chessboard detected and transformation matrix calculated.")
            QMessageBox.information(self, "Success", "Chessboard detected successfully!")
            return True

        self.chessboard_found = False
        self.corners = None
        self.chess_status.setText("Status: Chessboard not found")
        self.chess_status.setStyleSheet("color: red;")
        self.log_status("ERROR: Chessboard not found.")
        QMessageBox.warning(self, "Error", "Chessboard not found.")
        return False
            
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
        base_max = max(chess_width, chess_height)
        scale = max_dimension / base_max if base_max > 0 else 1.0
        chess_output_width = max(1, int(round(chess_width * scale)))
        chess_output_height = max(1, int(round(chess_height * scale)))
        
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
            
            raw_width = float(max_x - min_x)
            raw_height = float(max_y - min_y)
            scale_matrix = np.eye(3, dtype=np.float32)
            if raw_width <= 0 or raw_height <= 0:
                raw_width = float(w)
                raw_height = float(h)
            if raw_width > MAX_TRANSFORM_DIMENSION or raw_height > MAX_TRANSFORM_DIMENSION:
                limit = float(MAX_TRANSFORM_DIMENSION)
                scale_factor = min(limit / raw_width, limit / raw_height)
                scale_matrix = np.array(
                    [
                        [scale_factor, 0, 0],
                        [0, scale_factor, 0],
                        [0, 0, 1],
                    ],
                    dtype=np.float32,
                )
                raw_width *= scale_factor
                raw_height *= scale_factor

            self.transformation_matrix = scale_matrix @ translation_matrix @ M
            output_width = max(1, int(math.ceil(raw_width)))
            output_height = max(1, int(math.ceil(raw_height)))
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
        self.refresh_transformed_pixmap()
    
    def refresh_transformed_pixmap(self):
        """Render the transformed frame with overlays and zoom."""
        if self.transformed_label is None:
            return

        if self.transformed_frame is None:
            self.transformed_label.setPixmap(QPixmap())
            self.transformed_label.setText("Transformed View")
            self.transformed_label.setMinimumSize(0, 0)
            if self.transformed_scroll is not None:
                self.transformed_label.resize(self.transformed_scroll.viewport().size())
            return

        display_frame = self.transformed_frame.copy()

        if self.calibration_points:
            for i, (img_pt, _) in enumerate(self.calibration_points):
                center = (int(img_pt[0]), int(img_pt[1]))
                cv2.circle(display_frame, center, 8, (0, 255, 0), -1)
                cv2.putText(
                    display_frame,
                    f"P{i+1}",
                    (center[0] + 10, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

        if self.temp_click_point:
            cv2.circle(display_frame, self.temp_click_point, 5, (255, 0, 0), -1)
            cv2.drawMarker(
                display_frame,
                self.temp_click_point,
                (255, 0, 0),
                cv2.MARKER_CROSS,
                20,
                2,
            )

        if self.crop_rect_image:
            x1, y1, x2, y2 = self.crop_rect_image
            if x2 > x1 and y2 > y1:
                cv2.rectangle(display_frame, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 2)
                dims_text = f"{x2 - x1}x{y2 - y1}"
                text_origin = (x1, max(0, y1 - 10))
                cv2.putText(
                    display_frame,
                    dims_text,
                    text_origin,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

        rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_image = QImage(
            rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888
        )
        pixmap = QPixmap.fromImage(qt_image)

        zoom = max(self.transformed_zoom, 0.01)
        if abs(zoom - 1.0) > 1e-3:
            scaled_width = max(1, int(pixmap.width() * zoom))
            scaled_height = max(1, int(pixmap.height() * zoom))
            pixmap = pixmap.scaled(
                scaled_width,
                scaled_height,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )

        self.transformed_label.setText("")
        self.transformed_label.setPixmap(pixmap)
        self.transformed_label.setMinimumSize(0, 0)
        self.transformed_label.resize(pixmap.size())
        self.transformed_label.update()

    def auto_center_transformed_view(self, margin_ratio=0.95):
        if self.transformed_scroll is None or self.transformed_frame is None:
            return

        viewport = self.transformed_scroll.viewport()
        if viewport is None:
            return

        frame_height, frame_width = self.transformed_frame.shape[:2]
        viewport_width = viewport.width()
        viewport_height = viewport.height()
        if (
            frame_width <= 0
            or frame_height <= 0
            or viewport_width <= 0
            or viewport_height <= 0
        ):
            return

        fit_zoom = min(viewport_width / frame_width, viewport_height / frame_height)
        if fit_zoom <= 0:
            return

        fit_zoom *= margin_ratio

        current_pixmap = self.transformed_label.pixmap() if self.transformed_label else None
        if current_pixmap and not current_pixmap.isNull():
            focus_pos = QPoint(current_pixmap.width() // 2, current_pixmap.height() // 2)
        else:
            focus_pos = QPoint(int(frame_width / 2), int(frame_height / 2))

        self.set_transformed_zoom(fit_zoom, focus_pos=focus_pos)

        if self.transformed_scroll is not None:
            hbar = self.transformed_scroll.horizontalScrollBar()
            vbar = self.transformed_scroll.verticalScrollBar()
            if hbar is not None:
                hbar.setValue((hbar.maximum() + hbar.minimum()) // 2)
            if vbar is not None:
                vbar.setValue((vbar.maximum() + vbar.minimum()) // 2)

    def set_transformed_zoom(self, zoom_value, focus_pos=None):
        if zoom_value is None:
            return

        zoom_value = float(zoom_value)
        zoom_value = max(self.min_transformed_zoom, min(self.max_transformed_zoom, zoom_value))
        old_zoom = self.transformed_zoom
        if self.transformed_label is None:
            self.transformed_zoom = zoom_value
            return

        pixmap_before = self.transformed_label.pixmap()
        maintain_view = (
            focus_pos is not None
            and self.transformed_frame is not None
            and self.transformed_scroll is not None
            and pixmap_before is not None
            and pixmap_before.width() > 0
            and pixmap_before.height() > 0
            and old_zoom > 0
        )

        if maintain_view:
            focus_x = max(0, min(pixmap_before.width() - 1, focus_pos.x()))
            focus_y = max(0, min(pixmap_before.height() - 1, focus_pos.y()))
            img_width = self.transformed_frame.shape[1]
            img_height = self.transformed_frame.shape[0]
            scale_x = img_width / pixmap_before.width()
            scale_y = img_height / pixmap_before.height()
            image_x = focus_x * scale_x
            image_y = focus_y * scale_y
            viewport = self.transformed_scroll.viewport()
            viewport_pos = self.transformed_label.mapTo(viewport, QPoint(int(focus_x), int(focus_y)))
        else:
            image_x = image_y = 0.0
            viewport_pos = QPoint(0, 0)

        if abs(zoom_value - old_zoom) < 1e-6 and not maintain_view:
            return

        self.transformed_zoom = zoom_value

        slider_percentage = int(round(self.transformed_zoom * 100))
        if self.zoom_slider is not None:
            slider_percentage = max(self.zoom_slider.minimum(), min(self.zoom_slider.maximum(), slider_percentage))
            if self.zoom_slider.value() != slider_percentage:
                self.zoom_slider.blockSignals(True)
                self.zoom_slider.setValue(slider_percentage)
                self.zoom_slider.blockSignals(False)
        if self.zoom_value_label is not None:
            self.zoom_value_label.setText(f"{slider_percentage}%")

        self.refresh_transformed_pixmap()

        if maintain_view:
            new_pixmap = self.transformed_label.pixmap()
            if new_pixmap and new_pixmap.width() > 0 and new_pixmap.height() > 0:
                img_width = self.transformed_frame.shape[1]
                img_height = self.transformed_frame.shape[0]
                new_scale_x = img_width / new_pixmap.width()
                new_scale_y = img_height / new_pixmap.height()
                content_x = image_x / new_scale_x
                content_y = image_y / new_scale_y
                target_x = content_x - viewport_pos.x()
                target_y = content_y - viewport_pos.y()
                hbar = self.transformed_scroll.horizontalScrollBar()
                vbar = self.transformed_scroll.verticalScrollBar()
                if hbar is not None:
                    value = int(round(target_x))
                    value = max(hbar.minimum(), min(hbar.maximum(), value))
                    hbar.setValue(value)
                if vbar is not None:
                    value = int(round(target_y))
                    value = max(vbar.minimum(), min(vbar.maximum(), value))
                    vbar.setValue(value)

    def on_zoom_slider_changed(self, value):
        if value <= 0:
            return

        desired_zoom = value / 100.0
        focus_pos = None
        if (
            self.transformed_scroll is not None
            and self.transformed_label is not None
            and self.transformed_label.pixmap() is not None
        ):
            viewport = self.transformed_scroll.viewport()
            focus_pos = self.transformed_label.mapFrom(viewport, viewport.rect().center())
        self.set_transformed_zoom(desired_zoom, focus_pos=focus_pos)

    def reset_transformed_zoom(self):
        focus_pos = None
        if (
            self.transformed_scroll is not None
            and self.transformed_label is not None
            and self.transformed_label.pixmap() is not None
        ):
            viewport = self.transformed_scroll.viewport()
            focus_pos = self.transformed_label.mapFrom(viewport, viewport.rect().center())
        self.set_transformed_zoom(1.0, focus_pos=focus_pos)

    def on_crop_button_clicked(self):
        if self.crop_mode:
            self.set_crop_mode(False, message="Crop cancelled.")
        else:
            self.set_crop_mode(True)

    def set_crop_mode(self, enabled, message=None):
        if enabled and self.transformed_frame is None:
            QMessageBox.warning(self, "Crop Unavailable", "No transformed image is available to crop.")
            return

        if self.crop_mode == enabled:
            if message:
                self.log_status(message)
            return

        self.crop_mode = bool(enabled)
        if self.btn_crop_mode:
            if self.crop_mode:
                self.btn_crop_mode.setText("Cancel Crop")
                self.btn_crop_mode.setStyleSheet("background-color: #ff9800; color: #000;")
            else:
                self.btn_crop_mode.setText("Crop Image")
                self.btn_crop_mode.setStyleSheet("")

        if self.crop_mode:
            if message is None:
                message = "Crop mode enabled. Drag on the top-down view to select an area."
            self.temp_click_point = None
            self.click_mode = None
            self.crop_start_label_pos = None
            self.crop_rect_image = None
            self.refresh_transformed_pixmap()
            self.log_status(message)
        else:
            self.crop_start_label_pos = None
            self.crop_rect_image = None
            self.refresh_transformed_pixmap()
            if message:
                self.log_status(message)

    def on_transformed_wheel(self, pos, delta):
        if self.transformed_frame is None or delta == 0:
            return

        steps = delta / 120.0
        if steps == 0:
            return

        zoom_factor = math.pow(1.2, steps)
        new_zoom = self.transformed_zoom * zoom_factor
        self.set_transformed_zoom(new_zoom, focus_pos=pos)

    def eventFilter(self, source, event):
        if source is self.transformed_label and self.crop_mode:
            if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                self._handle_crop_press(event.pos())
                event.accept()
                return True
            if event.type() == QEvent.MouseMove and event.buttons() & Qt.LeftButton:
                self._handle_crop_move(event.pos())
                event.accept()
                return True
            if event.type() == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton:
                self._handle_crop_release(event.pos())
                event.accept()
                return True
        return super().eventFilter(source, event)

    def _ensure_qpoint(self, pos):
        if isinstance(pos, QPoint):
            return QPoint(pos.x(), pos.y())
        return QPoint(int(round(pos.x())), int(round(pos.y())))

    def map_label_pos_to_image(self, pos):
        if self.transformed_label is None or self.transformed_frame is None:
            return None

        pixmap = self.transformed_label.pixmap()
        if pixmap is None:
            return None

        width = pixmap.width()
        height = pixmap.height()
        if width <= 0 or height <= 0:
            return None

        point = self._ensure_qpoint(pos)
        x = point.x()
        y = point.y()
        if x < 0 or y < 0 or x >= width or y >= height:
            return None

        scale_x = self.transformed_frame.shape[1] / width
        scale_y = self.transformed_frame.shape[0] / height
        img_x = int(round(x * scale_x))
        img_y = int(round(y * scale_y))
        img_x = max(0, min(self.transformed_frame.shape[1] - 1, img_x))
        img_y = max(0, min(self.transformed_frame.shape[0] - 1, img_y))
        return img_x, img_y

    def _handle_crop_press(self, pos):
        if self.transformed_frame is None:
            return
        point = self._ensure_qpoint(pos)
        self.crop_start_label_pos = point
        start_img = self.map_label_pos_to_image(point)
        if start_img is None:
            self.crop_rect_image = None
        else:
            rect = self._normalize_crop_rect(start_img, start_img)
            self.crop_rect_image = rect
        self.refresh_transformed_pixmap()

    def _handle_crop_move(self, pos):
        if self.crop_start_label_pos is None:
            return
        start_img = self.map_label_pos_to_image(self.crop_start_label_pos)
        current_img = self.map_label_pos_to_image(pos)
        rect = self._normalize_crop_rect(start_img, current_img)
        if rect != self.crop_rect_image:
            self.crop_rect_image = rect
            self.refresh_transformed_pixmap()

    def _handle_crop_release(self, pos):
        if self.crop_start_label_pos is None:
            return
        start_img = self.map_label_pos_to_image(self.crop_start_label_pos)
        end_img = self.map_label_pos_to_image(pos)
        rect = self._normalize_crop_rect(start_img, end_img)
        success = False
        if rect:
            self.crop_rect_image = rect
            self.refresh_transformed_pixmap()
            success = self._finalize_crop(rect)
        self.crop_start_label_pos = None
        if success:
            self.set_crop_mode(False)
        else:
            self.crop_rect_image = None
            self.refresh_transformed_pixmap()

    def _normalize_crop_rect(self, start_img, end_img):
        if start_img is None or end_img is None or self.transformed_frame is None:
            return None
        x1, y1 = start_img
        x2, y2 = end_img
        min_x = max(0, min(x1, x2))
        max_x = min(self.transformed_frame.shape[1], max(x1, x2) + 1)
        min_y = max(0, min(y1, y2))
        max_y = min(self.transformed_frame.shape[0], max(y1, y2) + 1)
        if max_x - min_x < 1 or max_y - min_y < 1:
            return None
        return min_x, min_y, max_x, max_y

    def _finalize_crop(self, rect):
        if self.transformed_frame is None:
            return False
        x1, y1, x2, y2 = rect
        width = x2 - x1
        height = y2 - y1
        if width < 5 or height < 5:
            self.log_status("Crop area too small; please select a larger region.")
            return False
        cropped = self.transformed_frame[y1:y2, x1:x2].copy()
        if cropped.size == 0:
            self.log_status("Selected crop region is empty.")
            return False

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = f"smarthand_crop_{timestamp}.png"
        initial_path = str(Path.home() / default_name)
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Cropped Image",
            initial_path,
            "PNG Image (*.png);;JPEG Image (*.jpg *.jpeg);;BMP Image (*.bmp);;All Files (*)",
        )
        if not file_path:
            self.log_status("Crop save cancelled.")
            return False

        output_path = Path(file_path).expanduser()
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass

        success = cv2.imwrite(str(output_path), cropped)
        if success:
            self.log_status(f"Cropped image saved to {output_path}")
            return True

        QMessageBox.warning(self, "Save Failed", f"Unable to save cropped image to {output_path}")
        self.log_status("Failed to save cropped image.")
        return False
        
        
    def on_camera_click(self, pos):
        """Camera clicks are unused without manual corner selection."""
        return

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
        if self.crop_mode or self.transformed_frame is None:
            return
            
        coords = self.map_label_pos_to_image(pos)
        if coords is None:
            return

        img_x, img_y = coords
        
        self.temp_click_point = (img_x, img_y)
        self.refresh_transformed_pixmap()
        
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

    def start_server(self):
        """Launch the chessboard helper web server."""
        if self.server_process and self.server_process.poll() is None:
            self.server_active = True
            self.update_server_label()
            return

        server_script = ROOT_DIR / "chessboard" / "server.py"
        if not server_script.exists():
            self.log_status(f"Chessboard server script not found: {server_script}")
            self.update_server_label("Web server: {url} (script not found)")
            return

        self.server_local_ip = get_local_ip()
        self.server_base_url = f"http://{self.server_local_ip}:{self.server_port}"
        self.server_active = False
        self.update_server_label("Web server: {url} (starting)")

        self._cleanup_status_file()
        status_fd, status_path = tempfile.mkstemp(prefix="chessboard_server_", suffix=".json")
        os.close(status_fd)
        status_file = Path(status_path)
        try:
            status_file.unlink()
        except FileNotFoundError:
            pass
        self.server_status_file = status_file

        try:
            creationflags = (
                getattr(subprocess, "CREATE_NO_WINDOW", 0)
                if sys.platform.startswith("win")
                else 0
            )
            popen_kwargs = {
                "cwd": str(server_script.parent),
            }
            if creationflags:
                popen_kwargs["creationflags"] = creationflags

            self.server_process = subprocess.Popen(
                [
                    sys.executable,
                    str(server_script),
                    "--port",
                    str(self.server_port),
                    "--status-file",
                    str(status_file),
                ],
                **popen_kwargs,
            )

            status_data = self._wait_for_server_status(status_file, timeout=6.0)
            if not status_data:
                raise RuntimeError("Server did not report its status in time.")

            self._apply_server_status(status_data)
            self.server_active = True
            self.update_server_label()
            self.log_status(f"Chessboard server started at {self.server_base_url}")
        except Exception as exc:
            if self.server_process and self.server_process.poll() is None:
                self.log_status("Stopping failed chessboard server startup...")
                self.server_process.terminate()
                try:
                    self.server_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.server_process.kill()
                    try:
                        self.server_process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pass
            self.server_process = None
            self.server_active = False
            self.log_status(f"Failed to start chessboard server: {exc}")
            self.update_server_label("Web server: {url} (failed to start)")
        finally:
            self._cleanup_status_file()

    def _wait_for_server_status(self, status_file: Path, timeout: float = 5.0):
        """Poll for the JSON status file emitted by the server startup routine."""
        deadline = time.monotonic() + max(timeout, 0.1)
        while time.monotonic() < deadline:
            if status_file.exists():
                try:
                    content = status_file.read_text(encoding="utf-8").strip()
                except OSError:
                    content = ""
                if not content:
                    time.sleep(0.05)
                    continue
                try:
                    return json.loads(content)
                except json.JSONDecodeError:
                    time.sleep(0.05)
                    continue
            if self.server_process and self.server_process.poll() is not None:
                break
            time.sleep(0.05)
        return None

    def _apply_server_status(self, status_data: dict):
        """Update connection details from the server's reported status."""
        local_ip = status_data.get("local_ip")
        if isinstance(local_ip, str) and local_ip:
            self.server_local_ip = local_ip

        port_value = status_data.get("port")
        try:
            if port_value is not None:
                self.server_port = int(port_value)
        except (TypeError, ValueError):
            pass

        url = status_data.get("local_url") or status_data.get("bound_url") or status_data.get("url")
        if isinstance(url, str) and url:
            parsed = urlparse(url)
            if parsed.hostname:
                self.server_local_ip = parsed.hostname
            if parsed.port:
                self.server_port = parsed.port
            self.server_base_url = url
        else:
            self.server_base_url = f"http://{self.server_local_ip}:{self.server_port}"

    def _cleanup_status_file(self):
        """Remove any temporary status file created for server startup."""
        if self.server_status_file:
            try:
                status_path = Path(self.server_status_file)
                if status_path.exists():
                    status_path.unlink()
            except OSError:
                pass
        self.server_status_file = None

    def stop_server(self):
        """Terminate the chessboard helper web server if it is running."""
        process = self.server_process
        if not process:
            self.server_active = False
            self.update_server_label("Web server: {url} (stopped)")
            return

        self.server_process = None
        if process.poll() is None:
            self.log_status("Stopping chessboard server...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            self.log_status("Chessboard server stopped.")

        self.server_active = False
        self.update_server_label("Web server: {url} (stopped)")
        self._cleanup_status_file()

    def update_server_label(self, message=None):
        """Display the server URL in the status panel and refresh QR code."""
        if not self.server_url_label:
            return

        base_url = self.server_base_url or f"http://{self.server_local_ip}:{self.server_port}"
        text = message or f"Web server: {base_url}"
        if "{url}" in text:
            text = text.replace("{url}", base_url)
        self.server_url_label.setText(text)
        self.update_server_qr(base_url, active=self.server_active and qrcode is not None)

    def update_server_qr(self, url, active):
        """Update the QR code preview for the server URL."""
        if not self.server_qr_label:
            return

        if not active:
            if qrcode is None:
                self.server_qr_label.setText("QR code unavailable (install 'qrcode')")
            else:
                self.server_qr_label.setText("Server chưa sẵn sàng")
            self.server_qr_label.setPixmap(QPixmap())
            return

        try:
            qr = qrcode.QRCode(box_size=4, border=1)
            qr.add_data(url)
            qr.make(fit=True)
            img = qr.make_image(fill_color="black", back_color="white").resize((160, 160))
            buffer = BytesIO()
            img.save(buffer, format="PNG")
            qimage = QImage.fromData(buffer.getvalue(), "PNG")
            self.server_qr_label.setPixmap(QPixmap.fromImage(qimage))
            self.server_qr_label.setText("")
        except Exception as exc:
            self.server_qr_label.setPixmap(QPixmap())
            self.server_qr_label.setText(f"QR code error: {exc}")
    
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
        self.stop_server()
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

