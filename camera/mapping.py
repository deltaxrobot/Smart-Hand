'''
    - Có giao diện QT.
    - Thu hình ảnh từ camera (webcam)
    - Nhận diện chessboard và biến đổi ảnh từ góc nhìn phối cảnh sang góc nhìn trực diện
    - Có chức năng mapping tọa độ từ ảnh sang tọa độ thực tế
    + Nhập thủ công tọa độ 2 điểm để tính toán ma trận biến đổi
    + Có hàm tính tọa độ thực tế từ tọa độ ảnh

'''

import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QLineEdit, 
                            QGroupBox, QGridLayout, QSpinBox, QMessageBox,
                            QFileDialog, QComboBox, QCheckBox)
from PyQt5.QtCore import QTimer, Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen


class CameraCalibrationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Calibration & Coordinate Mapping")
        self.setGeometry(100, 100, 1400, 800)
        
        # Camera properties
        self.camera = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.current_frame = None
        self.transformed_frame = None
        
        # Chessboard detection
        self.chessboard_size = (9, 6)  # Default chessboard size
        self.chessboard_found = False
        self.corners = None
        self.transformation_matrix = None
        self.transformed_size = (800, 600)
        self.transform_full_image = True  # Transform entire image, not just chessboard
        
        # Coordinate mapping
        self.calibration_points = []  # List of (image_point, real_point)
        self.mapping_matrix = None
        self.click_mode = None  # 'point1', 'point2', or 'test'
        self.temp_click_point = None
        self.temp_test_point = None  # For test mode visualization
        
        self.init_ui()
        
    def init_ui(self):
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        
        # Left panel - Camera and transformed view
        left_panel = QVBoxLayout()
        
        # Camera view
        self.camera_label = QLabel("Camera View")
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid black; background-color: #333;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        left_panel.addWidget(QLabel("<b>Original Camera View</b>"))
        left_panel.addWidget(self.camera_label)
        
        # Transformed view (clickable for coordinate mapping)
        self.transformed_label = ClickableLabel("Transformed View")
        self.transformed_label.setFixedSize(640, 480)
        self.transformed_label.setStyleSheet("border: 2px solid black; background-color: #333;")
        self.transformed_label.setAlignment(Qt.AlignCenter)
        self.transformed_label.clicked.connect(self.on_transformed_click)
        left_panel.addWidget(QLabel("<b>Transformed View (Top-down) - Click here for mapping</b>"))
        left_panel.addWidget(self.transformed_label)
        
        main_layout.addLayout(left_panel)
        
        # Right panel - Controls
        right_panel = QVBoxLayout()
        
        # Camera controls
        camera_group = QGroupBox("Camera Controls")
        camera_layout = QVBoxLayout()
        
        # Camera selection
        cam_select_layout = QHBoxLayout()
        cam_select_layout.addWidget(QLabel("Camera ID:"))
        self.camera_id_spin = QSpinBox()
        self.camera_id_spin.setMinimum(0)
        self.camera_id_spin.setMaximum(10)
        self.camera_id_spin.setValue(0)
        cam_select_layout.addWidget(self.camera_id_spin)
        camera_layout.addLayout(cam_select_layout)
        
        self.btn_start_camera = QPushButton("Start Camera")
        self.btn_start_camera.clicked.connect(self.start_camera)
        camera_layout.addWidget(self.btn_start_camera)
        
        self.btn_stop_camera = QPushButton("Stop Camera")
        self.btn_stop_camera.clicked.connect(self.stop_camera)
        self.btn_stop_camera.setEnabled(False)
        camera_layout.addWidget(self.btn_stop_camera)
        
        camera_group.setLayout(camera_layout)
        right_panel.addWidget(camera_group)
        
        # Chessboard detection controls
        chess_group = QGroupBox("Chessboard Detection")
        chess_layout = QVBoxLayout()
        
        # Chessboard size
        size_layout = QHBoxLayout()
        size_layout.addWidget(QLabel("Columns:"))
        self.chess_cols = QSpinBox()
        self.chess_cols.setMinimum(3)
        self.chess_cols.setMaximum(20)
        self.chess_cols.setValue(9)
        size_layout.addWidget(self.chess_cols)
        size_layout.addWidget(QLabel("Rows:"))
        self.chess_rows = QSpinBox()
        self.chess_rows.setMinimum(3)
        self.chess_rows.setMaximum(20)
        self.chess_rows.setValue(6)
        size_layout.addWidget(self.chess_rows)
        chess_layout.addLayout(size_layout)
        
        self.btn_detect = QPushButton("Detect Chessboard")
        self.btn_detect.clicked.connect(self.detect_chessboard)
        chess_layout.addWidget(self.btn_detect)
        
        self.chk_full_image = QCheckBox("Transform Full Image (no crop)")
        self.chk_full_image.setChecked(True)
        self.chk_full_image.stateChanged.connect(self.on_full_image_changed)
        chess_layout.addWidget(self.chk_full_image)
        
        self.btn_transform = QPushButton("Apply Transformation")
        self.btn_transform.clicked.connect(self.apply_transformation)
        self.btn_transform.setEnabled(False)
        chess_layout.addWidget(self.btn_transform)
        
        self.chess_status = QLabel("Status: Not detected")
        chess_layout.addWidget(self.chess_status)
        
        chess_group.setLayout(chess_layout)
        right_panel.addWidget(chess_group)
        
        # Coordinate mapping controls
        mapping_group = QGroupBox("Coordinate Mapping Calibration")
        mapping_layout = QVBoxLayout()
        
        mapping_layout.addWidget(QLabel("<i>Note: Click on the transformed view (top-down image)</i>"))
        mapping_layout.addWidget(QLabel("<b>Point 1</b>"))
        
        # Point 1 - Image coordinates
        p1_img_layout = QHBoxLayout()
        p1_img_layout.addWidget(QLabel("Transformed (x,y):"))
        self.p1_img_x = QLineEdit()
        self.p1_img_x.setPlaceholderText("x")
        p1_img_layout.addWidget(self.p1_img_x)
        self.p1_img_y = QLineEdit()
        self.p1_img_y.setPlaceholderText("y")
        p1_img_layout.addWidget(self.p1_img_y)
        self.btn_click_p1 = QPushButton("Click to Select")
        self.btn_click_p1.clicked.connect(self.enable_point1_selection)
        p1_img_layout.addWidget(self.btn_click_p1)
        mapping_layout.addLayout(p1_img_layout)
        
        # Point 1 - Real coordinates
        p1_real_layout = QHBoxLayout()
        p1_real_layout.addWidget(QLabel("Real (x,y):"))
        self.p1_real_x = QLineEdit()
        self.p1_real_x.setPlaceholderText("x (mm)")
        p1_real_layout.addWidget(self.p1_real_x)
        self.p1_real_y = QLineEdit()
        self.p1_real_y.setPlaceholderText("y (mm)")
        p1_real_layout.addWidget(self.p1_real_y)
        mapping_layout.addLayout(p1_real_layout)
        
        mapping_layout.addWidget(QLabel("<b>Point 2</b>"))
        
        # Point 2 - Image coordinates
        p2_img_layout = QHBoxLayout()
        p2_img_layout.addWidget(QLabel("Transformed (x,y):"))
        self.p2_img_x = QLineEdit()
        self.p2_img_x.setPlaceholderText("x")
        p2_img_layout.addWidget(self.p2_img_x)
        self.p2_img_y = QLineEdit()
        self.p2_img_y.setPlaceholderText("y")
        p2_img_layout.addWidget(self.p2_img_y)
        self.btn_click_p2 = QPushButton("Click to Select")
        self.btn_click_p2.clicked.connect(self.enable_point2_selection)
        p2_img_layout.addWidget(self.btn_click_p2)
        mapping_layout.addLayout(p2_img_layout)
        
        # Point 2 - Real coordinates
        p2_real_layout = QHBoxLayout()
        p2_real_layout.addWidget(QLabel("Real (x,y):"))
        self.p2_real_x = QLineEdit()
        self.p2_real_x.setPlaceholderText("x (mm)")
        p2_real_layout.addWidget(self.p2_real_x)
        self.p2_real_y = QLineEdit()
        self.p2_real_y.setPlaceholderText("y (mm)")
        p2_real_layout.addWidget(self.p2_real_y)
        mapping_layout.addLayout(p2_real_layout)
        
        self.btn_calibrate = QPushButton("Calculate Mapping Matrix")
        self.btn_calibrate.clicked.connect(self.calibrate_mapping)
        mapping_layout.addWidget(self.btn_calibrate)
        
        self.mapping_status = QLabel("Status: Not calibrated")
        mapping_layout.addWidget(self.mapping_status)
        
        mapping_group.setLayout(mapping_layout)
        right_panel.addWidget(mapping_group)
        
        # Test coordinate conversion
        test_group = QGroupBox("Test Coordinate Conversion")
        test_layout = QVBoxLayout()
        
        test_input_layout = QHBoxLayout()
        test_input_layout.addWidget(QLabel("Transformed (x,y):"))
        self.test_img_x = QLineEdit()
        self.test_img_x.setPlaceholderText("x")
        test_input_layout.addWidget(self.test_img_x)
        self.test_img_y = QLineEdit()
        self.test_img_y.setPlaceholderText("y")
        test_input_layout.addWidget(self.test_img_y)
        self.btn_click_test = QPushButton("Click to Select")
        self.btn_click_test.clicked.connect(self.enable_test_selection)
        test_input_layout.addWidget(self.btn_click_test)
        test_layout.addLayout(test_input_layout)
        
        self.btn_convert = QPushButton("Convert to Real Coordinates")
        self.btn_convert.clicked.connect(self.test_conversion)
        test_layout.addWidget(self.btn_convert)
        
        self.result_label = QLabel("Real coordinates: -")
        self.result_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        test_layout.addWidget(self.result_label)
        
        test_group.setLayout(test_layout)
        right_panel.addWidget(test_group)
        
        # Save/Load calibration
        file_group = QGroupBox("Save/Load Calibration")
        file_layout = QHBoxLayout()
        
        self.btn_save = QPushButton("Save Calibration")
        self.btn_save.clicked.connect(self.save_calibration)
        file_layout.addWidget(self.btn_save)
        
        self.btn_load = QPushButton("Load Calibration")
        self.btn_load.clicked.connect(self.load_calibration)
        file_layout.addWidget(self.btn_load)
        
        file_group.setLayout(file_layout)
        right_panel.addWidget(file_group)
        
        right_panel.addStretch()
        main_layout.addLayout(right_panel)
        
    def start_camera(self):
        camera_id = self.camera_id_spin.value()
        self.camera = cv2.VideoCapture(camera_id)
        if self.camera.isOpened():
            self.timer.start(30)  # 30ms refresh rate
            self.btn_start_camera.setEnabled(False)
            self.btn_stop_camera.setEnabled(True)
            QMessageBox.information(self, "Success", f"Camera {camera_id} started successfully!")
        else:
            QMessageBox.warning(self, "Error", f"Cannot open camera {camera_id}")
            
    def stop_camera(self):
        if self.camera:
            self.timer.stop()
            self.camera.release()
            self.camera = None
            self.btn_start_camera.setEnabled(True)
            self.btn_stop_camera.setEnabled(False)
            
    def update_frame(self):
        if self.camera and self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                self.current_frame = frame.copy()
                
                # Convert to QPixmap and display (no annotations on camera view)
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                    self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.camera_label.setPixmap(scaled_pixmap)
                
                # Update transformed view if transformation is available
                if self.transformation_matrix is not None:
                    self.update_transformed_view()
                    
    def on_full_image_changed(self, state):
        self.transform_full_image = (state == Qt.Checked)
        # Recalculate transformation matrix if chessboard was already detected
        if self.chessboard_found:
            self.calculate_transformation_matrix()
            self.update_transformed_view()
    
    def detect_chessboard(self):
        if self.current_frame is None:
            QMessageBox.warning(self, "Error", "No camera frame available")
            return
            
        self.chessboard_size = (self.chess_cols.value(), self.chess_rows.value())
        gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            self.corners = corners
            self.chessboard_found = True
            
            # Calculate transformation matrix
            self.calculate_transformation_matrix()
            
            self.chess_status.setText("Status: Chessboard detected!")
            self.chess_status.setStyleSheet("color: green;")
            self.btn_transform.setEnabled(True)
            QMessageBox.information(self, "Success", "Chessboard detected successfully!")
        else:
            self.chessboard_found = False
            self.chess_status.setText("Status: Chessboard not found")
            self.chess_status.setStyleSheet("color: red;")
            QMessageBox.warning(self, "Error", "Chessboard not found. Try adjusting the size or camera position.")
            
    def calculate_transformation_matrix(self):
        if not self.chessboard_found or self.corners is None or self.current_frame is None:
            return
            
        # Get the four corner points of the chessboard
        top_left = self.corners[0][0]
        top_right = self.corners[self.chessboard_size[0] - 1][0]
        bottom_left = self.corners[-self.chessboard_size[0]][0]
        bottom_right = self.corners[-1][0]
        
        chessboard_src = np.float32([top_left, top_right, bottom_left, bottom_right])
        
        # Calculate the output size based on the chessboard dimensions
        # Use the distances between corners to determine the aspect ratio
        width_top = np.linalg.norm(top_right - top_left)
        width_bottom = np.linalg.norm(bottom_right - bottom_left)
        chess_width = max(width_top, width_bottom)
        
        height_left = np.linalg.norm(bottom_left - top_left)
        height_right = np.linalg.norm(bottom_right - top_right)
        chess_height = max(height_left, height_right)
        
        # Scale to a reasonable size while maintaining aspect ratio
        max_dimension = 800
        scale = max_dimension / max(chess_width, chess_height)
        chess_output_width = int(chess_width * scale)
        chess_output_height = int(chess_height * scale)
        
        # Define destination points for chessboard (rectangular view)
        chessboard_dst = np.float32([
            [0, 0], 
            [chess_output_width, 0], 
            [0, chess_output_height], 
            [chess_output_width, chess_output_height]
        ])
        
        # Calculate base perspective transformation matrix
        M = cv2.getPerspectiveTransform(chessboard_src, chessboard_dst)
        
        if self.transform_full_image:
            # Transform full image without cropping
            h, w = self.current_frame.shape[:2]
            
            # Get corners of the full image
            image_corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
            
            # Transform the image corners using the calculated matrix
            transformed_corners = cv2.perspectiveTransform(image_corners, M)
            
            # Find bounding box of transformed image
            x_coords = transformed_corners[:, 0, 0]
            y_coords = transformed_corners[:, 0, 1]
            
            min_x, max_x = x_coords.min(), x_coords.max()
            min_y, max_y = y_coords.min(), y_coords.max()
            
            # Calculate translation to bring all content into view
            translation_matrix = np.array([
                [1, 0, -min_x],
                [0, 1, -min_y],
                [0, 0, 1]
            ], dtype=np.float32)
            
            # Combine transformation: first apply M, then translate
            self.transformation_matrix = translation_matrix @ M
            
            # Store the calculated size (size of the bounding box)
            output_width = int(np.ceil(max_x - min_x))
            output_height = int(np.ceil(max_y - min_y))
            self.transformed_size = (output_width, output_height)
        else:
            # Only transform chessboard region
            self.transformation_matrix = M
            self.transformed_size = (chess_output_width, chess_output_height)
        
    def apply_transformation(self):
        if self.transformation_matrix is None:
            QMessageBox.warning(self, "Error", "No transformation matrix available")
            return
            
        self.update_transformed_view()
        QMessageBox.information(self, "Success", "Transformation applied!")
        
    def update_transformed_view(self):
        if self.current_frame is None or self.transformation_matrix is None:
            return
            
        # Apply perspective transformation
        self.transformed_frame = cv2.warpPerspective(
            self.current_frame, 
            self.transformation_matrix, 
            self.transformed_size
        )
        
        # Create display frame with annotations
        display_frame = self.transformed_frame.copy()
        
        # Draw calibration points on transformed view
        if self.calibration_points:
            for i, (img_pt, real_pt) in enumerate(self.calibration_points):
                cv2.circle(display_frame, (int(img_pt[0]), int(img_pt[1])), 8, (0, 255, 0), -1)
                cv2.putText(display_frame, f"P{i+1}", 
                           (int(img_pt[0]) + 10, int(img_pt[1]) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw temporary click point for calibration
        if self.temp_click_point:
            cv2.circle(display_frame, self.temp_click_point, 5, (255, 0, 0), -1)
            cv2.drawMarker(display_frame, self.temp_click_point, (255, 0, 0), 
                          cv2.MARKER_CROSS, 20, 2)
        
        # Draw test point with different color and show real coordinates if available
        if self.temp_test_point:
            cv2.circle(display_frame, self.temp_test_point, 7, (0, 165, 255), -1)  # Orange
            cv2.drawMarker(display_frame, self.temp_test_point, (0, 165, 255), 
                          cv2.MARKER_STAR, 25, 2)
            
            # Show real-world coordinates on the image if mapping is available
            if self.mapping_matrix is not None:
                try:
                    real_x, real_y = self.image_to_real_coordinates(
                        self.temp_test_point[0], self.temp_test_point[1])
                    coord_text = f"({real_x:.1f}, {real_y:.1f}mm)"
                    cv2.putText(display_frame, coord_text, 
                               (int(self.temp_test_point[0]) + 15, int(self.temp_test_point[1]) - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    cv2.putText(display_frame, coord_text, 
                               (int(self.temp_test_point[0]) + 15, int(self.temp_test_point[1]) - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                except:
                    pass
        
        # Convert to QPixmap and display
        rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
            self.transformed_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.transformed_label.setPixmap(scaled_pixmap)
        
    def enable_point1_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", 
                "Please detect chessboard and apply transformation first!")
            return
            
        self.click_mode = 'point1'
        self.btn_click_p1.setText("Clicking... (Point 1)")
        self.btn_click_p1.setStyleSheet("background-color: yellow;")
        self.btn_click_p2.setEnabled(False)
        self.btn_click_test.setEnabled(False)
        
    def enable_point2_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", 
                "Please detect chessboard and apply transformation first!")
            return
            
        self.click_mode = 'point2'
        self.btn_click_p2.setText("Clicking... (Point 2)")
        self.btn_click_p2.setStyleSheet("background-color: yellow;")
        self.btn_click_p1.setEnabled(False)
        self.btn_click_test.setEnabled(False)
        
    def enable_test_selection(self):
        if self.transformed_frame is None:
            QMessageBox.warning(self, "Error", 
                "Please detect chessboard and apply transformation first!")
            return
            
        self.click_mode = 'test'
        self.btn_click_test.setText("Clicking... (Test)")
        self.btn_click_test.setStyleSheet("background-color: yellow;")
        self.btn_click_p1.setEnabled(False)
        self.btn_click_p2.setEnabled(False)
        
    def on_transformed_click(self, pos):
        if self.click_mode is None or self.transformed_frame is None:
            return
            
        # Convert click position to transformed image coordinates
        label_size = self.transformed_label.size()
        pixmap = self.transformed_label.pixmap()
        if pixmap is None:
            return
            
        pixmap_size = pixmap.size()
        
        # Calculate offset due to KeepAspectRatio
        x_offset = (label_size.width() - pixmap_size.width()) / 2
        y_offset = (label_size.height() - pixmap_size.height()) / 2
        
        # Adjust click position
        click_x = pos.x() - x_offset
        click_y = pos.y() - y_offset
        
        if click_x < 0 or click_y < 0 or click_x >= pixmap_size.width() or click_y >= pixmap_size.height():
            return
            
        # Scale to transformed image coordinates
        scale_x = self.transformed_frame.shape[1] / pixmap_size.width()
        scale_y = self.transformed_frame.shape[0] / pixmap_size.height()
        
        img_x = int(click_x * scale_x)
        img_y = int(click_y * scale_y)
        
        if self.click_mode == 'point1':
            self.temp_click_point = (img_x, img_y)
            self.p1_img_x.setText(str(img_x))
            self.p1_img_y.setText(str(img_y))
            self.btn_click_p1.setText("Click to Select")
            self.btn_click_p1.setStyleSheet("")
            self.btn_click_p2.setEnabled(True)
            self.btn_click_test.setEnabled(True)
        elif self.click_mode == 'point2':
            self.temp_click_point = (img_x, img_y)
            self.p2_img_x.setText(str(img_x))
            self.p2_img_y.setText(str(img_y))
            self.btn_click_p2.setText("Click to Select")
            self.btn_click_p2.setStyleSheet("")
            self.btn_click_p1.setEnabled(True)
            self.btn_click_test.setEnabled(True)
        elif self.click_mode == 'test':
            self.temp_test_point = (img_x, img_y)
            self.test_img_x.setText(str(img_x))
            self.test_img_y.setText(str(img_y))
            self.btn_click_test.setText("Click to Select")
            self.btn_click_test.setStyleSheet("")
            self.btn_click_p1.setEnabled(True)
            self.btn_click_p2.setEnabled(True)
            
            # Auto convert if mapping is calibrated
            if self.mapping_matrix is not None:
                self.test_conversion()
            
        self.click_mode = None
        
    def calibrate_mapping(self):
        try:
            # Get point 1
            p1_img_x = float(self.p1_img_x.text())
            p1_img_y = float(self.p1_img_y.text())
            p1_real_x = float(self.p1_real_x.text())
            p1_real_y = float(self.p1_real_y.text())
            
            # Get point 2
            p2_img_x = float(self.p2_img_x.text())
            p2_img_y = float(self.p2_img_y.text())
            p2_real_x = float(self.p2_real_x.text())
            p2_real_y = float(self.p2_real_y.text())
            
            # Store calibration points
            self.calibration_points = [
                ((p1_img_x, p1_img_y), (p1_real_x, p1_real_y)),
                ((p2_img_x, p2_img_y), (p2_real_x, p2_real_y))
            ]
            
            # Calculate scale and offset
            # Using two points to create a linear transformation
            img_vec = np.array([p2_img_x - p1_img_x, p2_img_y - p1_img_y])
            real_vec = np.array([p2_real_x - p1_real_x, p2_real_y - p1_real_y])
            
            # Calculate scale factor
            img_dist = np.linalg.norm(img_vec)
            real_dist = np.linalg.norm(real_vec)
            
            if img_dist == 0:
                QMessageBox.warning(self, "Error", "Points are too close together")
                return
                
            scale = real_dist / img_dist
            
            # Calculate angle difference
            img_angle = np.arctan2(img_vec[1], img_vec[0])
            real_angle = np.arctan2(real_vec[1], real_vec[0])
            rotation_angle = real_angle - img_angle
            
            # Create transformation matrix (rotation + scale)
            cos_a = np.cos(rotation_angle) * scale
            sin_a = np.sin(rotation_angle) * scale
            
            # Calculate translation
            # real = R * img + t
            # t = real - R * img
            rot_scale_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
            p1_img_vec = np.array([p1_img_x, p1_img_y])
            p1_real_vec = np.array([p1_real_x, p1_real_y])
            translation = p1_real_vec - rot_scale_matrix @ p1_img_vec
            
            # Store the mapping matrix as [R | t]
            self.mapping_matrix = {
                'rotation_scale': rot_scale_matrix,
                'translation': translation
            }
            
            self.mapping_status.setText("Status: Calibrated successfully!")
            self.mapping_status.setStyleSheet("color: green;")
            QMessageBox.information(self, "Success", 
                f"Mapping calibrated!\nScale: {scale:.4f} mm/pixel\nRotation: {np.degrees(rotation_angle):.2f}°")
            
        except ValueError:
            QMessageBox.warning(self, "Error", "Please enter valid numbers for all coordinates")
            
    def image_to_real_coordinates(self, img_x, img_y):
        """
        Convert transformed image coordinates to real-world coordinates.
        
        Args:
            img_x, img_y: Coordinates on the transformed (top-down) image
            
        Returns:
            (real_x, real_y): Real-world coordinates in mm
        """
        if self.mapping_matrix is None:
            return None
            
        img_point = np.array([img_x, img_y])
        real_point = self.mapping_matrix['rotation_scale'] @ img_point + self.mapping_matrix['translation']
        
        return real_point[0], real_point[1]
        
    def test_conversion(self):
        if self.mapping_matrix is None:
            QMessageBox.warning(self, "Error", "Please calibrate mapping first")
            return
            
        try:
            img_x = float(self.test_img_x.text())
            img_y = float(self.test_img_y.text())
            
            real_x, real_y = self.image_to_real_coordinates(img_x, img_y)
            
            self.result_label.setText(f"Real coordinates: ({real_x:.2f}, {real_y:.2f}) mm")
            
        except ValueError:
            QMessageBox.warning(self, "Error", "Please enter valid numbers")
            
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
                    transformation_matrix=self.transformation_matrix if self.transformation_matrix is not None else np.array([]))
            QMessageBox.information(self, "Success", "Calibration saved successfully!")
            
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
                    # Update UI with loaded points
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
                
            self.mapping_status.setText("Status: Calibration loaded!")
            self.mapping_status.setStyleSheet("color: green;")
            QMessageBox.information(self, "Success", "Calibration loaded successfully!")
            
    def closeEvent(self, event):
        self.stop_camera()
        event.accept()


class ClickableLabel(QLabel):
    """Custom QLabel that emits click signals"""
    clicked = pyqtSignal(QPoint)
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(event.pos())


def main():
    app = QApplication(sys.argv)
    window = CameraCalibrationApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
