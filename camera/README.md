# Camera Calibration & Coordinate Mapping

This application provides a GUI tool for camera calibration and coordinate mapping from image space to real-world coordinates.

## Features

1. **Camera Capture**: Capture live video feed from webcam
2. **Chessboard Detection**: Detect chessboard patterns and apply perspective transformation to get top-down view
3. **Coordinate Mapping**: Calibrate the system using 2 reference points to map image coordinates to real-world coordinates
4. **Coordinate Conversion**: Convert any image coordinate to real-world coordinate after calibration
5. **Save/Load Calibration**: Save and load calibration data for future use

## Installation

Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Usage

Run the application:

```bash
python mapping.py
```

### Step-by-Step Guide

#### 1. Start Camera
- Select camera ID (usually 0 for default camera)
- Click "Start Camera" button
- The camera feed will appear in the left panel

#### 2. Detect Chessboard (Optional)
- Set the chessboard dimensions (columns and rows - internal corners count)
- Place a chessboard pattern in the camera view
- Click "Detect Chessboard"
- If detected successfully, click "Apply Transformation" to see the top-down view

#### 3. Calibrate Coordinate Mapping
**Important:** Coordinate mapping works on the **transformed (top-down) view**, not the original camera view.

You need 2 reference points to calibrate the mapping:

**For each point:**
- Click "Click to Select" button to enter selection mode
- Click on the **transformed view** (bottom image) at the desired location
- The transformed image coordinates will be automatically filled
- Enter the corresponding real-world coordinates in millimeters

**After entering both points:**
- Click "Calculate Mapping Matrix"
- The system will calculate the transformation parameters

**Note:** You must detect the chessboard and apply transformation before you can select calibration points.

#### 4. Test Coordinate Conversion
- Enter any transformed image coordinates in the test section
- Click "Convert to Real Coordinates"
- The real-world coordinates will be displayed

**Note:** The coordinates you enter should be from the transformed (top-down) view, not the original camera view.

#### 5. Save/Load Calibration
- Click "Save Calibration" to save the current calibration data
- Click "Load Calibration" to load previously saved calibration

## Coordinate System

### Why Use Transformed View for Mapping?

The coordinate mapping is performed on the **transformed (top-down) view** rather than the original camera view for several important reasons:

1. **Orthogonal Projection**: The transformed view provides an orthogonal (perpendicular) view of the workspace, eliminating perspective distortion
2. **Linear Relationships**: Distances and angles are preserved in the top-down view, making the coordinate transformation more accurate
3. **Easier Calibration**: It's much easier to identify reference points and measure real-world coordinates when viewing the workspace from directly above
4. **Practical Usage**: Most robotic applications work in a 2D plane, so a top-down view is the natural coordinate system

### Workflow

```
Original Camera Image → Perspective Transform → Top-Down View → Coordinate Mapping → Real-World Coordinates
```

## Mathematical Background

### Coordinate Mapping

The system uses a 2D similarity transformation (rotation + scale + translation) to map transformed image coordinates to real-world coordinates:

```
[x_real]   [cos(θ)·s  -sin(θ)·s] [x_img]   [tx]
[y_real] = [sin(θ)·s   cos(θ)·s] [y_img] + [ty]
```

Where:
- `s` is the scale factor (mm/pixel)
- `θ` is the rotation angle
- `(tx, ty)` is the translation vector

The parameters are calculated using the two calibration points.

### Perspective Transformation

For chessboard detection, the system uses OpenCV's `getPerspectiveTransform` to compute a 3x3 homography matrix that maps the detected chessboard corners to a rectangular top-down view.

## Requirements

- Python 3.6+
- OpenCV 4.5+
- PyQt5 5.15+
- NumPy 1.19+

## Notes

- Make sure you have good lighting for chessboard detection
- The calibration accuracy depends on the precision of the reference points
- For better accuracy, choose reference points that are far apart
- The coordinate system assumes the origin is at the top-left corner of the image

