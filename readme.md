## SmartHand - Robot Delta Phone Interaction System

### Concept
Control a delta robot to interact with an iPhone application using a stylus to tap the screen, with a camera providing visual feedback of the device.

### Problems To Solve
- Convert phone screen coordinates into robot workspace coordinates
- Detect the phone plane to guarantee safe touch interactions
- Provide tools that replicate a human hand operating the phone

### Solution (SmartHand.py)

**SmartHand.py** delivers a complete user interface with the following capabilities:

#### Camera & Detection Tab
1. **Camera Setup:** Connect and control the camera
2. **Phone Detection:**
   - Method 1: Detect the phone plane with a chessboard pattern
   - Method 2: Manually select the four phone screen corners
3. **Perspective Transform:** Warp the camera view into a top-down phone view

#### Calibration Tab
1. **Coordinate Mapping:** Convert screen coordinates to robot coordinates
   - Select two reference points on the screen
   - Measure the corresponding robot coordinates
   - Automatically compute the transformation matrix
2. **Phone Surface Height:** Set the Z height of the phone surface
3. **Save/Load Calibration:** Persist or restore calibration data

#### Robot Control Tab
1. **Robot Connection:** Connect to the Delta robot via COM port
2. **Position Display:** Show the current robot position (X, Y, Z)
3. **Basic Controls:**
   - Home robot
   - Move to safe height
   - Emergency stop
4. **Manual Jog:** Manually jog along the X, Y, and Z axes

#### Touch Control Tab
1. **Touch Settings:**
   - Touch force
   - Touch duration
   - Movement speed
2. **Click-to-Touch Mode:** Click the transformed view to command a robot touch
3. **Test Touch:** Execute a test touch at a selected point
4. **Gesture Recording:** (Future enhancement) Record and play complex gestures

### Typical Workflow

```
1. Start Camera -> View the camera feed
2. Detect Phone -> Detect via chessboard or select four corners
3. Calibrate Mapping -> Choose reference points and record robot coordinates
4. Set Phone Z Height -> Define the phone surface height
5. Connect Robot -> Establish a serial connection to the Delta robot
6. Test Touch -> Verify a sample touch
7. Operate! -> Control the phone through the robot
```

### Running The System

```
1. Serve a webpage that displays the chessboard:
```bash
cd chessboard
python server.py --host 0.0.0.0 --port 8080
```
The server prints the URL. Example: http://192.168.1.7:8080

2. Open the URL on the phone
![Image](https://firebasestorage.googleapis.com/v0/b/deltax-hub.firebasestorage.app/o/documents%2Fdefault-company%2Fimages%2F1760426526618_bf868361-ca40-4310-92e8-c3a8f1b97599.jpg?alt=media&token=0ae198a8-6225-4709-8fc5-349224b80305)

3. Select the 8x8 chessboard

4. Place the phone under the camera within the robot workspace
![Image](https://firebasestorage.googleapis.com/v0/b/deltax-hub.firebasestorage.app/o/documents%2Fdefault-company%2Fimages%2F1760426644981_04bfa1b2-976c-49d7-be9c-571700dd8ee9.png?alt=media&token=59323a5d-b56e-49a0-b738-3268edc68765)


## Now run Smart Hand software

```bash
cd smartphone
pip install -r camera/requirements.txt
python SmartHand.py
```
![Image](https://firebasestorage.googleapis.com/v0/b/deltax-hub.firebasestorage.app/o/documents%2Fdefault-company%2Fimages%2F1760427411207_d3781a5c-b34f-4f12-a1dc-46bb57d6f4f7.png?alt=media&token=106090e1-6429-469a-b091-39474b878668)

### Safety Features
- Safe Z Height: Robot always travels at a safe altitude between touches
- Visual Feedback: All points are rendered on the transformed view
- Status Logging: Every action is logged in the interface
- Manual Override: Manual jogging is available at any time
