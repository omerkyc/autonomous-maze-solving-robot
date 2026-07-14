# Autonomous Maze-Solving Robot

[![Python](https://img.shields.io/badge/Python-3.7%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.8%2B-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A vision-guided robotic system that detects a physical maze from a live camera feed, generates a clearance-aware path, and executes the solution with a Dobot Magician Lite after a one-time camera-to-robot calibration.

- **Computer vision** – detects the maze from a live camera feed and applies perspective correction to handle rotated maze placements.
- **Clearance-aware path planning** – generates a navigation grid using a pixel-resolution distance transform and computes a safe path with BFS.
- **Robot execution** – maps image coordinates into robot coordinates using a one-time homography calibration and traces the solution with a Dobot Magician Lite.

## Demo

[![Watch the demo](https://img.youtube.com/vi/kedj4rLqyjw/maxresdefault.jpg)](https://www.youtube.com/shorts/kedj4rLqyjw)

## Requirements

**Hardware**
- USB camera (default index `1`)
- Dobot Magician Lite (optional, vision-only mode works without it)
- Example serial ports: `/dev/cu.usbmodem101` (macOS), `/dev/ttyUSB0` (Linux)

**Software**
```bash
pip3 install -r requirements.txt
```
Python 3.7+, OpenCV, NumPy, SciPy, PySerial, PyDobot.

## Quick Start

1. **Set up the maze**: rectangular maze with dark walls on a light background, a red dot at one entrance, a green dot at the other.
2. **Calibrate the robot** (one-time, only needed for robot control):
   ```bash
   python3 calibrate_camera.py
   ```
   Click 4 points on the camera feed, then physically move the Dobot to each one and press Enter. The robot's position is recorded automatically and saved to `calibration.json`.
3. **Run the solver**:
   ```bash
   python3 maze_solver.py
   ```
   Choose a direction (Red→Green or Green→Red) when prompted, then press `e` once a path is found to execute it on the robot.

## Controls

| Key | Action |
|-----|--------|
| `q` | Quit |
| `s` | Save the current frame |
| `d` | Toggle debug view (binary wall mask + grid) |
| `e` | Execute the current path with the robot |
| `+` / `-` | Increase/decrease minimum wall clearance (5–100px) |

## How It Works

```
Camera
   ↓
Computer Vision
   ↓
Perspective Correction
   ↓
Distance Transform
   ↓
BFS
   ↓
Homography
   ↓
Robot
```

1. **Maze detection**: adaptive threshold + contours → `cv2.minAreaRect` for a rotated bounding box → perspective transform to straighten the maze regardless of its angle in frame.
2. **Entrance detection**: HSV color matching finds the red/green dots in the straightened image, then maps them back to the original frame.
3. **Grid + clearance**: the straightened maze is thresholded into walls/paths, cleaned with morphology, and run through a distance transform so every cell knows its true pixel-distance from the nearest wall. Only cells clearing the minimum distance count as walkable.
4. **Pathfinding**: BFS on the clearance-filtered grid; if no clearance-safe path exists, it automatically retries on the unfiltered grid and flags the result `[No Clearance]`.
5. **Robot execution**: each grid point is converted grid → warped frame → original frame → robot mm (via the calibration transform), then queued as a continuous, non-blocking motion so the arm traces the path smoothly.

## Troubleshooting

- **No maze detected**: needs good lighting and clear dark walls on a light background; the maze should be the largest object in frame.
- **No path found**: press `d` for the debug view to check wall detection, or `-` to relax the clearance requirement.
- **Start/End on a wall**: the app auto-snaps to the nearest walkable cell within 20 grid cells; if that fails, move the dot closer to the opening.
- **Robot won't connect**: check the serial port matches your OS/device and that nothing else has the port open.

## License

MIT. See [LICENSE](LICENSE).
