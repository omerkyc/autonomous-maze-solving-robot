# Autonomous Maze-Solving Robot

[![Python](https://img.shields.io/badge/Python-3.7%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.8%2B-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A computer-vision pipeline that watches a physical maze through a webcam, solves it, and drives a **Dobot Magician Lite** robot arm to trace the solution live — at any camera angle, with no manual setup beyond a one-time calibration.

- **Works at any rotation** — detects the maze's rotated bounding box and perspective-warps it flat before doing anything else
- **Wall-clearance aware pathfinding** — uses a pixel-resolution distance transform (not just a coarse grid) so the path stays a configurable distance from walls, with automatic fallback if no clear path exists
- **Real robot execution** — calibrates camera pixels to robot millimeters once, then drives the arm through the solved path with continuous, non-stop motion, complete with automatic recovery if a move fails mid-path

## Demo

[![Watch the demo](https://img.youtube.com/vi/kedj4rLqyjw/maxresdefault.jpg)](https://www.youtube.com/shorts/kedj4rLqyjw)

## Requirements

**Hardware**
- USB camera (default index `1`)
- Dobot Magician Lite (optional — vision-only mode works without it)
- Serial connection: `/dev/cu.usbmodem101` (macOS) or `/dev/ttyUSB0` (Linux)

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
   Click 4 points on the camera feed, then physically move the Dobot to each one and press Enter — the robot's position is recorded automatically and saved to `calibration.json`.
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

1. **Maze detection** — adaptive threshold + contours → `cv2.minAreaRect` for a rotated bounding box → perspective transform to straighten the maze regardless of its angle in frame.
2. **Entrance detection** — HSV color matching finds the red/green dots in the straightened image, then maps them back to the original frame.
3. **Grid + clearance** — the straightened maze is thresholded into walls/paths, cleaned with morphology, and run through a distance transform so every cell knows its true pixel-distance from the nearest wall. Only cells clearing the minimum distance count as walkable.
4. **Pathfinding** — BFS on the clearance-filtered grid; if no path exists (common on tight or tilted mazes), it automatically retries on the unfiltered grid and flags the result `[No Clearance]`.
5. **Robot execution** — each grid point is converted grid → warped frame → original frame → robot mm (via the calibration transform), then queued as a continuous, non-blocking motion so the arm traces the path smoothly.

## Troubleshooting

- **No maze detected** — needs good lighting and clear dark walls on a light background; the maze should be the largest object in frame.
- **No path found** — press `d` for the debug view to check wall detection, or `-` to relax the clearance requirement.
- **Start/End on a wall** — the app auto-snaps to the nearest walkable cell within 20 grid cells; if that fails, move the dot closer to the opening.
- **Robot won't connect** — check the serial port matches your OS/device and that nothing else has the port open.

## License

MIT — see [LICENSE](LICENSE).
