import cv2
import numpy as np
from collections import deque
import time
import json
import os

class MazeSolver:
    def __init__(self, camera_index=1, robot_enabled=False, robot_port='/dev/cu.usbmodem101'):
        self.camera_index = camera_index
        self.cap = None
        self.grid_size = 2  # Size of grid cells in pixels - denser grid for accurate clearance
        self.min_clearance_pixels = 23  # Minimum distance from walls in PIXELS (adjustable 5-100)
        self.debug_binary = None  # For debugging wall detection
        self.debug_distance = None  # For debugging distance transform
        self.base_grid = None  # Original grid without clearance filtering (for fallback)
        self.perspective_matrix = None  # Transform matrix for rotated mazes
        self.inverse_perspective = None  # Inverse transform to map coordinates back
        
        # Robot control parameters
        self.robot_enabled = robot_enabled
        self.robot_port = robot_port
        self.robot = None
        self.transform_matrix = None
        self.direction = 'red_to_green'  # Default direction
        
        # Robot Z heights
        self.SAFE_Z = 150.0  # Safe height for movement
        self.DRAW_Z = -40.0  # Drawing height
        
        # Load calibration if robot is enabled
        if self.robot_enabled:
            self.load_calibration()

    def load_calibration(self, filename='calibration.json'):
        """Load calibration data from file"""
        if not os.path.exists(filename):
            print(f"\n⚠ WARNING: Calibration file '{filename}' not found!")
            print("Please run 'python3 calibrate_camera.py' first to create calibration.")
            self.robot_enabled = False
            return
        
        try:
            with open(filename, 'r') as f:
                calib_data = json.load(f)
            
            self.transform_matrix = np.array(calib_data['transform_matrix'], dtype=np.float32)
            print(f"✓ Calibration loaded from {filename}")
            print(f"  Transform matrix shape: {self.transform_matrix.shape}")
        except Exception as e:
            print(f"\n⚠ Error loading calibration: {e}")
            print(f"  Error type: {type(e).__name__}")
            self.robot_enabled = False
    
    def pixel_to_robot(self, pixel_x, pixel_y):
        """Transform pixel coordinates to robot coordinates"""
        if self.transform_matrix is None:
            return None
        
        # Apply perspective transformation
        pixel_pt = np.array([[pixel_x, pixel_y, 1]], dtype=np.float32).T
        robot_pt = self.transform_matrix @ pixel_pt
        robot_pt = robot_pt[:2] / robot_pt[2]
        
        return (float(robot_pt[0, 0]), float(robot_pt[1, 0]))
    
    def connect_robot(self):
        """Connect to the Dobot robot"""
        if not self.robot_enabled:
            return False
        
        try:
            from pydobot import Dobot
            
            print(f"\nConnecting to Dobot at {self.robot_port}...")
            self.robot = Dobot(port=self.robot_port)
            print("✓ Robot connected successfully!")
            
            # Get current position
            pos = self.robot.pose()
            print(f"  Current position: X={pos[0]:.2f}mm, Y={pos[1]:.2f}mm, Z={pos[2]:.2f}mm")
            
            # Move to viewing position
            print(f"\nMoving to viewing position (X=240mm, Y=0mm, Z=150mm)...")
            self.robot.move_to(240, 0, 150, 0, wait=True)
            print("✓ Moved to viewing position")
            print("✓ Robot ready!")
            
            return True
        
        except Exception as e:
            print(f"\n⚠ Error connecting to robot: {e}")
            print("Continuing without robot control...")
            self.robot_enabled = False
            return False
    
    def disconnect_robot(self):
        """Disconnect from the robot"""
        if self.robot:
            try:
                # Return to viewing position
                print("\nReturning to viewing position...")
                self.robot.move_to(240, 0, 150, 0, wait=True)
                self.robot.close()
                print("✓ Robot disconnected")
            except Exception as e:
                print(f"⚠ Error disconnecting robot: {e}")
    
    def execute_path(self, path, bbox):
        """Execute the maze solution path with the robot"""
        # Validate robot state
        if not self.robot_enabled:
            print("⚠ Robot control is not enabled")
            return
        
        if self.robot is None:
            print("⚠ Robot is not connected")
            return
        
        if path is None or bbox is None:
            print("⚠ No valid path to execute")
            return
        
        print("\n" + "="*60)
        print("EXECUTING PATH WITH ROBOT")
        print("="*60)
        
        print(f"\nConverting {len(path)} grid points to world coordinates...")
        
        # Convert path to world coordinates
        world_path = []
        for i, grid_pos in enumerate(path):
            world_pos = self.grid_to_world(grid_pos, bbox)
            if world_pos:
                world_path.append(world_pos)
            else:
                print(f"⚠ Warning: Failed to convert grid point {i}: {grid_pos}")
        
        if not world_path:
            print("⚠ No valid path coordinates")
            print(f"   Path had {len(path)} points but none converted successfully")
            print(f"   Bbox: {bbox}")
            print(f"   Perspective matrix exists: {self.perspective_matrix is not None}")
            print(f"   Inverse perspective exists: {self.inverse_perspective is not None}")
            return
        
        print(f"✓ Converted to {len(world_path)} world coordinates")
        
        # Convert to robot coordinates
        print(f"Converting world coordinates to robot coordinates...")
        robot_path = []
        for i, (pixel_x, pixel_y) in enumerate(world_path):
            robot_coords = self.pixel_to_robot(pixel_x, pixel_y)
            if robot_coords:
                robot_path.append(robot_coords)
            else:
                if i < 3:  # Only show first few errors
                    print(f"⚠ Warning: Failed to convert pixel point {i}: ({pixel_x}, {pixel_y})")
        
        if not robot_path:
            print("⚠ Failed to convert path to robot coordinates")
            print(f"   World path had {len(world_path)} points but none converted")
            print(f"   Transform matrix exists: {self.transform_matrix is not None}")
            return
        
        print(f"✓ Converted to {len(robot_path)} robot coordinates")
        
        print(f"\nPath length: {len(robot_path)} points")
        print(f"Start: ({robot_path[0][0]:.2f}mm, {robot_path[0][1]:.2f}mm)")
        print(f"End: ({robot_path[-1][0]:.2f}mm, {robot_path[-1][1]:.2f}mm)")
        
        # Validate coordinates are within robot workspace (warning only, no prompt)
        for i, (x, y) in enumerate(robot_path):
            if abs(x) > 300 or abs(y) > 300:
                print(f"\n⚠ WARNING: Point {i+1} is outside safe workspace!")
                print(f"   Coordinates: X={x:.2f}mm, Y={y:.2f}mm")
                print("   Dobot safe range: X: ±300mm, Y: ±300mm")
                print("   Continuing anyway...")
                break
        
        print("\n▶ Starting robot movement...")
        
        current_point = None
        
        try:
            # Move to start position at safe height
            print(f"\n1. Moving to start position (safe height)...")
            current_point = (robot_path[0][0], robot_path[0][1], self.SAFE_Z)
            self.robot.move_to(current_point[0], current_point[1], current_point[2], 0, wait=True)
            
            # Lower to drawing height
            print(f"2. Lowering to drawing height ({self.DRAW_Z}mm)...")
            current_point = (robot_path[0][0], robot_path[0][1], self.DRAW_Z)
            self.robot.move_to(current_point[0], current_point[1], current_point[2], 0, wait=True)
            
            # Execute path at drawing height - continuous movement
            print(f"3. Following path ({len(robot_path)} points)...")
            print(f"   Executing continuous movement...")
            
            # Queue all movements without waiting (continuous motion)
            for i, (x, y) in enumerate(robot_path[1:], 1):
                current_point = (x, y, self.DRAW_Z)
                self.robot.move_to(x, y, self.DRAW_Z, 0, wait=False)
                if i % 50 == 0:
                    print(f"   Queued: {i}/{len(robot_path)-1} points")
            
            # Wait for all movements to complete
            print(f"   Waiting for robot to complete path...")
            time.sleep(0.5)  # Small delay to let queue process
            
            # Wait for the robot to finish by checking if it's moving
            while True:
                try:
                    # Check if robot is still moving
                    # The robot's pose will be updated as it moves
                    time.sleep(0.1)
                    # Try to get position - if successful and stable, we're done
                    pos1 = self.robot.pose()
                    time.sleep(0.2)
                    pos2 = self.robot.pose()
                    
                    # If position hasn't changed much, movement is complete
                    if abs(pos1[0] - pos2[0]) < 0.5 and abs(pos1[1] - pos2[1]) < 0.5:
                        break
                except:
                    break
            
            print(f"   ✓ Path complete")
            
            # Raise to safe height
            print(f"4. Raising to safe height...")
            current_point = (robot_path[-1][0], robot_path[-1][1], self.SAFE_Z)
            self.robot.move_to(current_point[0], current_point[1], current_point[2], 0, wait=True)
            
            # Return to viewing position
            print(f"5. Returning to viewing position...")
            current_point = (240, 0, 150)
            self.robot.move_to(240, 0, 150, 0, wait=True)
            
            print("\n✓ Path execution complete!")
        
        except Exception as e:
            print(f"\n⚠ Error during path execution: {e}")
            print(f"   Error type: {type(e).__name__}")
            
            # Try to return to safe height
            if self.robot is not None and current_point is not None:
                try:
                    print(f"\nAttempting emergency return to safe height...")
                    print(f"   Last known position: X={current_point[0]:.2f}mm, Y={current_point[1]:.2f}mm")
                    
                    # Try to get current position
                    try:
                        pos = self.robot.pose()
                        print(f"   Current position: X={pos[0]:.2f}mm, Y={pos[1]:.2f}mm, Z={pos[2]:.2f}mm")
                        self.robot.move_to(pos[0], pos[1], self.SAFE_Z, 0, wait=True)
                    except:
                        # If pose() fails, use last known position
                        self.robot.move_to(current_point[0], current_point[1], self.SAFE_Z, 0, wait=True)
                    
                    print("✓ Returned to safe height")
                    
                    # Try to return to viewing position
                    try:
                        self.robot.move_to(240, 0, 150, 0, wait=True)
                        print("✓ Returned to viewing position")
                    except:
                        print("⚠ Could not return to viewing position")
                        
                except Exception as recovery_error:
                    print(f"⚠ Emergency recovery failed: {recovery_error}")
                    print("   MANUAL INTERVENTION REQUIRED!")
                    print("   Please manually move the robot to a safe position.")
            else:
                print("⚠ Cannot perform emergency recovery (robot state unknown)")
                print("   MANUAL INTERVENTION REQUIRED!")
    
    def start_camera(self):
        """Initialize camera capture"""
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception(f"Cannot open camera at index {self.camera_index}")
        
        # Try to disable autofocus and set manual focus
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
        
        # Set to VGA resolution (640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Get actual resolution being used
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        if self.robot_enabled:
            print(f"Camera resolution: {int(actual_width)}x{int(actual_height)}")
            
        # Try to set field of view if camera supports it
        # Some cameras zoom in with certain resolutions
        try:
            self.cap.set(cv2.CAP_PROP_ZOOM, 100)  # 100 = no zoom (1x)
        except:
            pass
        
    def detect_maze_contour(self, frame):
        """Detect the outer boundary of the maze with rotation handling"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply adaptive thresholding
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 11, 2)
        
        # Morphological operations to close gaps (especially at entrances)
        kernel = np.ones((3, 3), np.uint8)
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, None
        
        # Find all significant maze contours (could be multiple disconnected parts)
        maze_contours = []
        min_area = 1500  # Minimum area threshold for maze parts (adjusted for VGA 640x480)
        
        # Debug: track largest contour for troubleshooting
        largest_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
            if area > min_area:
                maze_contours.append(contour)
        
        # If no contours found, try with lower threshold
        if not maze_contours and largest_area > 500:
            min_area = 500  # Fallback to lower threshold
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_area:
                    maze_contours.append(contour)
        
        if not maze_contours:
            return None, None, None
        
        # Combine all maze contours
        all_points = np.vstack(maze_contours)
        
        # Get ROTATED bounding rectangle (handles tilted mazes)
        rect = cv2.minAreaRect(all_points)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        # Get the center, size, and angle of the rotated rectangle
        center, (width, height), angle = rect
        
        # Ensure width > height (rotate if needed)
        if width < height:
            width, height = height, width
            angle += 90
        
        # Compute perspective transform to straighten the maze
        # Source points: the 4 corners of the rotated rectangle
        src_pts = box.astype(np.float32)
        
        # Destination points: axis-aligned rectangle
        dst_pts = np.array([
            [0, 0],
            [width - 1, 0],
            [width - 1, height - 1],
            [0, height - 1]
        ], dtype=np.float32)
        
        # Order the points consistently (top-left, top-right, bottom-right, bottom-left)
        src_pts = self.order_points(src_pts)
        
        # Compute perspective transformation matrix
        self.perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self.inverse_perspective = cv2.getPerspectiveTransform(dst_pts, src_pts)
        
        # For visualization, return the rotated box contour
        return box.reshape((-1, 1, 2)), (int(width), int(height)), thresh
    
    def order_points(self, pts):
        """Order points in consistent order: top-left, top-right, bottom-right, bottom-left"""
        # Sort by y-coordinate
        sorted_pts = pts[np.argsort(pts[:, 1])]
        
        # Get top 2 and bottom 2
        top_pts = sorted_pts[:2]
        bottom_pts = sorted_pts[2:]
        
        # Sort top points by x (left to right)
        top_pts = top_pts[np.argsort(top_pts[:, 0])]
        # Sort bottom points by x (right to left for correct order)
        bottom_pts = bottom_pts[np.argsort(bottom_pts[:, 0])[::-1]]
        
        # Return in order: TL, TR, BR, BL
        return np.array([top_pts[0], top_pts[1], bottom_pts[0], bottom_pts[1]], dtype=np.float32)
    
    def detect_colored_dots(self, frame, bbox):
        """Detect red and green entrance markers"""
        if bbox is None or self.perspective_matrix is None:
            return None, None
        
        w, h = bbox  # bbox now contains (width, height) of straightened maze
        w = int(w)
        h = int(h)
        
        # Apply perspective transform to straighten the frame
        warped = cv2.warpPerspective(frame, self.perspective_matrix, (w, h))
        roi = warped
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for red and green
        # Red has two ranges in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        # Create masks
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find centroids of red and green dots in warped image
        red_pos_warped = self.find_centroid(mask_red)
        green_pos_warped = self.find_centroid(mask_green)
        
        # Transform back to original frame coordinates using inverse perspective
        red_pos = None
        green_pos = None
        
        if red_pos_warped and self.inverse_perspective is not None:
            pt = np.array([[red_pos_warped]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(pt, self.inverse_perspective)
            red_pos = (int(transformed[0][0][0]), int(transformed[0][0][1]))
        
        if green_pos_warped and self.inverse_perspective is not None:
            pt = np.array([[green_pos_warped]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(pt, self.inverse_perspective)
            green_pos = (int(transformed[0][0][0]), int(transformed[0][0][1]))
        
        return red_pos, green_pos
    
    def find_centroid(self, mask):
        """Find the centroid of the largest blob in a mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) < 25:  # Minimum area threshold (adjusted for VGA)
            return None
        
        # Calculate centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return (cx, cy)
    
    def create_maze_grid(self, frame, bbox, thresh):
        """Convert maze image to a binary grid for pathfinding"""
        if bbox is None or self.perspective_matrix is None:
            return None
        
        w, h = bbox  # bbox now contains (width, height) of straightened maze
        w = int(w)
        h = int(h)
        
        # Apply perspective transform to straighten the maze
        warped_frame = cv2.warpPerspective(frame, self.perspective_matrix, (w, h))
        gray_region = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
        
        # Apply binary threshold to detect dark walls
        # Walls are dark (black), paths are light (gray background)
        # Invert so that walls are white (255) and paths are black (0) for easier logic
        _, binary = cv2.threshold(gray_region, 80, 255, cv2.THRESH_BINARY_INV)
        
        # Remove red and green dots from the binary image (they're not walls!)
        # Convert warped frame to HSV to detect colored markers
        hsv = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2HSV)
        
        # Red detection (same as in detect_colored_dots)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Green detection
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # Combine red and green masks
        color_dots_mask = cv2.bitwise_or(mask_red, mask_green)
        
        # Dilate the color mask slightly to ensure complete removal
        kernel_dilate = np.ones((5, 5), np.uint8)
        color_dots_mask = cv2.dilate(color_dots_mask, kernel_dilate, iterations=2)
        
        # Remove colored dots from binary (treat them as paths, not walls)
        # Where color_dots_mask is white (255), set binary to black (0 = path)
        binary[color_dots_mask > 0] = 0
        
        # 1. Morphologically clean the wall mask
        # This removes noise and solidifies walls for better distance calculation
        kernel_clean = np.ones((3, 3), np.uint8)
        binary_clean = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_clean, iterations=1)
        binary_clean = cv2.morphologyEx(binary_clean, cv2.MORPH_OPEN, kernel_clean, iterations=1)
        
        # Store the cleaned binary image for debugging
        self.debug_binary = binary_clean.copy()
        
        # 2. Distance transform on the cleaned binary PIXEL image (high resolution)
        # binary_clean: 255 = wall, 0 = path
        import scipy.ndimage
        path_mask = (binary_clean == 0).astype(np.uint8)
        distance_px = scipy.ndimage.distance_transform_edt(path_mask)
        
        # Store pixel-level distance for debugging
        self.debug_distance = distance_px.copy()
        
        # 3. Sample the high-res distance map into the denser grid
        grid_h = h // self.grid_size
        grid_w = w // self.grid_size
        
        # Create base grid (walkable vs wall)
        base_grid = np.zeros((grid_h, grid_w), dtype=np.uint8)
        
        # Create safe grid (with clearance requirement)
        safe_grid = np.zeros((grid_h, grid_w), dtype=np.uint8)
        
        for i in range(grid_h):
            for j in range(grid_w):
                y_start = i * self.grid_size
                y_end = min((i + 1) * self.grid_size, h)
                x_start = j * self.grid_size
                x_end = min((j + 1) * self.grid_size, w)
                
                if y_start < h and x_start < w:
                    # Get the cell region from cleaned binary image
                    cell_region = binary_clean[y_start:y_end, x_start:x_end]
                    
                    # Calculate the percentage of black (path) pixels
                    black_percentage = 1.0 - (np.mean(cell_region) / 255.0)
                    
                    # Base walkability (for fallback)
                    if black_percentage > 0.5:
                        base_grid[i, j] = 1  # Path (walkable)
                    else:
                        base_grid[i, j] = 0  # Wall (blocked)
                    
                    # Clearance-based walkability (primary)
                    # Use MAX distance in this cell from high-res distance map (max pooling)
                    # This is more conservative - cell is only safe if ALL points have clearance
                    cell_distance = np.max(distance_px[y_start:y_end, x_start:x_end])
                    
                    if cell_distance >= self.min_clearance_pixels:
                        safe_grid[i, j] = 1  # Safe to walk (has clearance)
                    else:
                        safe_grid[i, j] = 0  # Too close to walls
        
        # Store the base grid for fallback
        self.base_grid = base_grid.copy()
        
        return safe_grid
    
    def world_to_grid(self, pos, bbox):
        """Convert world coordinates to grid coordinates"""
        if pos is None or bbox is None or self.perspective_matrix is None:
            return None
        
        w, h = bbox
        px, py = pos
        
        # Transform world coordinates to warped (straightened) coordinates
        pt = np.array([[[px, py]]], dtype=np.float32)
        warped_pt = cv2.perspectiveTransform(pt, self.perspective_matrix)
        
        rel_x = int(warped_pt[0][0][0])
        rel_y = int(warped_pt[0][0][1])
        
        # Convert to grid coordinates
        grid_x = rel_x // self.grid_size
        grid_y = rel_y // self.grid_size
        
        return (grid_y, grid_x)  # Note: (row, col) format for array indexing
    
    def grid_to_world(self, grid_pos, bbox):
        """Convert grid coordinates to world coordinates"""
        if grid_pos is None or bbox is None or self.inverse_perspective is None:
            return None
        
        w, h = bbox
        row, col = grid_pos
        
        # Convert to warped (straightened) maze coordinates (center of grid cell)
        warped_x = col * self.grid_size + self.grid_size // 2
        warped_y = row * self.grid_size + self.grid_size // 2
        
        # Transform back to original frame coordinates using inverse perspective
        pt = np.array([[[warped_x, warped_y]]], dtype=np.float32)
        world_pt = cv2.perspectiveTransform(pt, self.inverse_perspective)
        
        world_x = int(world_pt[0][0][0])
        world_y = int(world_pt[0][0][1])
        
        return (world_x, world_y)
    
    def find_nearest_walkable(self, grid, grid_pos, max_distance=10):
        """Find the nearest walkable cell to a given position"""
        if grid is None or grid_pos is None:
            return None
        
        rows, cols = grid.shape
        start_row, start_col = grid_pos
        
        # Check if position is already valid
        if 0 <= start_row < rows and 0 <= start_col < cols:
            if grid[start_row, start_col] == 1:
                return grid_pos
        
        # BFS to find nearest walkable cell
        from collections import deque
        queue = deque([(start_row, start_col, 0)])
        visited = set()
        visited.add((start_row, start_col))
        
        while queue:
            row, col, dist = queue.popleft()
            
            # Check if we've exceeded max distance
            if dist > max_distance:
                break
            
            # Check if this is a walkable cell
            if 0 <= row < rows and 0 <= col < cols:
                if grid[row, col] == 1:
                    return (row, col)
            
            # Explore neighbors (8 directions for better results)
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    
                    new_row, new_col = row + dr, col + dc
                    
                    if (new_row, new_col) not in visited:
                        visited.add((new_row, new_col))
                        queue.append((new_row, new_col, dist + 1))
        
        # No walkable cell found
        return None
    
    def bfs_solve(self, grid, start, end):
        """Solve maze using Breadth-First Search"""
        if grid is None or start is None or end is None:
            return None
        
        rows, cols = grid.shape
        start_row, start_col = start
        end_row, end_col = end
        
        # Validate start and end positions
        if not (0 <= start_row < rows and 0 <= start_col < cols):
            return None
        if not (0 <= end_row < rows and 0 <= end_col < cols):
            return None
        
        # BFS initialization
        queue = deque([(start_row, start_col, [(start_row, start_col)])])
        visited = set()
        visited.add((start_row, start_col))
        
        # Directions: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        while queue:
            row, col, path = queue.popleft()
            
            # Check if we reached the goal
            if (row, col) == (end_row, end_col):
                return path
            
            # Explore neighbors
            for dr, dc in directions:
                new_row, new_col = row + dr, col + dc
                
                # Check bounds
                if not (0 <= new_row < rows and 0 <= new_col < cols):
                    continue
                
                # Check if already visited
                if (new_row, new_col) in visited:
                    continue
                
                # Check if it's a valid path (not a wall)
                if grid[new_row, new_col] == 0:
                    continue
                
                # Add to queue
                visited.add((new_row, new_col))
                new_path = path + [(new_row, new_col)]
                queue.append((new_row, new_col, new_path))
        
        # No path found
        return None
    
    def visualize_grid(self, grid, bbox):
        """Create a visualization of the grid for debugging"""
        if grid is None or bbox is None:
            return None
        
        w, h = bbox  # bbox now contains (width, height)
        rows, cols = grid.shape
        
        # Create an image to visualize the grid
        vis = np.zeros((int(h), int(w), 3), dtype=np.uint8)
        
        w = int(w)
        h = int(h)
        
        for i in range(rows):
            for j in range(cols):
                y_start = i * self.grid_size
                y_end = min((i + 1) * self.grid_size, h)
                x_start = j * self.grid_size
                x_end = min((j + 1) * self.grid_size, w)
                
                if grid[i, j] == 1:
                    # Path - white
                    vis[y_start:y_end, x_start:x_end] = (255, 255, 255)
                else:
                    # Wall - black
                    vis[y_start:y_end, x_start:x_end] = (0, 0, 0)
        
        # Draw grid lines
        for i in range(0, h, self.grid_size):
            cv2.line(vis, (0, i), (w, i), (128, 128, 128), 1)
        for j in range(0, w, self.grid_size):
            cv2.line(vis, (j, 0), (j, h), (128, 128, 128), 1)
        
        return vis
    
    def draw_solution(self, frame, path, bbox):
        """Draw the solution path on the frame"""
        if path is None or bbox is None or len(path) < 2:
            return frame
        
        result = frame.copy()
        
        # Convert path to world coordinates
        world_path = []
        for grid_pos in path:
            world_pos = self.grid_to_world(grid_pos, bbox)
            if world_pos:
                world_path.append(world_pos)
        
        # Draw the path
        for i in range(len(world_path) - 1):
            pt1 = world_path[i]
            pt2 = world_path[i + 1]
            cv2.line(result, pt1, pt2, (255, 0, 255), 3)  # Magenta line
        
        # Draw start and end markers
        if world_path:
            cv2.circle(result, world_path[0], 8, (0, 0, 255), -1)  # Red start
            cv2.circle(result, world_path[-1], 8, (0, 255, 0), -1)  # Green end
        
        return result
    
    def run(self):
        """Main loop"""
        self.start_camera()
        
        # Connect to robot if enabled
        if self.robot_enabled:
            self.connect_robot()
        
        print("\n" + "="*60)
        print("MAZE SOLVER WITH ROBOT CONTROL")
        print("="*60)
        print(f"\nDirection: {self.direction.replace('_', ' → ').upper()}")
        print(f"Minimum Clearance: {self.min_clearance_pixels} pixels (distance from walls)")
        print("\nControls:")
        print("  'q' - Quit")
        print("  's' - Save current frame")
        print("  'd' - Toggle debug mode")
        print("  'e' - Execute path with robot")
        print("  '+' - Increase clearance by 2px (path stays further from walls)")
        print("  '-' - Decrease clearance by 2px (path can get closer to walls)")
        print()
        
        debug_mode = False
        grid = None
        current_path = None
        current_bbox = None

        try:
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("Failed to grab frame")
                    break
                
                # Detect maze
                maze_contour, bbox, thresh = self.detect_maze_contour(frame)
                
                if maze_contour is not None and bbox is not None:
                    # Draw maze boundary
                    cv2.drawContours(frame, [maze_contour], -1, (0, 255, 255), 2)
                    
                    # Detect colored entrance markers
                    red_pos, green_pos = self.detect_colored_dots(frame, bbox)
                    
                    # Create maze grid
                    grid = self.create_maze_grid(frame, bbox, thresh)
                    
                    if grid is not None and red_pos and green_pos:
                        # Convert positions to grid coordinates
                        start_grid_raw = self.world_to_grid(red_pos, bbox)
                        end_grid_raw = self.world_to_grid(green_pos, bbox)
                        
                        # Find nearest walkable cells
                        start_grid = self.find_nearest_walkable(grid, start_grid_raw, max_distance=20)
                        end_grid = self.find_nearest_walkable(grid, end_grid_raw, max_distance=20)
                        
                        # Debug info
                        grid_h, grid_w = grid.shape
                        path_cells = np.sum(grid)
                        total_cells = grid_h * grid_w
                        
                        # Check if start and end are valid
                        start_valid = start_grid is not None
                        end_valid = end_grid is not None
                        
                        # Reverse path if going from green to red
                        if self.direction == 'green_to_red':
                            start_grid, end_grid = end_grid, start_grid
                        
                        # Solve maze with clearance-filtered grid
                        path = self.bfs_solve(grid, start_grid, end_grid)
                        
                        # Fallback: if no path found with clearance filter, try with base grid
                        # This is especially useful for tilted mazes where clearance is too restrictive
                        used_fallback = False
                        if path is None and hasattr(self, 'base_grid') and self.base_grid is not None:
                            # Retry with original grid (no clearance requirement)
                            path = self.bfs_solve(self.base_grid, start_grid, end_grid)
                            if path:
                                used_fallback = True
                        
                        if path:
                            # Store current path and bbox for robot execution
                            current_path = path
                            current_bbox = bbox
                            
                            # Draw solution
                            frame = self.draw_solution(frame, path, bbox)

                            # Display info
                            path_status = "Path Length: " + str(len(path))
                            if used_fallback:
                                path_status += " [No Clearance]"
                                color = (0, 200, 255)  # Orange color for fallback
                            else:
                                color = (0, 255, 0)  # Green for safe path
                            
                            cv2.putText(frame, path_status, (10, 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                            cv2.putText(frame, f"Grid: {grid_w}x{grid_h} | Safe: {path_cells}/{total_cells} | Clearance: {self.min_clearance_pixels}px", (10, 60),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                            
                            if used_fallback:
                                cv2.putText(frame, "⚠ Using basic path (may be close to walls)", (10, 90),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                            
                            if self.robot_enabled:
                                cv2.putText(frame, "Press 'e' to execute path", (10, 120),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        else:
                            cv2.putText(frame, "No path found!", (10, 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            cv2.putText(frame, f"Grid: {grid_w}x{grid_h} | Safe: {path_cells}/{total_cells} | Clearance: {self.min_clearance_pixels}px", (10, 60),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                            
                            # Show why path finding failed
                            if not start_valid:
                                cv2.putText(frame, "START on wall!", (10, 90),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                            if not end_valid:
                                cv2.putText(frame, "END on wall!", (10, 110),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        
                        # Draw entrance markers
                        if red_pos:
                            cv2.circle(frame, red_pos, 10, (0, 0, 255), 2)
                            cv2.putText(frame, "START", (red_pos[0] + 15, red_pos[1]),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            
                            # Draw adjusted start position if different
                            if start_grid and start_grid != start_grid_raw:
                                adjusted_pos = self.grid_to_world(start_grid, bbox)
                                if adjusted_pos:
                                    cv2.circle(frame, adjusted_pos, 6, (255, 0, 0), -1)
                                    cv2.line(frame, red_pos, adjusted_pos, (255, 0, 0), 2)
                        
                        if green_pos:
                            cv2.circle(frame, green_pos, 10, (0, 255, 0), 2)
                            cv2.putText(frame, "END", (green_pos[0] + 15, green_pos[1]),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            # Draw adjusted end position if different
                            if end_grid and end_grid != end_grid_raw:
                                adjusted_pos = self.grid_to_world(end_grid, bbox)
                                if adjusted_pos:
                                    cv2.circle(frame, adjusted_pos, 6, (0, 255, 255), -1)
                                    cv2.line(frame, green_pos, adjusted_pos, (0, 255, 255), 2)
                    
                    else:
                        cv2.putText(frame, "Detecting entrances...", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                else:
                    cv2.putText(frame, "No maze detected", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Show debug view if enabled
                if debug_mode:
                    debug_y_offset = 0
                    
                    # Show binary threshold image
                    if self.debug_binary is not None:
                        binary_color = cv2.cvtColor(self.debug_binary, cv2.COLOR_GRAY2BGR)
                        h_bin, w_bin = binary_color.shape[:2]
                        scale = min(300 / w_bin, 200 / h_bin)
                        new_w = int(w_bin * scale)
                        new_h = int(h_bin * scale)
                        binary_small = cv2.resize(binary_color, (new_w, new_h))
                        
                        # Add label
                        cv2.putText(binary_small, "Binary (White=Wall)", (5, 15),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                        
                        frame[debug_y_offset:debug_y_offset+new_h, 0:new_w] = binary_small
                        debug_y_offset += new_h + 10
                    
                    # Show grid visualization
                    if grid is not None and bbox is not None:
                        grid_vis = self.visualize_grid(grid, bbox)
                        if grid_vis is not None:
                            h_vis, w_vis = grid_vis.shape[:2]
                            scale = min(300 / w_vis, 200 / h_vis)
                            new_w = int(w_vis * scale)
                            new_h = int(h_vis * scale)
                            grid_vis_small = cv2.resize(grid_vis, (new_w, new_h))
                            
                            # Add label
                            cv2.putText(grid_vis_small, "Grid (White=Path)", (5, 15),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                            
                            if debug_y_offset + new_h < frame.shape[0]:
                                frame[debug_y_offset:debug_y_offset+new_h, 0:new_w] = grid_vis_small
                
                # Display frame
                cv2.imshow('Maze Solver', frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"maze_solution_{int(time.time())}.png"
                    cv2.imwrite(filename, frame)
                    print(f"Saved: {filename}")
                elif key == ord('d'):
                    debug_mode = not debug_mode
                    print(f"Debug mode: {'ON' if debug_mode else 'OFF'}")
                elif key == ord('e') and self.robot_enabled:
                    print("\n'e' key pressed - attempting to execute path...")
                    print(f"  Robot enabled: {self.robot_enabled}")
                    print(f"  Current path exists: {current_path is not None}")
                    print(f"  Current bbox exists: {current_bbox is not None}")
                    if current_path:
                        print(f"  Path length: {len(current_path)}")
                    if current_bbox:
                        print(f"  Bbox: {current_bbox}")
                    
                    if current_path and current_bbox:
                        self.execute_path(current_path, current_bbox)
                    else:
                        print("⚠ No valid path to execute!")
                elif key == ord('+') or key == ord('='):
                    self.min_clearance_pixels = min(self.min_clearance_pixels + 2, 100)
                    print(f"Minimum clearance: {self.min_clearance_pixels} pixels (path will stay further from walls)")
                elif key == ord('-') or key == ord('_'):
                    self.min_clearance_pixels = max(self.min_clearance_pixels - 2, 5)
                    print(f"Minimum clearance: {self.min_clearance_pixels} pixels (path can get closer to walls)")
        
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            if self.robot_enabled:
                self.disconnect_robot()

def main():
    """Main entry point with user prompts"""
    print("\n" + "="*60)
    print("MAZE SOLVER WITH DOBOT CONTROL")
    print("="*60)
    
    # Robot control is always enabled
    robot_enabled = True
    
    # Ask for direction
    print("\nWhich direction should the robot follow?")
    print("  1. Red (START) → Green (END)")
    print("  2. Green (END) → Red (START)")
    dir_choice = input("Enter 1 or 2 [1]: ").strip()
    
    if dir_choice == '2':
        direction = 'green_to_red'
    else:
        direction = 'red_to_green'
    
    print(f"\n✓ Direction set: {direction.replace('_', ' → ').upper()}")
    
    # Create and run solver
    solver = MazeSolver(camera_index=1, robot_enabled=robot_enabled)
    solver.direction = direction
    solver.run()

if __name__ == "__main__":
    main()

