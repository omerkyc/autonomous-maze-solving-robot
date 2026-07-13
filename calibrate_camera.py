import cv2
import numpy as np
import json
import sys

class CameraCalibrator:
    def __init__(self, camera_index=1, robot_port='/dev/cu.usbmodem101'):
        self.camera_index = camera_index
        self.robot_port = robot_port
        self.cap = None
        self.robot = None
        self.pixel_points = []
        self.robot_points = []
        self.current_frame = None
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to record calibration points"""
        if event == cv2.EVENT_LBUTTONDOWN and len(self.pixel_points) < 4:
            self.pixel_points.append([x, y])
            print(f"\n✓ Point {len(self.pixel_points)} recorded at pixel: ({x}, {y})")
            
            # Draw the point on the frame
            cv2.circle(self.current_frame, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(self.current_frame, str(len(self.pixel_points)), (x + 10, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    def connect_robot(self):
        """Connect to the Dobot robot"""
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
            
            return True
        
        except Exception as e:
            print(f"\n⚠ Error connecting to robot: {e}")
            print("Please check:")
            print("  1. Robot is powered on")
            print("  2. USB cable is connected")
            print("  3. Port is correct: {self.robot_port}")
            sys.exit(1)
    
    def disconnect_robot(self):
        """Disconnect from the robot"""
        if self.robot:
            try:
                # Return to viewing position before disconnecting
                print("\nReturning to viewing position...")
                self.robot.move_to(240, 0, 150, 0, wait=True)
                self.robot.close()
                print("✓ Robot disconnected")
            except:
                pass
    
    def start_camera(self):
        """Initialize camera capture"""
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception(f"Cannot open camera at index {self.camera_index}")
        
        # Try to disable autofocus
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        
        # Set to VGA resolution (640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Get actual resolution
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        # Try to disable zoom
        try:
            self.cap.set(cv2.CAP_PROP_ZOOM, 100)
        except:
            pass
        
        print(f"✓ Camera started successfully!")
        print(f"  Resolution: {int(actual_width)}x{int(actual_height)}")
    
    def capture_calibration_frame(self):
        """Capture a single frame for calibration"""
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture frame")
        
        self.current_frame = frame.copy()
        return frame
    
    def collect_pixel_points(self):
        """Collect 4 pixel points by mouse clicking"""
        print("\n" + "="*60)
        print("PIXEL POINT SELECTION")
        print("="*60)
        print("\nInstructions:")
        print("1. Click on 4 points anywhere on the image")
        print("2. Choose points that cover your workspace area")
        print("3. Points can be in any order")
        print("4. Press 'r' to reset if you make a mistake")
        print("5. Press 'c' to continue once all 4 points are selected")
        print("\nWaiting for frame capture...")
        
        # Capture a reference frame
        frame = self.capture_calibration_frame()
        
        cv2.namedWindow('Calibration - Click 4 Points')
        cv2.setMouseCallback('Calibration - Click 4 Points', self.mouse_callback)
        
        while True:
            display_frame = self.current_frame.copy()
            
            # Draw existing points
            for i, pt in enumerate(self.pixel_points):
                cv2.circle(display_frame, tuple(pt), 5, (0, 255, 0), -1)
                cv2.putText(display_frame, str(i+1), (pt[0] + 10, pt[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw lines between points
            if len(self.pixel_points) >= 2:
                for i in range(len(self.pixel_points)):
                    pt1 = tuple(self.pixel_points[i])
                    pt2 = tuple(self.pixel_points[(i + 1) % len(self.pixel_points)])
                    cv2.line(display_frame, pt1, pt2, (255, 0, 0), 2)
            
            # Show instructions
            status = f"Points: {len(self.pixel_points)}/4 | Press 'r' to reset, 'c' to continue"
            cv2.putText(display_frame, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Calibration - Click 4 Points', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('r'):
                # Reset points
                self.pixel_points = []
                self.current_frame = frame.copy()
                print("\n⟲ Points reset. Click again.")
            
            elif key == ord('c'):
                if len(self.pixel_points) == 4:
                    break
                else:
                    print(f"\n⚠ Need 4 points, only {len(self.pixel_points)} selected!")
            
            elif key == ord('q'):
                cv2.destroyAllWindows()
                sys.exit(0)
        
        cv2.destroyAllWindows()
        print(f"\n✓ All 4 pixel points collected!")
    
    def collect_robot_points(self):
        """Automatically read robot coordinates for each point"""
        print("\n" + "="*60)
        print("ROBOT COORDINATE COLLECTION")
        print("="*60)
        print("\nInstructions:")
        print("1. Manually move the Dobot to each point marked in the image")
        print("2. Move the robot tip to touch the EXACT location of each pixel point")
        print("3. Press ENTER to automatically record the robot's position")
        print("4. Repeat for all 4 points\n")
        
        for i, pixel_pt in enumerate(self.pixel_points):
            print(f"\n--- Point {i+1}/4 ---")
            print(f"    Pixel coordinates: ({pixel_pt[0]}, {pixel_pt[1]})")
            print(f"\n    → Move robot to this point now")
            print(f"    → Position the tip at the exact location")
            
            while True:
                try:
                    input(f"    Press ENTER when robot is in position... ")
                    
                    # Read current robot position
                    pos = self.robot.pose()
                    x, y = pos[0], pos[1]
                    
                    self.robot_points.append([x, y])
                    print(f"    ✓ Robot point {i+1} recorded: X={x:.2f}mm, Y={y:.2f}mm, Z={pos[2]:.2f}mm")
                    break
                
                except KeyboardInterrupt:
                    print("\n\nCalibration cancelled.")
                    sys.exit(0)
                except Exception as e:
                    print(f"    ⚠ Error reading robot position: {e}")
                    print("    Try again...")
        
        print(f"\n✓ All 4 robot points collected!")
    
    def compute_transformation(self):
        """Compute perspective transformation matrix"""
        print("\n" + "="*60)
        print("COMPUTING TRANSFORMATION")
        print("="*60)
        
        # Convert to numpy arrays
        pixel_pts = np.array(self.pixel_points, dtype=np.float32)
        robot_pts = np.array(self.robot_points, dtype=np.float32)
        
        # Compute perspective transformation matrix
        # This maps pixel coordinates to robot coordinates
        self.transform_matrix = cv2.getPerspectiveTransform(pixel_pts, robot_pts)
        
        print("\n✓ Transformation matrix computed!")
        print("\nTransformation Matrix:")
        print(self.transform_matrix)
        
        # Verify transformation
        print("\n--- Verification ---")
        for i in range(4):
            pixel_pt = np.array([self.pixel_points[i] + [1]], dtype=np.float32).T
            robot_pt_computed = self.transform_matrix @ pixel_pt
            robot_pt_computed = robot_pt_computed[:2] / robot_pt_computed[2]
            
            error_x = abs(robot_pt_computed[0, 0] - self.robot_points[i][0])
            error_y = abs(robot_pt_computed[1, 0] - self.robot_points[i][1])
            
            print(f"Point {i+1}:")
            print(f"  Expected: ({self.robot_points[i][0]:.2f}, {self.robot_points[i][1]:.2f})")
            print(f"  Computed: ({robot_pt_computed[0, 0]:.2f}, {robot_pt_computed[1, 0]:.2f})")
            print(f"  Error: ({error_x:.2f}, {error_y:.2f}) mm")
    
    def save_calibration(self, filename='calibration.json'):
        """Save calibration data to file"""
        calibration_data = {
            'pixel_points': self.pixel_points,
            'robot_points': self.robot_points,
            'transform_matrix': self.transform_matrix.tolist(),
            'camera_index': self.camera_index
        }
        
        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"\n✓ Calibration saved to: {filename}")
    
    def run(self):
        """Run the complete calibration process"""
        print("\n" + "="*60)
        print("DOBOT CAMERA CALIBRATION")
        print("="*60)
        print("\nThis tool will create a mapping between camera pixels")
        print("and robot coordinates for precise maze solving.\n")
        
        try:
            # Connect to robot first
            self.connect_robot()
            
            # Start camera
            self.start_camera()
            
            # Collect pixel points
            self.collect_pixel_points()
            
            # Collect corresponding robot points
            self.collect_robot_points()
            
            # Compute transformation
            self.compute_transformation()
            
            # Save calibration
            self.save_calibration()
            
            print("\n" + "="*60)
            print("CALIBRATION COMPLETE!")
            print("="*60)
            print("\nYou can now run the maze solver with robot control.")
            print("The calibration file 'calibration.json' will be used automatically.\n")
        
        finally:
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            self.disconnect_robot()

if __name__ == "__main__":
    calibrator = CameraCalibrator(camera_index=1)
    calibrator.run()

