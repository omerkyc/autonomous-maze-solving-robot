#!/usr/bin/env python3
"""
Quick camera test utility to check zoom and resolution settings
"""

import cv2
import sys

def test_camera(camera_index=1):
    """Test camera and try different settings"""
    
    print("="*60)
    print("CAMERA TEST UTILITY")
    print("="*60)
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ Cannot open camera at index {camera_index}")
        sys.exit(1)
    
    print(f"\n✓ Camera {camera_index} opened successfully\n")
    
    # Get default settings
    print("Default Camera Properties:")
    print(f"  Width:  {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
    print(f"  Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"  FPS:    {cap.get(cv2.CAP_PROP_FPS)}")
    print(f"  Autofocus: {cap.get(cv2.CAP_PROP_AUTOFOCUS)}")
    print(f"  Zoom:   {cap.get(cv2.CAP_PROP_ZOOM)}")
    print(f"  Focus:  {cap.get(cv2.CAP_PROP_FOCUS)}")
    
    # Common resolutions to try
    resolutions = [
        (640, 480, "VGA"),
        (800, 600, "SVGA"),
        (1024, 768, "XGA"),
        (1280, 720, "HD 720p"),
        (1920, 1080, "HD 1080p"),
    ]
    
    print("\n" + "="*60)
    print("Testing Resolutions (press 'q' to quit, 'n' for next)")
    print("="*60)
    
    for width, height, name in resolutions:
        # Try to set resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Disable autofocus and zoom
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_ZOOM, 100)
        
        # Get actual resolution
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        print(f"\n{name} - Requested: {width}x{height}, Actual: {actual_w}x{actual_h}")
        
        if actual_w != width or actual_h != height:
            print(f"  ⚠ Camera adjusted to closest supported resolution")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("  ❌ Failed to capture frame")
                break
            
            # Display info on frame
            info_text = f"{name}: {actual_w}x{actual_h}"
            cv2.putText(frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, "Press 'n' for next, 'q' to quit", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow('Camera Test', frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                print("\nTest completed.")
                return
            elif key == ord('n'):
                break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n" + "="*60)
    print("Test completed. All resolutions tested.")
    print("="*60)
    print("\nRecommendation:")
    print("  Use the resolution that showed the widest field of view")
    print("  without zooming in on the subject.")

if __name__ == "__main__":
    camera_idx = 1
    
    if len(sys.argv) > 1:
        try:
            camera_idx = int(sys.argv[1])
        except:
            print("Usage: python test_camera.py [camera_index]")
            print("Default camera index is 1")
            sys.exit(1)
    
    test_camera(camera_idx)

