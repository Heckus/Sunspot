#!/usr/bin/env python3
"""
Headless Picamera2 Test Script
Tests picamera2 without any preview/GUI dependencies
"""

import sys
import time
import os
import traceback

# Disable any display-related imports
os.environ['DISPLAY'] = ''
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

def test_minimal_import():
    """Test minimal picamera2 import"""
    print("=" * 50)
    print("TESTING MINIMAL PICAMERA2 IMPORT")
    print("=" * 50)
    
    try:
        # Try importing core components only
        print("Importing libcamera...")
        import libcamera
        print("✓ libcamera imported successfully")
        
        print("Importing picamera2 core...")
        # Import specific components to avoid preview modules
        from picamera2.picamera2 import Picamera2
        from picamera2.configuration import CameraConfiguration
        print("✓ picamera2 core imported successfully")
        
        return True
        
    except Exception as e:
        print(f"✗ Import failed: {e}")
        traceback.print_exc()
        return False

def test_direct_libcamera():
    """Test direct libcamera access"""
    print("\n" + "=" * 50)
    print("TESTING DIRECT LIBCAMERA ACCESS")
    print("=" * 50)
    
    try:
        import libcamera
        
        # Create camera manager
        print("Creating camera manager...")
        cm = libcamera.CameraManager.singleton()
        
        # Get cameras
        print("Getting camera list...")
        cameras = cm.cameras
        print(f"Found {len(cameras)} cameras")
        
        for i, camera in enumerate(cameras):
            print(f"  Camera {i}: {camera.id}")
            print(f"    Properties: {camera.properties}")
        
        if len(cameras) > 0:
            print("✓ Direct libcamera access successful")
            return True
        else:
            print("✗ No cameras found via libcamera")
            return False
            
    except Exception as e:
        print(f"✗ Direct libcamera access failed: {e}")
        traceback.print_exc()
        return False

def test_headless_capture():
    """Test headless capture without preview"""
    print("\n" + "=" * 50)
    print("TESTING HEADLESS CAPTURE")
    print("=" * 50)
    
    try:
        # Force headless mode
        os.environ['PICAMERA2_HEADLESS'] = '1'
        
        # Import after setting environment
        from picamera2 import Picamera2
        import numpy as np
        
        print("Creating Picamera2 instance...")
        picam2 = Picamera2()
        
        print("Getting camera info...")
        camera_info = picam2.camera_properties
        print(f"Camera info: {camera_info}")
        
        print("Creating still configuration...")
        # Use still configuration instead of video
        config = picam2.create_still_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        print(f"Configuration: {config}")
        
        print("Configuring camera...")
        picam2.configure(config)
        
        print("Starting camera...")
        picam2.start()
        
        # Warm up
        time.sleep(2)
        
        print("Capturing frames...")
        for i in range(5):
            start_time = time.time()
            
            # Use capture_array for headless operation
            frame = picam2.capture_array()
            
            end_time = time.time()
            capture_time = end_time - start_time
            
            print(f"  Frame {i+1}: {frame.shape}, time: {capture_time:.4f}s")
            
            time.sleep(0.5)
        
        picam2.stop()
        picam2.close()
        
        print("✓ Headless capture successful!")
        return True
        
    except Exception as e:
        print(f"✗ Headless capture failed: {e}")
        traceback.print_exc()
        return False

def test_alternative_capture():
    """Test alternative capture method"""
    print("\n" + "=" * 50)
    print("TESTING ALTERNATIVE CAPTURE METHOD")
    print("=" * 50)
    
    try:
        # Try using capture_file method
        from picamera2 import Picamera2
        import tempfile
        import os
        
        print("Creating Picamera2 instance...")
        picam2 = Picamera2()
        
        print("Creating configuration...")
        config = picam2.create_still_configuration()
        picam2.configure(config)
        
        print("Starting camera...")
        picam2.start()
        
        # Warm up
        time.sleep(2)
        
        print("Testing capture_file method...")
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            tmp_path = tmp.name
        
        try:
            # Capture to file
            picam2.capture_file(tmp_path)
            
            # Check if file was created
            if os.path.exists(tmp_path):
                file_size = os.path.getsize(tmp_path)
                print(f"✓ Captured image to file: {tmp_path} ({file_size} bytes)")
                
                # Clean up
                os.unlink(tmp_path)
                
                picam2.stop()
                picam2.close()
                return True
            else:
                print("✗ Capture file not created")
                return False
                
        except Exception as e:
            print(f"✗ Capture file failed: {e}")
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            return False
        
    except Exception as e:
        print(f"✗ Alternative capture failed: {e}")
        traceback.print_exc()
        return False

def test_v4l2_fallback():
    """Test V4L2 direct access as fallback"""
    print("\n" + "=" * 50)
    print("TESTING V4L2 FALLBACK")
    print("=" * 50)
    
    try:
        import cv2
        
        print("Testing OpenCV camera access...")
        
        # Try different video device indices
        for device_id in [0, 1, 10, 11, 12]:
            print(f"Trying /dev/video{device_id}...")
            
            cap = cv2.VideoCapture(device_id)
            
            if cap.isOpened():
                print(f"✓ Opened /dev/video{device_id}")
                
                # Set resolution
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                
                # Try to capture a frame
                ret, frame = cap.read()
                
                if ret:
                    print(f"✓ Captured frame: {frame.shape}")
                    
                    # Test capture speed
                    start_time = time.time()
                    for i in range(10):
                        ret, frame = cap.read()
                        if not ret:
                            break
                    end_time = time.time()
                    
                    if ret:
                        fps = 10.0 / (end_time - start_time)
                        print(f"✓ V4L2 capture FPS: {fps:.1f}")
                        
                        cap.release()
                        return True
                    else:
                        print("✗ Failed to capture multiple frames")
                else:
                    print("✗ Failed to capture frame")
                
                cap.release()
            else:
                print(f"- /dev/video{device_id} not accessible")
        
        print("✗ No V4L2 devices accessible")
        return False
        
    except Exception as e:
        print(f"✗ V4L2 fallback failed: {e}")
        traceback.print_exc()
        return False

def main():
    print("HEADLESS PICAMERA2 TEST")
    print("Testing picamera2 without preview dependencies")
    print("=" * 70)
    
    # Run tests in order of complexity
    tests = [
        ("Minimal Import", test_minimal_import),
        ("Direct libcamera", test_direct_libcamera),
        ("Headless Capture", test_headless_capture),
        ("Alternative Capture", test_alternative_capture),
        ("V4L2 Fallback", test_v4l2_fallback),
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            result = test_func()
            results[test_name] = result
            if result:
                print(f"✓ {test_name} PASSED")
                # If this test passed, we might not need the others
                if test_name in ["Headless Capture", "Alternative Capture"]:
                    print(f"\n🎉 SUCCESS: {test_name} worked! Camera is functional.")
                    break
            else:
                print(f"✗ {test_name} FAILED")
        except Exception as e:
            print(f"✗ {test_name} failed with exception: {e}")
            results[test_name] = False
    
    # Summary
    print("\n" + "=" * 70)
    print("FINAL RESULTS")
    print("=" * 70)
    
    working_methods = [name for name, result in results.items() if result]
    
    if working_methods:
        print(f"✓ Working methods: {', '.join(working_methods)}")
        print("\nRECOMMENDATIONS:")
        
        if "Headless Capture" in working_methods:
            print("- Use picamera2 with headless configuration")
            print("- Set PICAMERA2_HEADLESS=1 environment variable")
            print("- Use capture_array() for fastest operation")
        elif "Alternative Capture" in working_methods:
            print("- Use picamera2 with capture_file() method")
            print("- Consider converting files to arrays if needed")
        elif "V4L2 Fallback" in working_methods:
            print("- Use OpenCV with V4L2 backend")
            print("- This will be slower than picamera2 but should work")
            
    else:
        print("✗ No working camera methods found")
        print("\nTROUBLESHOoting:")
        print("1. Install missing dependencies:")
        print("   pip install picamera2 libcamera")
        print("2. Check container privileges:")
        print("   docker run --privileged ...")
        print("3. Mount camera devices:")
        print("   --device=/dev/video0 --device=/dev/video1 ...")
        print("4. Try different base image (Ubuntu 22.04 recommended)")
    
    return 0 if working_methods else 1

if __name__ == "__main__":
    sys.exit(main())
