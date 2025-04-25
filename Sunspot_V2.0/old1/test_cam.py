#!/usr/bin/env python3

print("Attempting to import Picamera2...")
try:
    from picamera2 import Picamera2
    print("Import successful!")
    # Optional: Try listing cameras
    # cameras = Picamera2.global_camera_info()
    # print("Available cameras:", cameras)
except ImportError as e:
    print(f"Import failed: {e}")
    import sys
    import pprint
    print("\nPython sys.path:")
    pprint.pprint(sys.path)
    print("\nExiting due to import error.")
    exit(1)

print("Minimal script finished.")