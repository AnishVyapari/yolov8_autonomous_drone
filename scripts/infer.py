#!/usr/bin/env python3
"""
YOLOv8 Inference Script with DroneKit Integration

This script performs real-time object detection using YOLOv8 on a drone's camera feed.
It integrates with DroneKit to provide position data and enable autonomous responses.

Hardware Setup:
- Companion computer (Raspberry Pi 4, Jetson Nano, or similar)
- Camera module (USB or CSI camera)
- Pixhawk/APM flight controller connected via serial/USB
- MAVLink connection configured

Usage:
    python scripts/infer.py --config config/detector.yaml
"""

import argparse
import cv2
import yaml
import time
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: ultralytics not installed. Run: pip install ultralytics")
    exit(1)

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
except ImportError:
    print("WARNING: dronekit not installed. Run: pip install dronekit")
    print("Continuing without drone connection...")
    DRONEKIT_AVAILABLE = False
else:
    DRONEKIT_AVAILABLE = True


class DroneInference:
    """
    YOLOv8 inference engine with DroneKit integration for autonomous drones.
    """
    
    def __init__(self, config_path: str):
        """
        Initialize the inference engine.
        
        Args:
            config_path: Path to YAML configuration file
        """
        self.config = self._load_config(config_path)
        self.model = self._load_model()
        self.vehicle = None
        self.camera = None
        self.detection_history = []
        
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config
    
    def _load_model(self) -> YOLO:
        """Load YOLOv8 model from configuration."""
        model_path = self.config['model']['weights']
        print(f"Loading YOLOv8 model from {model_path}")
        model = YOLO(model_path)
        return model
    
    def connect_drone(self, connection_string: Optional[str] = None) -> bool:
        """
        Connect to drone via DroneKit.
        
        Args:
            connection_string: MAVLink connection string (e.g., '/dev/ttyACM0', 'udp:127.0.0.1:14550')
            
        Returns:
            True if connection successful, False otherwise
        """
        if not DRONEKIT_AVAILABLE:
            print("DroneKit not available, skipping drone connection")
            return False
            
        conn_str = connection_string or self.config.get('drone', {}).get('connection_string', '/dev/ttyACM0')
        baud_rate = self.config.get('drone', {}).get('baud_rate', 57600)
        
        try:
            print(f"Connecting to drone on {conn_str}...")
            self.vehicle = connect(conn_str, baud=baud_rate, wait_ready=True)
            print(f"Connected to drone. Mode: {self.vehicle.mode.name}")
            print(f"GPS: {self.vehicle.gps_0}")
            print(f"Battery: {self.vehicle.battery}")
            return True
        except Exception as e:
            print(f"Failed to connect to drone: {e}")
            return False
    
    def setup_camera(self, camera_id: Optional[int] = None) -> bool:
        """
        Initialize camera for video capture.
        
        Args:
            camera_id: Camera device ID (default: 0)
            
        Returns:
            True if camera initialized successfully
        """
        cam_id = camera_id or self.config['camera']['device_id']
        width = self.config['camera'].get('width', 640)
        height = self.config['camera'].get('height', 480)
        fps = self.config['camera'].get('fps', 30)
        
        print(f"Initializing camera {cam_id} at {width}x{height} @ {fps}fps")
        self.camera = cv2.VideoCapture(cam_id)
        
        if not self.camera.isOpened():
            print(f"ERROR: Failed to open camera {cam_id}")
            return False
        
        # Set camera properties
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera.set(cv2.CAP_PROP_FPS, fps)
        
        print("Camera initialized successfully")
        return True
    
    def run_inference(self, frame: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        Run YOLOv8 inference on a single frame.
        
        Args:
            frame: Input image as numpy array
            
        Returns:
            Tuple of (detections list, annotated frame)
        """
        conf_threshold = self.config['model'].get('confidence_threshold', 0.25)
        iou_threshold = self.config['model'].get('iou_threshold', 0.45)
        
        # Run inference
        results = self.model.predict(
            frame,
            conf=conf_threshold,
            iou=iou_threshold,
            verbose=False
        )
        
        detections = []
        annotated_frame = frame.copy()
        
        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract detection info
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = self.model.names[cls]
                
                detection = {
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'confidence': conf,
                    'class_id': cls,
                    'class_name': class_name,
                    'timestamp': time.time()
                }
                
                # Add GPS coordinates if available
                if self.vehicle and self.vehicle.location.global_relative_frame:
                    detection['gps'] = {
                        'lat': self.vehicle.location.global_relative_frame.lat,
                        'lon': self.vehicle.location.global_relative_frame.lon,
                        'alt': self.vehicle.location.global_relative_frame.alt
                    }
                
                detections.append(detection)
                
                # Draw bounding box and label
                cv2.rectangle(annotated_frame, 
                             (int(x1), int(y1)), (int(x2), int(y2)), 
                             (0, 255, 0), 2)
                label = f"{class_name} {conf:.2f}"
                cv2.putText(annotated_frame, label, 
                           (int(x1), int(y1) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return detections, annotated_frame
    
    def add_telemetry_overlay(self, frame: np.ndarray) -> np.ndarray:
        """
        Add drone telemetry data overlay to the frame.
        
        Args:
            frame: Input frame
            
        Returns:
            Frame with telemetry overlay
        """
        if not self.vehicle:
            return frame
        
        # Prepare telemetry text
        telemetry = [
            f"Mode: {self.vehicle.mode.name}",
            f"Alt: {self.vehicle.location.global_relative_frame.alt:.1f}m",
            f"Speed: {self.vehicle.groundspeed:.1f}m/s",
            f"Battery: {self.vehicle.battery.voltage:.1f}V ({self.vehicle.battery.level}%)",
            f"GPS: {self.vehicle.gps_0.satellites_visible} sats"
        ]
        
        # Draw telemetry on frame
        y_offset = 30
        for text in telemetry:
            cv2.putText(frame, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_offset += 25
        
        return frame
    
    def run(self, display: bool = True, save_output: bool = False):
        """
        Main inference loop.
        
        Args:
            display: Whether to display video output
            save_output: Whether to save output video
        """
        if not self.camera:
            print("ERROR: Camera not initialized")
            return
        
        print("Starting inference loop. Press 'q' to quit.")
        
        # Video writer setup if saving
        video_writer = None
        if save_output:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = self.config['camera'].get('fps', 30)
            width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            output_path = f"output_{int(time.time())}.mp4"
            video_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
            print(f"Saving output to {output_path}")
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Capture frame
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to capture frame")
                    break
                
                # Run inference
                detections, annotated_frame = self.run_inference(frame)
                
                # Add telemetry overlay
                if self.vehicle:
                    annotated_frame = self.add_telemetry_overlay(annotated_frame)
                
                # Calculate and display FPS
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                cv2.putText(annotated_frame, f"FPS: {fps:.1f}", 
                           (frame.shape[1] - 150, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Save frame if enabled
                if video_writer:
                    video_writer.write(annotated_frame)
                
                # Display frame
                if display:
                    cv2.imshow('YOLOv8 Drone Inference', annotated_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Quit signal received")
                        break
                
                # Print detections
                if detections:
                    print(f"Frame {frame_count}: Detected {len(detections)} objects")
                    for det in detections:
                        print(f"  - {det['class_name']}: {det['confidence']:.2f}")
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup(video_writer)
    
    def cleanup(self, video_writer=None):
        """Release resources."""
        print("Cleaning up...")
        
        if self.camera:
            self.camera.release()
        
        if video_writer:
            video_writer.release()
        
        if self.vehicle:
            self.vehicle.close()
        
        cv2.destroyAllWindows()
        print("Cleanup complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='YOLOv8 Inference with DroneKit')
    parser.add_argument('--config', type=str, default='config/detector.yaml',
                       help='Path to configuration file')
    parser.add_argument('--no-display', action='store_true',
                       help='Run without display (headless mode)')
    parser.add_argument('--save-output', action='store_true',
                       help='Save output video')
    parser.add_argument('--camera', type=int, default=None,
                       help='Camera device ID (overrides config)')
    parser.add_argument('--connection', type=str, default=None,
                       help='Drone connection string (overrides config)')
    
    args = parser.parse_args()
    
    print("="*50)
    print("YOLOv8 Autonomous Drone Inference")
    print("="*50)
    
    # Initialize inference engine
    engine = DroneInference(args.config)
    
    # Connect to drone (optional)
    if engine.config.get('drone', {}).get('enabled', False):
        engine.connect_drone(args.connection)
    
    # Setup camera
    if not engine.setup_camera(args.camera):
        print("Failed to initialize camera, exiting.")
        return 1
    
    # Run inference
    engine.run(display=not args.no_display, save_output=args.save_output)
    
    return 0


if __name__ == '__main__':
    exit(main())
